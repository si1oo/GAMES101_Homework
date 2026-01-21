// clang-format off
//
// Created by goksu on 4/6/19.
//

#include <algorithm>
#include <vector>
#include "rasterizer.hpp"
#include <opencv2/opencv.hpp>
#include <math.h>

rst::pos_buf_id rst::rasterizer::load_positions(const std::vector<Eigen::Vector3f> &positions)
{
    auto id = get_next_id();
    pos_buf.emplace(id, positions);

    return {id};
}

rst::ind_buf_id rst::rasterizer::load_indices(const std::vector<Eigen::Vector3i> &indices)
{
    auto id = get_next_id();
    ind_buf.emplace(id, indices);

    return {id};
}

rst::col_buf_id rst::rasterizer::load_colors(const std::vector<Eigen::Vector3f> &cols)
{
    auto id = get_next_id();
    col_buf.emplace(id, cols);

    return {id};
}

auto to_vec4(const Eigen::Vector3f& v3, float w = 1.0f)
{
    return Vector4f(v3.x(), v3.y(), v3.z(), w);
}

static bool insideTriangle(float a,float b,float c)
{   
    //重心坐标法
	return a >= 0 && b >= 0 && c >= 0;
}

static std::tuple<float, float, float> computeBarycentric2D(float x, float y, const Vector3f* v)
{
    float c1 = (x*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*y + v[1].x()*v[2].y() - v[2].x()*v[1].y()) / (v[0].x()*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*v[0].y() + v[1].x()*v[2].y() - v[2].x()*v[1].y());
    float c2 = (x*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*y + v[2].x()*v[0].y() - v[0].x()*v[2].y()) / (v[1].x()*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*v[1].y() + v[2].x()*v[0].y() - v[0].x()*v[2].y());
    float c3 = (x*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*y + v[0].x()*v[1].y() - v[1].x()*v[0].y()) / (v[2].x()*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*v[2].y() + v[0].x()*v[1].y() - v[1].x()*v[0].y());
    return {c1,c2,c3};
}

void rst::rasterizer::draw(pos_buf_id pos_buffer, ind_buf_id ind_buffer, col_buf_id col_buffer, Primitive type)
{
    auto& buf = pos_buf[pos_buffer.pos_id];
    auto& ind = ind_buf[ind_buffer.ind_id];
    auto& col = col_buf[col_buffer.col_id];

    float f1 = (50 - 0.1) / 2.0;
    float f2 = (50 + 0.1) / 2.0;

    Eigen::Matrix4f mvp = projection * view * model;
	for (auto& i : ind) //循环每个三角形
    {
        Triangle t;
        Eigen::Vector4f v[] = {
                mvp * to_vec4(buf[i[0]], 1.0f),
                mvp * to_vec4(buf[i[1]], 1.0f),
                mvp * to_vec4(buf[i[2]], 1.0f)
        };
        //Homogeneous division
        for (auto& vec : v) {
            vec /= vec.w();
        }
        //Viewport transformation
        for (auto & vert : v)
        {
            vert.x() = 0.5*width*(vert.x()+1.0);
            vert.y() = 0.5*height*(vert.y()+1.0);
            vert.z() = vert.z() * f1 + f2;
        }

        for (int i = 0; i < 3; ++i)
        {
            t.setVertex(i, v[i].head<3>());
        }

        auto col_x = col[i[0]];
        auto col_y = col[i[1]];
        auto col_z = col[i[2]];

        t.setColor(0, col_x[0], col_x[1], col_x[2]);
        t.setColor(1, col_y[0], col_y[1], col_y[2]);
        t.setColor(2, col_z[0], col_z[1], col_z[2]);

        rasterize_triangle(t);
    }
}

//Screen space rasterization
void rst::rasterizer::rasterize_triangle(const Triangle& t) { //此时三角形的坐标已经变换到屏幕空间
    auto v = t.toVector4();
    
    //获得包围盒（bounding box）
	float minX = static_cast<int>(std::floor(std::min(v[0].x(), std::min(v[1].x(), v[2].x()))));
	float maxX = static_cast<int>(std::ceil(std::max(v[0].x(), std::max(v[1].x(), v[2].x()))));
	float minY = static_cast<int>(std::floor(std::min(v[0].y(), std::min(v[1].y(), v[2].y()))));
	float maxY = static_cast<int>(std::ceil(std::max(v[0].y(), std::max(v[1].y(), v[2].y()))));

    if (ssaa_factor == 1) {

        //遍历包围盒内的每个像素并设置像素颜色

        for (int i = minX; i <= maxX; ++i) {
            for (int j = minY; j <= maxY; ++j) {

                float pixel_x = static_cast<float>(i) + 0.5;
                float pixel_y = static_cast<float>(j) + 0.5;

                auto [alpha, beta, gamma] = computeBarycentric2D(pixel_x, pixel_y, t.v);

                if (!insideTriangle(alpha, beta, gamma)) continue; //不在三角形内部直接跳过

                float w_reciprocal = 1.0 / (alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                z_interpolated *= w_reciprocal; //透视矫正

                if (depth_buf[get_index(i, j)] <= z_interpolated) continue; //GL_LESS

                depth_buf[get_index(i, j)] = z_interpolated;
                //这里在设置像素颜色时设置的是(i,j)（像素左下角）而非(pixel_x,pixel_y)（像素中心）
                //(pixel_x,pixel_y)未得到正确结果，尚未知晓为什么
                set_pixel(Eigen::Vector3f(i, j, z_interpolated), t.getColor());
            }
        }

        return;
    }
    else if(ssaa_factor > 1){
        superSamping(maxX, maxY, minX, minY, t, v);

        downSampling();

        return;
    }
}

void rst::rasterizer::superSamping(float maxX, float maxY, float minX, float minY, const Triangle& t, std::array<Eigen::Vector4f, 3Ui64>& v)
{
    //SSAA 

    //获得SSAA包围盒（bounding box）
    int ssaa_maxX = maxX * ssaa_factor;
    int ssaa_maxY = maxY * ssaa_factor;
    int ssaa_minX = minX * ssaa_factor;
    int ssaa_minY = minY * ssaa_factor;

    for (int sx = ssaa_minX; sx <= ssaa_maxX; ++sx) {
        for (int sy = ssaa_minY; sy <= ssaa_maxY; ++sy) {

            //超采样，每个像素内四个样本记录数据相同(一个像素分成四等分)

            //计算原本像素中心坐标
            float pixel_x = static_cast<float>(sx) / ssaa_factor + 0.5;
            float pixel_y = static_cast<float>(sy) / ssaa_factor + 0.5;

            auto [alpha, beta, gamma] = computeBarycentric2D(pixel_x, pixel_y, t.v);

            if (!insideTriangle(alpha, beta, gamma)) continue;

            float w_reciprocal = 1.0 / (alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
            float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
            z_interpolated *= w_reciprocal;

            int ssaa_index = get_ssaa_index(sx, sy);

            if (ssaa_depth_buf[ssaa_index] <= z_interpolated) continue;

            ssaa_depth_buf[ssaa_index] = z_interpolated;
            ssaa_frame_buf[ssaa_index] = t.getColor();
        }
    }
}

void rst::rasterizer::downSampling() {

    for (int i = 0; i < height; ++i) {
        for (int j = 0; j < width; ++j) {
            Eigen::Vector3f color_sum(0, 0, 0);
            //每个像素内四个样本求平均
            for (int si = 0; si < ssaa_factor; ++si) {
                for (int sj = 0; sj < ssaa_factor; ++sj) {
                    int ssaa_x = j * ssaa_factor + sj;
                    int ssaa_y = i * ssaa_factor + si;
                    int ssaa_index = get_ssaa_index(ssaa_x, ssaa_y);
                    color_sum += ssaa_frame_buf[ssaa_index];
                }
            }
            color_sum /= (ssaa_factor * ssaa_factor);
            set_pixel(Eigen::Vector3f(j, i, 0), color_sum);
        }
	}
}

void rst::rasterizer::set_model(const Eigen::Matrix4f& m)
{
    model = m;
}

void rst::rasterizer::set_view(const Eigen::Matrix4f& v)
{
    view = v;
}

void rst::rasterizer::set_projection(const Eigen::Matrix4f& p)
{
    projection = p;
}

void rst::rasterizer::clear(rst::Buffers buff)
{
    if ((buff & rst::Buffers::Color) == rst::Buffers::Color)
    {
        std::fill(frame_buf.begin(), frame_buf.end(), Eigen::Vector3f{0, 0, 0});
		std::fill(ssaa_frame_buf.begin(), ssaa_frame_buf.end(), Eigen::Vector3f{ 0,0,0 });
    }
    if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth)
    {
        std::fill(depth_buf.begin(), depth_buf.end(), std::numeric_limits<float>::infinity());
		std::fill(ssaa_depth_buf.begin(), ssaa_depth_buf.end(), std::numeric_limits<float>::infinity());
    }

}

rst::rasterizer::rasterizer(int w, int h,int ssaa_factor) : width(w), height(h)
{
	this->ssaa_factor = ssaa_factor;

    frame_buf.resize(w * h);
    depth_buf.resize(w * h);

	//增加SSAA所需的frame buffer和depth buffer
	ssaa_depth_buf.resize(w * ssaa_factor * h * ssaa_factor);
	ssaa_frame_buf.resize(w * ssaa_factor * h * ssaa_factor);
}

int rst::rasterizer::get_index(int x, int y)
{
    return (height-1-y)*width + x;
}

int rst::rasterizer::get_ssaa_index(int x, int y)
{
    return (height * ssaa_factor - 1 - y) * (width * ssaa_factor) + x;
}

void rst::rasterizer::set_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color)
{
    //old index: auto ind = point.y() + point.x() * width;
    auto ind = (height-1-point.y())*width + point.x();
    frame_buf[ind] = color;

}

// clang-format on