#include <chrono>
#include <iostream>
#include <opencv2/opencv.hpp>

std::vector<cv::Point2f> control_points; //贝塞尔曲线控制点

const int control_point_counts = 4; //控制点个数

void mouse_handler(int event, int x, int y, int flags, void *userdata)
{
    if (event == cv::EVENT_LBUTTONDOWN && control_points.size() < control_point_counts) 
    {
        std::cout << "Left button of the mouse is clicked - position (" << x << ", "
        << y << ")" << '\n';
        control_points.emplace_back(x, y);
    }     
}

void naive_bezier(const std::vector<cv::Point2f> &points, cv::Mat &window) //该方法仅计算4个控制点的贝塞尔曲线
{
    auto &p_0 = points[0];
    auto &p_1 = points[1];
    auto &p_2 = points[2];
    auto &p_3 = points[3];

    for (double t = 0.0; t <= 1.0; t += 0.001) 
    {
        //多项式方程
        auto point = std::pow(1 - t, 3) * p_0 + 3 * t * std::pow(1 - t, 2) * p_1 +
                 3 * std::pow(t, 2) * (1 - t) * p_2 + std::pow(t, 3) * p_3;

        window.at<cv::Vec3b>(point.y, point.x)[2] = 255; //set color.r = 255 (顺序为BGR)
    }
}

cv::Point2f recursive_bezier(const std::vector<cv::Point2f> &control_points, float t) //迭代函数，返回步长为t时的单个屏幕坐标
{
    //递归
    if (control_points.size() == 1)
        return control_points[0];

    std::vector<cv::Point2f> sub_control_points;

    for (int i = 1; i < control_points.size(); ++i) {
        auto control_point = t * control_points[i - 1] + (1 - t) * control_points[i];
        sub_control_points.push_back(control_point);
    }

    return recursive_bezier(sub_control_points, t);

}

float getDistanceWeigtht(const cv::Point2f& current_point, float pixel_x, float pixel_y) {
    float center_x = pixel_x + 0.5f;
    float center_y = pixel_y + 0.5f;

    //计算距离
    float dx = current_point.x - center_x;
    float dy = current_point.y - center_y;
    float distance = std::sqrt(dx * dx + dy * dy);

    //高斯模糊
    float sigma = 0.5f;
    float weight = exp(-distance * distance / (2 * sigma * sigma));

    return std::max(0.0f, weight);
}

void antiAliasPoint(const cv::Point2f& current_point, cv::Mat& window) {
    float base_x = std::floor(current_point.x);
    float base_y = std::floor(current_point.y);

    for (int dx = 0; dx <= 1; ++dx) {
        for (int dy = 0; dy <= 1; ++dy) {

            float pixel_x = base_x + dx;
            float pixel_y = base_y + dy;

            if (pixel_x < 0 || pixel_x >= 700 || pixel_y < 0 || pixel_y >= 700)
                continue;

            float weight = getDistanceWeigtht(current_point, pixel_x, pixel_y);

            cv::Vec3b& pixel = window.at<cv::Vec3b>(pixel_y, pixel_x);
            pixel[1] = std::min(255, pixel[1] + static_cast<uchar>(weight * 255.0f));
        }
    }
}

void bezier(const std::vector<cv::Point2f> &control_points, cv::Mat &window) 
{
    for (double t = 0.0; t <= 1.0; t += 0.001) {
        auto line_point = recursive_bezier(control_points, t);

         antiAliasPoint(line_point, window);
         // window.at<cv::Vec3b>(line_point.y, line_point.x)[1] = 255;
    }
}
int main() 
{
    cv::Mat window = cv::Mat(700, 700, CV_8UC3, cv::Scalar(0));
    cv::cvtColor(window, window, cv::COLOR_BGR2RGB);
    cv::namedWindow("Bezier Curve", cv::WINDOW_AUTOSIZE);

    cv::setMouseCallback("Bezier Curve", mouse_handler, nullptr);

    int key = -1;
    while (key != 27) 
    {
        for (auto &point : control_points) 
        {
            cv::circle(window, point, 3, {255, 255, 255}, 3); //绘制控制点
        }

        if (control_points.size() == control_point_counts) 
        {
             //naive_bezier(control_points, window);
               bezier(control_points, window);

            cv::imshow("Bezier Curve", window);
            cv::imwrite("my_bezier_curve.png", window);
            key = cv::waitKey(0);

            return 0;
        }

        cv::imshow("Bezier Curve", window);
        key = cv::waitKey(20);
    }

    return 0;
}
