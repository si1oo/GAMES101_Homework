//
// Created by LEI XU on 4/27/19.
//

#ifndef RASTERIZER_TEXTURE_H
#define RASTERIZER_TEXTURE_H
#include "global.hpp"
#include "Eigen/Dense"
#include <opencv2/opencv.hpp>
class Texture{
private:
    cv::Mat image_data;

public:
    Texture(const std::string& name)
    {
        image_data = cv::imread(name);
        cv::cvtColor(image_data, image_data, cv::COLOR_RGB2BGR);
        width = image_data.cols;
        height = image_data.rows;
    }

    int width, height;

    Eigen::Vector3f getColor(float u, float v)
    {
        //get uv in [0,1]
        u = std::clamp(u, 0.0f, 1.0f);
        v = std::clamp(v, 0.0f, 1.0f);

        //get img_uv in [0,width - 1 / height - 1]
        auto u_img = u * width;
        auto v_img = (1 - v) * height;

        int u_img_i = std::clamp((int)u_img, 0, width - 1);
        int v_img_i = std::clamp((int)v_img, 0, height - 1);

        auto color = image_data.at<cv::Vec3b>(v_img_i, u_img_i);
        return Eigen::Vector3f(color[0], color[1], color[2]);
    }

    Eigen::Vector3f getColorBilinear(float u, float v) {
        //get uv in [0,1]
        u = std::clamp(u, 0.0f, 1.0f);
        v = std::clamp(v, 0.0f, 1.0f);

        auto u_img = u * width;
        auto v_img = (1 - v) * height;

        int u_min = std::clamp((int)std::floor(u_img), 0,width - 1);
        int u_max = std::clamp((int)std::ceil(u_img), 0, width - 1);
        int v_min = std::clamp((int)std::floor(v_img), 0, height - 1);
        int v_max = std::clamp((int)std::ceil(v_img), 0, height - 1);
        //计算插值系数
        float s = u_img - (float)u_min;
        float t = v_img - (float)v_min;

        cv::Vec3b c00 = image_data.at<cv::Vec3b>(v_min, u_min); // (u_min, v_min)
        cv::Vec3b c10 = image_data.at<cv::Vec3b>(v_min, u_max); // (u_max, v_min)
        cv::Vec3b c01 = image_data.at<cv::Vec3b>(v_max, u_min); // (u_min, v_max)
        cv::Vec3b c11 = image_data.at<cv::Vec3b>(v_max, u_max); // (u_max, v_max)

        Eigen::Vector3f c00f((float)c00[0], (float)c00[1], (float)c00[2]);
        Eigen::Vector3f c10f((float)c10[0], (float)c10[1], (float)c10[2]);
        Eigen::Vector3f c01f((float)c01[0], (float)c01[1], (float)c01[2]);
        Eigen::Vector3f c11f((float)c11[0], (float)c11[1], (float)c11[2]);

        //水平插值
        Eigen::Vector3f row_col_down = (1 - s) * c00f + s * c10f;
        Eigen::Vector3f row_col_up = (1 - s) * c01f + s * c11f;
        //竖直插值
        Eigen::Vector3f col = (1 - t) * row_col_down + t * row_col_up;

        return col;
    }

};
#endif //RASTERIZER_TEXTURE_H

