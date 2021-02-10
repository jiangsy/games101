//
// Created by LEI XU on 4/27/19.
//

#ifndef RASTERIZER_TEXTURE_H
#define RASTERIZER_TEXTURE_H
#include "global.hpp"
#include <eigen3/Eigen/Eigen>
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
        auto u_img = u * width;
        auto v_img = (1 - v) * height;
        auto color = image_data.at<cv::Vec3b>(v_img, u_img);
        return Eigen::Vector3f(color[0], color[1], color[2]);
    }

    Eigen::Vector3f getColorBilinear(float u, float v)
    {
        auto u_img = u * width;
        auto v_img = (1 - v) * height;
        auto u_img_offset = (u_img - std::floor(u_img)) > 0.5? +1: -1;
        auto v_img_offset = (v_img - std::floor(v_img)) > 0.5? +1: -1;
        auto color_00 = image_data.at<cv::Vec3b>(v_img, u_img);
        auto color_10 = image_data.at<cv::Vec3b>(v_img, std::clamp(int(u_img)+u_img_offset, 0, width));
        auto color_01 = image_data.at<cv::Vec3b>(std::clamp(int(v_img)+v_img_offset, 0, height), u_img);
        auto color_11 = image_data.at<cv::Vec3b>(std::clamp(int(v_img)+v_img_offset, 0, height),
                                                 std::clamp(int(u_img)+u_img_offset, 0, width));

        float x = std::abs(u_img-std::floor(u_img)-0.5f);
        float y = std::abs(v_img-std::floor(v_img)-0.5f);
        auto color_0 = (1-x)*color_00 + x*color_01;
        auto color_1 = (1-x)*color_10 + x*color_11;
        auto color = (1-y)*color_0 + y*color_1;
        return Eigen::Vector3f(color[0], color[1], color[2]);
    }

};
#endif //RASTERIZER_TEXTURE_H
