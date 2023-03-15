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
    Eigen::Vector3f getColorBilinear(float u, float v){
        auto u_img = u * width;
        auto v_img = (1 - v) * height;
        int u0 = u_img,v0 = v_img;
        int u1 = u0 + 1, v1 = v0 + 1;
        auto lerp = [](Eigen::Vector3f &color1,Eigen::Vector3f &color2, float weight){
            Eigen::Vector3f ret = color1 * (1 - weight) + color2 * weight;
            return ret;
        };
        auto get_color = [this](int u,int v){
            auto color = image_data.at<cv::Vec3b>(v, u);
            return Eigen::Vector3f(color[0], color[1], color[2]);
        };
        auto color1 = get_color(u0,v0);
        auto color2 = get_color(u1,v1);
        auto color_a = lerp(color1,color2,u_img - u0);
        color1 = get_color(u0,v1);
        color2 = get_color(u1,v1);
        auto color_b = lerp(color1,color2,u_img - u0);
        return lerp(color_a,color_b,v_img - v0);


    }
    Eigen::Vector3f getColor(float u, float v)
    {
        auto u_img = u * width;
        auto v_img = (1 - v) * height;
        auto color = image_data.at<cv::Vec3b>(v_img, u_img);
        return Eigen::Vector3f(color[0], color[1], color[2]);
    }

};
#endif //RASTERIZER_TEXTURE_H
