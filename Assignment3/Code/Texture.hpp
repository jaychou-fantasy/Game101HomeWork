//
// Created by LEI XU on 4/27/19.
//

#ifndef RASTERIZER_TEXTURE_H
#define RASTERIZER_TEXTURE_H
#include "global.hpp"
#include <algorithm>
#include <Eigen/Eigen>
#include <cmath>
#include <opencv2/core.hpp>
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
        auto u_img = static_cast<int>(std::clamp(u, 0.0f, 1.0f) * (width - 1));
        auto v_img = static_cast<int>((1 - std::clamp(v, 0.0f, 1.0f)) * (height - 1));
        auto color = image_data.at<cv::Vec3b>(v_img, u_img);
        return Eigen::Vector3f(color[0], color[1], color[2]);
    }

    Eigen::Vector3f getColorBilinear(float u,float v)
    {
        //get image coordinates
        auto u_img = std::clamp(u, 0.0f, 1.0f) * (width-1);
        auto v_img = (1 - std::clamp(v, 0.0f, 1.0f)) * (height-1);

        //find the sorrounding texels
        int x0 = static_cast<int>(std::floor(u_img));
        int y0 = static_cast<int>(std::floor(v_img));
        int x1 = std::min(x0+1,width-1);
        int y1 = std::min(y0+1,height-1);

        float tx = u_img - x0;
        float ty = v_img - y0;

        auto sample = [this](int xx,int yy){//this means this lambda function can use all varible in Texture.hpp
            const auto& color = image_data.at<cv::Vec3b>(yy,xx);
            return Eigen::Vector3f(color[0],color[1],color[2]);
        };
        Eigen::Vector3f top = (1.0-tx) * sample(x0, y0) + tx * sample(x1,y0);
        Eigen::Vector3f bottom = (1.0-tx) * sample(x0, y1) + tx * sample(x1,y1);

        return (1.0-ty) * top + ty * bottom;
    }
};
#endif //RASTERIZER_TEXTURE_H
