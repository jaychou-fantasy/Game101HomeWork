#include <algorithm>
#include <chrono>
#include <iostream>
#include <cmath>
#include <opencv2/core.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/opencv.hpp>

std::vector<cv::Point2f> control_points;

void mouse_handler(int event, int x, int y, int flags, void *userdata) 
{
    if (event == cv::EVENT_LBUTTONDOWN && control_points.size() < 4) 
    {
        std::cout << "Left button of the mouse is clicked - position (" << x << ", "
        << y << ")" << '\n';
        control_points.emplace_back(x, y);// == controll_points.push_back(Eigen::Vector2f(x,y));
    }     
}

void naive_bezier(const std::vector<cv::Point2f> &points, cv::Mat &window) 
{
    auto &p_0 = points[0];
    auto &p_1 = points[1];
    auto &p_2 = points[2];
    auto &p_3 = points[3];

    for (double t = 0.0; t <= 1.0; t += 0.001) 
    {
        auto point = std::pow(1 - t, 3) * p_0 + 3 * t * std::pow(1 - t, 2) * p_1 +
                 3 * std::pow(t, 2) * (1 - t) * p_2 + std::pow(t, 3) * p_3;

        window.at<cv::Vec3b>(point.y, point.x)[2] = 255;
    }
}

cv::Point2f recursive_bezier(const std::vector<cv::Point2f> &control_points, float t) 
{
    // TODO: Implement de Casteljau's algorithm
    auto &b_0 = control_points[0];
    auto &b_1 = control_points[1];
    cv::Point2f b_2;
    b_2 = (1-t)*b_0 + t*b_1;
    return b_2;

}

void bezier(const std::vector<cv::Point2f> &control_points, cv::Mat &window) 
{
    // TODO: Iterate through all t = 0 to t = 1 with small steps, and call de Casteljau's 
    // recursive Bezier algorithm.
    auto &p_0 = control_points[0];
    auto &p_1 = control_points[1];
    auto &p_2 = control_points[2];
    auto &p_3 = control_points[3];

    cv::Point2f p_01,p_12,p_23;
    cv::Point2f p_012,p_123;
    cv::Point2f p_bezier;
    for(double t = 0.0f; t <= 1.0f; t+= 0.001)
    {
        p_01 = recursive_bezier({p_0,p_1},t);
        p_12 = recursive_bezier({p_1,p_2},t);
        p_23 = recursive_bezier({p_2,p_3},t);

        p_012 = recursive_bezier({p_01,p_12},t);
        p_123 = recursive_bezier({p_12,p_23},t);

        p_bezier = recursive_bezier({p_012,p_123}, t);

        //window.at<cv::Vec3b>(p_bezier.y,p_bezier.x)[1] = 255;//implicitly type comversion to integer,directly cut the fractional part (Truncate)

        int x0 = std::max(static_cast<int>(std::floor(p_bezier.x)),0);
        int x1 = std::min(x0+1,window.cols-1);
        int y0 = std::max(static_cast<int>(std::floor(p_bezier.y)),0);
        int y1 = std::min(y0+1,window.rows-1);

        auto paint = [&](int x, int y) {
            float dx = x - p_bezier.x;
            float dy = y - p_bezier.y;
            float dist = std::sqrt(dx * dx + dy * dy);

            float alpha = std::max(0.0f, (1.44f - dist)/1.44f);
            auto new_green = static_cast<uchar>(alpha * 255.0f);//unsigned char -- [0,255],char -- [-128,127]

            auto& green = window.at<cv::Vec3b>(y, x)[1];
            green = std::max(green, new_green);
        };

        paint(x0, y0);
        paint(x1, y0);
        paint(x0, y1);
        paint(x1, y1);
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
            cv::circle(window, point, 3, {255, 255, 255}, 3);
        }

        if (control_points.size() == 4) 
        {
            naive_bezier(control_points, window);
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
