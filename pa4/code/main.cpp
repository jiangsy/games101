#include <chrono>
#include <iostream>
#include <opencv2/opencv.hpp>


std::vector<cv::Point2f> control_points;

void mouse_handler(int event, int x, int y, int flags, void *userdata)
{
    if (event == cv::EVENT_LBUTTONDOWN && control_points.size() < 4) 
    {
        std::cout << "Left button of the mouse is clicked - position (" << x << ", "
        << y << ")" << '\n';
        control_points.emplace_back(x, y);
    }     
}

void draw_dot(const cv::Point2f &point, cv::Mat window, const cv::Vec3b &color,
              bool anti_aliasing=false) {
    if (anti_aliasing) {
        auto radius = 0.5f;
        auto cr = 1.1f;

        int x1 = std::floor(point.x - radius);
        int x2 = std::ceil(point.x + radius);
        int y1 = std::floor(point.y - radius);
        int y2 = std::ceil(point.y + radius);
        for (int x=y1; x<y2; x++) {
            for (int y=x1; y<x2; y++) {
                auto dist = std::sqrt(std::pow(point.y-x -0.5f, 2.0f) +
                                      std::pow(point.x-y-0.5f, 2.0f));
                dist = dist/radius/2.0f;
                if (dist < 2*radius)
                    dist = std::pow(dist, cr);
                for (int i=0; i<3; i++)
                    window.at<cv::Vec3b>(x, y)[i]=std::max(color[i] * (1.0f-dist),
                                                           float(window.at<cv::Vec3b>(x, y)[i]));
            }
        }
    } else {
        window.at<cv::Vec3b>(point.y, point.x)=color;
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
        draw_dot(point, window, cv::Vec3b(0, 0, 255),false);
    }
}

cv::Point2f recursive_bezier(const std::vector<cv::Point2f> &control_points, float t)
{
    std::vector<cv::Point2f> tmp_control_points;
    tmp_control_points.assign(control_points.begin(), control_points.end());
    for (auto j=tmp_control_points.size(); j > 0; j--) {
        for (auto i = 0; i < j-1; i++){
            tmp_control_points[i] = tmp_control_points[i]*t + tmp_control_points[i+1]*(1-t);
        }
    }
    return tmp_control_points[0];

}

void bezier(const std::vector<cv::Point2f> &control_points, cv::Mat &window)
{

    for (double t = 0.0; t <= 1.0; t += 0.001) {
        auto point = recursive_bezier(control_points, t);
        draw_dot(point, window, cv::Vec3b(0, 255, 0), true);
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
            cv::waitKey(0);
            control_points.clear();
            window = cv::Mat(700, 700, CV_8UC3, cv::Scalar(0));
        }

        cv::imshow("Bezier Curve", window);
        key = cv::waitKey(20);
    }

return 0;
}
