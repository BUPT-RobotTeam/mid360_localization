
#include "geometry_msgs/Transform.h"
#include "geometry_msgs/TransformStamped.h"
#include "ros/init.h"
#include <cmath>
#include <cstddef>
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/core/cvdef.h>
#include <opencv2/core/hal/interface.h>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/matx.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <vector>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PointStamped.h>


int main(int argc, char **argv)
{
    ros::init(argc, argv, "corner_detect");

    cv::Mat img = cv::imread("/home/yin/trash_ws/ooop.png");
    cv::cvtColor(img, img, cv::COLOR_BGR2GRAY);
    if (img.empty())
    {
        ROS_ERROR("Image not found");
        return -1;
    }
    cv::Mat close_kernel = cv::Mat::ones(2, 2, CV_8U);
    cv::morphologyEx(img, img, cv::MORPH_CLOSE, close_kernel);
    cv::Mat img_canny;
    cv::GaussianBlur(img, img_canny, cv::Size(5, 5), 5);
    cv::Canny(img_canny, img_canny, 100, 200);
    // cv::GaussianBlur(img, img, cv::Size(5, 5),1);
    // cv::Canny(img, img, 100, 200);
    // std::vector<std::vector<cv::Point>> contours;
    // cv::findContours(img, contours, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);
    // cv::Mat img_contours = cv::Mat::zeros(img.size(), CV_8UC1);
    // std::vector<std::vector<cv::Point>> contours_poly;
    // for (auto &contour : contours){
    //     std::vector<cv::Point> approx;
    //     cv::approxPolyDP(contour, approx, 3, true);
    //     if (cv::contourArea(approx) < 5){
    //         continue;
    //     }
    //     contours_poly.push_back(approx);
    //     // for (const auto &point : approx){
    //     //     cv::circle(img, point, 2, 255, 1);
    //     // }
    // }
    // std::vector<cv::Point> corners;
    // std::vector<cv::Vec4b> lines;
    // for (const auto & contour : contours_poly){
    //     for (size_t i = 0; i < contour.size(); i++){
    //         // cv::line(img, contour[i], contour[(i+1)%contour.size()], 255, 1);
    //         int i2 = (i+1)%contour.size();
    //         int i3 = (i+2)%contour.size();
    //         cv::Point2f p1 = contour[i];
    //         cv::Point2f p2 = contour[i2];
    //         cv::Point2f p3 = contour[i3];
    //         const static float RightAngleThres = 20;
    //         float angle = std::abs(atan2(p2.y - p1.y, p2.x - p1.x) - atan2(p3.y - p2.y, p3.x - p2.x)) * 180 / CV_PI;
    //         if (std::abs(90 - angle) < RightAngleThres){
    //             if (std::abs(p1.x - p2.x) < 3){
    //                 cv::circle(img, p2, 4, 255, 2);
    //                 corners.push_back(p2);
    //                 lines.push_back(cv::Vec4b(p2.x, p2.y, p1.x, p1.y));
    //             }else if (std::abs(p2.x - p3.x) < 3) {
    //                 cv::circle(img, p2, 4, 255, 2);
    //                 corners.push_back(p2);
    //                 lines.push_back(cv::Vec4b(p2.x, p2.y, p3.x, p3.y));
    //             }
    //         }
    //     }
    // }

    std::vector<cv::Vec4f> lines;
    cv::HoughLinesP(img_canny, lines, 1, CV_PI / 180, 50, 30, 60);

    std::cout << "lines size: " << lines.size() << std::endl;
    cv::Mat image = cv::Mat::zeros((img.size()), CV_8UC1);
    image.setTo(255);
    // img.copyTo(image);
    for (const auto &line : lines){
        cv::line(image, cv::Point(line[0], line[1]), cv::Point(line[2], line[3]), 0, 2);
    }
    cv::Point pt; //TODO: 获取墙角
    cv::Vec4b line;
    // img.setTo(0);
    // cv::drawContours(img_contours, contours_poly, -1, 255, 1);

    // cv::Mat corners;
    tf::TransformBroadcaster bd;
    geometry_msgs::TransformStamped transformStamped;

    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "base_link";
    transformStamped.child_frame_id = "camera_link";

    // 设置变换的平移
    transformStamped.transform.translation.x = pt.x;
    transformStamped.transform.translation.y = pt.y;
    transformStamped.transform.translation.z = 0.0;
    double yaw_angle = std::fabs(atan2(line[3] - line[1], line[2] - line[0]));
    yaw_angle = line[3] > line[1] ? yaw_angle : -yaw_angle;
    transformStamped.transform.rotation = tf::createQuaternionMsgFromYaw(yaw_angle);  // 90度逆时针旋转

    // cv::imshow("image", image);
    // cv::imshow("img", img);
    cv::Mat show;
    cv::hconcat(img, img_canny, show);
    cv::hconcat(show, image, show);
    cv::imshow("show", show);
    cv::waitKey(0);

    // cv::GaussianBlur(img, img, cv::Size(3, 3), 0);

    // ros::spin();
    return 0;
}