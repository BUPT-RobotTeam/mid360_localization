#include "octomap/AbstractOcTree.h"
#include "octomap/OcTree.h"
#include "ros/console.h"
#include "ros/init.h"
#include "ros/node_handle.h"
#include "ros/subscriber.h"
#include <opencv2/core/types.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <mutex>
#include <opencv2/core/mat.hpp>
#include <octomap/ColorOcTree.h>
#include <octomap/octomap.h>
#include <opencv2/opencv.hpp>
#include <vector>
#include <octomap_msgs/Octomap.h>
#include "octomap_msgs/conversions.h"

class OctomapProc
{
public:
    OctomapProc(const float resolution = 0.05, const std::string &sub_topic = "/octomap_full", const std::string &publish_topic = "/debug/octomap") 
        : sub_topic_(sub_topic)
    {
        sub_ = nh_.subscribe<octomap_msgs::Octomap>(sub_topic_, 1, &OctomapProc::callback, this);
        // pub_ = nh_.advertise<nav_msgs::OccupancyGrid>(publish_topic, 1);
    }

    void ShutdownSuber()
    {
        sub_.shutdown();
        is_callback_enabled_ = false;
        ROS_INFO("Callback disabled.");
    }

    void EnableSuber()
    {
        if (is_callback_enabled_) {
            ROS_WARN("Callback already enabled.");
            return;
        }
        sub_ = nh_.subscribe<octomap_msgs::Octomap>(sub_topic_, 1, &OctomapProc::callback, this);
        is_callback_enabled_ = true;
        ROS_INFO("Callback enabled.");
    }

    
private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    std::string sub_topic_;
    std::mutex mutex_;
    bool is_callback_enabled_ = true; //是否启用回调函数
    cv::Mat map2D_;
    float resol_ = 0.05;

    void callback(const octomap_msgs::Octomap::ConstPtr &msg)
    {
        mutex_.lock();
        
        ROS_INFO("CALLBACK");
        
        // octomap::OcTree* ocptr = dynamic_cast<octomap::OcTree*>(octomap_msgs::binaryMsgToMap(*msg));
        octomap::AbstractOcTree* ocptr = octomap_msgs::msgToMap(*msg);
        if (!ocptr) { //检查是否转换成功
            ROS_ERROR("Failed to convert octomap message to OcTree.");
            mutex_.unlock();
            return;
        }
        
        // 获取地图的边界
        double x_min, x_max, y_min, y_max;
        double z_min, z_max; //后续用不到
        // 设置二维图像的大小，根据分辨率确定
        int width = static_cast<int>((x_max - x_min) / resol_);
        int height = static_cast<int>((y_max - y_min) / resol_);
        cv::Mat map2D = cv::Mat(width, height, CV_8UC1, 255);
        
        for (octomap::OcTree::leaf_iterator it = ocptr., end = ocptr->end_leafs(); it != end; ++it) {
            // 获取体素的3D坐标
            octomap::point3d pt = it.getCoordinate();
            // 将 3D 坐标投影到 2D（XY平面）
            int x_pixel = static_cast<int>((pt.x() - x_min) / resol_);
            int y_pixel = static_cast<int>((pt.y() - y_min) / resol_);
    
            // 如果体素位于栅格图像的有效范围内
            if (x_pixel >= 0 && x_pixel < width && y_pixel >= 0 && y_pixel < height) {
                // 检查体素是否被占用
                if (it->getOccupancy() > 0.5) {
                    map2D.at<uchar>(y_pixel, x_pixel) = 0;
                } else {
                    map2D.at<uchar>(y_pixel, x_pixel) = 255;
                }
            }
        }
        cv::imshow("map2D", map2D);
        cv::waitKey(1);

        map2D.copyTo(map2D_);
        mutex_.unlock();
        
    }

    cv::Point GetCorner(cv::Mat &img)
    {
        // TODO: 识别图像中的角点
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(img, contours, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);

        for (const auto &contour : contours){
            std::vector<cv::Point> approx;
            cv::approxPolyDP(contour, approx, 3, true);

        }
        
        return cv::Point(0, 0);
    }

};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test");

    OctomapProc op;

    ros::spin();

    return 0;
}
