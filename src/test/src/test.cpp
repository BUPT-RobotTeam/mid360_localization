#include "octomap/OcTree.h"
#include "ros/console.h"
#include "ros/init.h"
#include "ros/node_handle.h"
#include "ros/publisher.h"
#include "ros/subscriber.h"
#include <opencv2/core/types.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "sensor_msgs/PointCloud2.h"
#include <mutex>
#include <opencv2/core/mat.hpp>
#include "pcl/point_cloud.h"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <octomap/ColorOcTree.h>
#include <octomap/octomap.h>
#include <octomap/Pointcloud.h>
#include <octomap/ColorOcTree.h>
#include <nav_msgs/OccupancyGrid.h>
#include <opencv2/opencv.hpp>
#include <vector>


class OctomapProc
{
public:
    OctomapProc(const std::string &topic, const float resolution, const double Map2Mat, const std::string &publish_topic = "/debug/octomap") 
        : octree_(resolution), Map2Mat_(Map2Mat)
    {
        sub_ = nh_.subscribe<sensor_msgs::PointCloud2>(topic, 1, &OctomapProc::callback, this);
        pub_ = nh_.advertise<nav_msgs::OccupancyGrid>(publish_topic, 1);
    }

    cv::Mat Octomap2Mat()
    {
        mutex_.lock();

        // 获取octomap的空间边界
        double x_min, x_max, y_min, y_max;
        double z_min, z_max; //后续用不到
        octree_.getMetricMin(x_min, y_min, z_min); 
        octree_.getMetricMax(x_max, y_max, z_max);

        // 设置二维图像的大小，根据分辨率确定
        int width = static_cast<int>((x_max - x_min) / Map2Mat_);
        int height = static_cast<int>((y_max - y_min) / Map2Mat_);

        cv::Mat map2D = cv::Mat(width, height, CV_8UC1);
        map2D.setTo(255);

        for (octomap::OcTree::leaf_iterator it = octree_.begin_leafs(), end = octree_.end_leafs(); it != end; ++it) {
            // 获取体素的3D坐标
            octomap::point3d pt = it.getCoordinate();
    
            // 将 3D 坐标投影到 2D（XY平面）
            int x_pixel = static_cast<int>((pt.x() - x_min) / Map2Mat_);
            int y_pixel = static_cast<int>((pt.y() - y_min) / Map2Mat_);
    
            // 如果体素位于栅格图像的有效范围内
            if (x_pixel >= 0 && x_pixel < width && y_pixel >= 0 && y_pixel < height) {
                // 检查体素是否被占用
                if (it->getOccupancy() > 0.5) {
                    map2D.at<uchar>(y_pixel, x_pixel) = 0;
                    ROS_INFO("%d %d %f", x_pixel, y_pixel, it->getOccupancy());
                }
            }
        }

        mutex_.unlock();
        return map2D;
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
        sub_ = nh_.subscribe<sensor_msgs::PointCloud2>("/camera/depth/points", 1, &OctomapProc::callback, this);
        is_callback_enabled_ = true;
        ROS_INFO("Callback enabled.");
    }

    void PublishOctomapAsOccupancyGrid() 
    {
        mutex_.lock();

        // 获取Octomap的边界
        double x_min, x_max, y_min, y_max;
        double z_min, z_max;
        octree_.getMetricMin(x_min, y_min, z_min);
        octree_.getMetricMax(x_max, y_max, z_max);
        ROS_INFO("Get Metric Size %f %f %f %f", x_min, x_max, y_min, y_max);

        // 设置二维栅格地图的尺寸
        int width = static_cast<int>((x_max - x_min) / Map2Mat_);
        int height = static_cast<int>((y_max - y_min) / Map2Mat_);

        // 创建OccupancyGrid消息
        nav_msgs::OccupancyGrid occupancy_grid;
        occupancy_grid.header.stamp = ros::Time::now();
        occupancy_grid.header.frame_id = "map";
        occupancy_grid.info.resolution = Map2Mat_;
        occupancy_grid.info.width = width;
        occupancy_grid.info.height = height;
        occupancy_grid.data.resize(width * height, -1); // 初始化为未知状态

        // 将Octomap中的体素转换为OccupancyGrid数据
        for (octomap::OcTree::leaf_iterator it = octree_.begin_leafs(),
                                            end = octree_.end_leafs();
            it != end; ++it) {
            octomap::point3d pt = it.getCoordinate();

            // 将3D坐标转换为2D栅格坐标
            int x_pixel = static_cast<int>((pt.x() - x_min) / Map2Mat_);
            int y_pixel = static_cast<int>((pt.y() - y_min) / Map2Mat_);

            // 判断栅格是否在有效范围内
            if (x_pixel >= 0 && x_pixel < width && y_pixel >= 0 &&
                y_pixel < height) {
                    int idx = y_pixel * width + x_pixel;
                    // ROS_INFO("...%d %d %f", x_pixel, y_pixel, it->getOccupancy());
                    // 如果体素被占用，设置为100，占用状态
                    if (it->getOccupancy() > 0.5) {
                        occupancy_grid.data[idx] = 100; // 表示占用
                    } else {
                        occupancy_grid.data[idx] = 0; // 空闲
                    }
            }
        }

        // 发布OccupancyGrid
        pub_.publish(occupancy_grid);

        mutex_.unlock();
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Publisher pub_;
    octomap::OcTree octree_;
    std::mutex mutex_;
    bool is_callback_enabled_ = true; //是否启用回调函数
    double Map2Mat_; //地图至图像的缩小比例

    void callback(const sensor_msgs::PointCloud2::ConstPtr &msg)
    {
        const static float z_min = 0.05;
        const static float z_max = 1.0;
        
        mutex_.lock();
        ROS_INFO("===================CALLBACK=======================");
        pcl::PointCloud<pcl::PointXYZ>::Ptr pts_ptr(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *pts_ptr);
        // RadiusOutlierFilter(pts_ptr, 0.1, 5);

        for (const auto &pt : pts_ptr->points) {
            // ROS_INFO("PT Write in. %f %f %f", pt.x, pt.y, pt.z);
            if (pt.z < z_min || pt.z > z_max){
                continue;
            }
            octree_.updateNode(octomap::point3d(pt.x, pt.y, 0), true); //转为栅格地图
        }
        
        
        mutex_.unlock();
        
        cv::Mat img = Octomap2Mat();
        
        // cv::imshow("Octomap", img);
        cv::imwrite("oc.png", img);
        // cv::waitKey(10);
        // PublishOctomapAsOccupancyGrid();

    }

    // 半径滤波，消除离群点
    void RadiusOutlierFilter(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud,
        const double radius, const int thre_count) {

        // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_after_Radius(new pcl::PointCloud<pcl::PointXYZ>);
        //创建滤波器
        pcl::RadiusOutlierRemoval<pcl::PointXYZ> radiusoutlier;
        //设置输入点云
        radiusoutlier.setInputCloud(input_cloud);
        //设置半径,在该范围内找临近点
        radiusoutlier.setRadiusSearch(radius);
        //设置查询点的邻域点集数，小于该阈值的删除
        radiusoutlier.setMinNeighborsInRadius(thre_count);
        radiusoutlier.filter(*input_cloud);
        
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

    OctomapProc pcl2octomap("/cloud_registered", 0.05, 0.01);

    ros::spin();

    return 0;
}
