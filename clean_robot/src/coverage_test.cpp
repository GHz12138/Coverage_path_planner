#include "tf/tf.h"
#include "tf/transform_listener.h"
#include <boost/shared_ptr.hpp>
#include <costmap_2d/costmap_2d_ros.h>
#include "coverage_planner.h"
#include <opencv2/opencv.hpp>
#include <nav_msgs/Path.h>

namespace cm = costmap_2d;
namespace rm = geometry_msgs;

using cm::Costmap2D;
using cm::Costmap2DROS;
using rm::PoseStamped;
using std::string;
using std::vector;

using namespace coverageplanner;
using namespace std;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "coverage_planner_node");
    ros::NodeHandle nh;
    // ros::Publisher plan_pub = nh.advertise<nav_msgs::Path>("coverage_path", 1, true);

    ros::NodeHandle private_nh("~/cleaning_plan_nodehandle");
    ros::Publisher plan_pub = private_nh.advertise<nav_msgs::Path>("cleaning_path", 1, true);

    tf2_ros::Buffer tf;
    tf2_ros::TransformListener tf2_listener(tf);
    costmap_2d::Costmap2DROS lcr("cleaning_costmap", tf);

    // 等待 costmap 初始化
    ros::Duration(2.0).sleep();

    // 获取底层 Costmap2D 指针
    costmap_2d::Costmap2D *costmap2d = lcr.getCostmap();

    int sizex = costmap2d->getSizeInCellsX(); // 列数（宽度）
    int sizey = costmap2d->getSizeInCellsY(); // 行数（高度）
    double resolution = costmap2d->getResolution();

    std::cout << "Costmap size: " << sizex << " x " << sizey
              << " , resolution = " << resolution << std::endl;

    // 转换为 OpenCV 图像（单通道 8bit）
    // cv::Mat1b map(sizey, sizex); // rows=sizey, cols=sizex
    // for (int r = 0; r < sizey; r++)
    // {
    //     for (int c = 0; c < sizex; c++)
    //     {
    //         // costmap 原点在左下角，OpenCV 图像原点在左上角
    //         unsigned char cost = costmap2d->getCost(c, sizey - r - 1);
    //         map(r, c) = cost;
    //     }
    // }
    cv::Mat1b map(sizey, sizex); // rows=sizey, cols=sizex
    for (int r = 0; r < sizey; r++)
    {
        for (int c = 0; c < sizex; c++)
        {
            unsigned char cost = costmap2d->getCost(c, sizey - r - 1);

            // 映射规则：
            if (cost == costmap_2d::FREE_SPACE)
            {
                map(r, c) = 255; // 空闲区域显示为白色
            }
            else if (cost == costmap_2d::LETHAL_OBSTACLE)
            {
                map(r, c) = 0; // 障碍物显示为黑色
            }
            else if (cost == costmap_2d::NO_INFORMATION)
            {
                map(r, c) = 127; // 未知区域显示为灰色
            }
            else
            {
                map(r, c) = 127; // 膨胀层/其他障碍：浅灰
            }
        }
    }
    // 可视化
    // cv::namedWindow("Costmap", cv::WINDOW_NORMAL);
    // cv::imshow("Costmap", map);
    // cv::waitKey(3000);
    // cv::destroyWindow("Costmap");

    double robot_radius = 0.2 / resolution; // 机器人半径（以像素为单位）
    std::cout << "Robot radius in pixels: " << robot_radius << std::endl;
    // 一般情况下 弓字间距 小于或者等于 2*robot_radius
    double clean_distance = 0.2 / resolution;
    CoveragePlanner cover_test;
    geometry_msgs::PoseStamped initPose_;
    bool isok = lcr.getRobotPose(initPose_);
    if (!isok)
    {
        ROS_INFO("Failed to get robot location! Please check where goes wrong!");
        return 0;
    }
    unsigned int mx, my;
    double wx = initPose_.pose.position.x; // 获取原点的x坐标
    double wy = initPose_.pose.position.y;
    lcr.getCostmap()->worldToMap(wx, wy, mx, my);
    int init_y = costmap2d->getSizeInCellsY() - my - 1;
    int init_x = mx;
    Point2D start = {init_x, init_y};
    std::cout << "Start point in map coordinates: (" << start.x << ", " << start.y << ")" << std::endl;

    std::deque<Point2D> coverage_path = cover_test.planner(map, robot_radius, clean_distance, start);

    // 打印 coverage_path 前十个元素
    // for (size_t i = 0; i < std::min<size_t>(10, coverage_path.size()); ++i)
    // {
    //     const auto &pt = coverage_path[i];
    //     std::cout << "coverage_path[" << i << "]: x=" << pt.x << ", y=" << pt.y << std::endl;
    // }
    geometry_msgs::PoseStamped posestamped;
    geometry_msgs::Pose pose;
    bool has_prev = false;
    double prev_wx = 0.0, prev_wy = 0.0;

    std::vector<geometry_msgs::PoseStamped> path;

    for (const auto &point : coverage_path)
    {
        int transformed_y = sizey - point.y - 1;

        double wx, wy;
        costmap2d->mapToWorld(point.x, transformed_y, wx, wy);

        if (has_prev)
        {
            double dx = wx - prev_wx;
            double dy = wy - prev_wy;
            double yaw = std::atan2(dy, dx);
            pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
        }
        else
        {
            pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
            has_prev = true;
        }

        pose.position.x = wx;
        pose.position.y = wy;
        pose.position.z = 0.0;

        prev_wx = wx;
        prev_wy = wy;

        geometry_msgs::PoseStamped posestamped;
        posestamped.header.stamp = ros::Time::now();
        posestamped.header.frame_id = "map";
        posestamped.pose = pose;

        path.push_back(posestamped);
    }

    // create a message for the plan
    nav_msgs::Path gui_path;
    gui_path.poses.resize(path.size());

    gui_path.header.frame_id = "map";
    gui_path.header.stamp = ros::Time::now();

    for (unsigned int i = 0; i < path.size(); i++)
    {
        gui_path.poses[i] = path[i];
    }

    plan_pub.publish(gui_path);

    ros::spin();
    return 0;
}

// int main(int argc, char **argv)
// {
//     ros::init(argc, argv, "coverage_planner_node");

//     tf2_ros::Buffer tf;
//     tf2_ros::TransformListener tf2_listener(tf);
//     costmap_2d::Costmap2DROS lcr("cleaning_costmap", tf);

//     ros::Duration(5).sleep();

//     int robot_radius = 0.5;
//     // 一般情况下 弓字间距 小于或者等于 2*robot_radius
//     int clean_distance = 1.0;

//     // costmap2d_ros_->updateMap();
//     costmap_2d::Costmap2D *costmap2d_ = lcr.getCostmap();

//     int sizex = costmap2d_->getSizeInCellsX(); // 获取地图尺寸
//     int sizey = costmap2d_->getSizeInCellsY();
//     cout << "The size of map is " << sizex << "  " << sizey << endl;
//     double resolution_ = costmap2d_->getResolution(); // 分辨率

//     cv::Mat map = cv::Mat(sizey, sizex, CV_8U);
//     for (int r = 0; r < sizey; r++)
//     {
//         for (int c = 0; c < sizex; c++)
//         {
//             map.at<uchar>(r, c) = costmap2d_->getCost(c, sizey - r - 1); //??sizey-r-1 caution: costmap's origin is at left bottom ,while opencv's pic's origin is at left-top.
//             // getCost（）:获取代价值
//         }
//     }
//     CoveragePlanner cover_test;
//     cover_test.planner(map, robot_radius, clean_distance);
//     // cv::namedWindow("map", cv::WINDOW_NORMAL); // 可缩放窗口
//     // cv::imshow("map", map);
//     // cv::waitKey(3000); // 等待 3000 毫秒（3 秒），然后继续
//     // cv::destroyWindow("map"); // 或者 cv::destroyAllWindows();
//     // ros::Rate r(1);
//     // while (ros::ok())
//     // {
//     //     //   clr.PublishCoveragePath();

//     //     ros::spinOnce();
//     //     r.sleep();
//     // }

//     ros::shutdown(); // 关闭节点以及所有与之相关的发布，订阅，调用与服务。
//     return 0;
// }
