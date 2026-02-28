/* 
Developer: Chunran Zheng <zhengcr@connect.hku.hk>

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#ifndef DATA_PREPROCESS_HPP
#define DATA_PREPROCESS_HPP

#include "CustomMsg.h"
#include <Eigen/Core>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <fstream>
#include "common_lib.h"
#include <filesystem>
#include <sstream>
#include <iomanip>

namespace fs = std::filesystem;

using namespace std;

enum class LiDARType : int {
    Unknown = 0,
    Solid   = 1,   // 固态（如 Livox）
    Mech    = 2    // 机械式多线
};

class DataPreprocess
{
public:
    // 改成带线号的点云
    pcl::PointCloud<Common::Point>::Ptr cloud_input_;
    cv::Mat img_input_;
    LiDARType lidar_type_{LiDARType::Unknown};
    LiDARType lidarType() const { return lidar_type_; }

    DataPreprocess(Params &params)
        : cloud_input_(new pcl::PointCloud<Common::Point>)
    {
        // string bag_path   = params.bag_path; // not being used anymore
        string image_path = params.image_path;
        // string lidar_topic = params.lidar_topic; // not being used anymore

        // read image
        img_input_ = cv::imread(image_path, cv::IMREAD_UNCHANGED);
        if (img_input_.empty())
        {
            std::string msg = "Loading the image " + image_path + " failed";
            ROS_ERROR_STREAM(msg.c_str());
            return;
        }

        // read lidar data
        int pc_frame_start = params.pc_frame_start;
        int pc_frame_end = params.pc_frame_end;
        string pc_dir_str = params.pc_dir;
        fs::path pc_dir(pc_dir_str);
        bool has_ring = params.has_ring;
        bool is_livox = params.is_livox;

        if(is_livox || !has_ring) {
            lidar_type_ = LiDARType::Solid;
        }
        else if(has_ring) {
            lidar_type_ = LiDARType::Mech;
        }
        else {
            lidar_type_ = LiDARType::Unknown;
        }

        if(is_livox || has_ring) {
            // point cloud has ring information (for livox, the line field is used)
            
            pcl::PointCloud<Common::Point> temp;

            // Compute size of aggregated point cloud
            size_t total_points = 0;
            for(size_t i = pc_frame_start; i <= pc_frame_end; i++) {
                // load PCD file
                std::stringstream ss;
                ss << std::setw(8) << std::setfill('0') << i << ".pcd";
                fs::path file_path = pc_dir / ss.str();
                if(pcl::io::loadPCDFile<Common::Point>(file_path.string(), temp) == -1) {
                    std::string msg = "Loading pcd " + file_path.string() + " failed";
                    ROS_ERROR_STREAM(msg.c_str());
                    return;
                }

                total_points += temp.size();
            }

            // Reserve space in cloud_input_
            cloud_input_->reserve(total_points);
            for(size_t i = pc_frame_start; i <= pc_frame_end; i++) {
                // load PCD file
                std::stringstream ss;
                ss << std::setw(8) << std::setfill('0') << i << ".pcd";
                fs::path file_path = pc_dir / ss.str();
                if(pcl::io::loadPCDFile<Common::Point>(file_path.string(), temp) == -1) {
                    std::string msg = "Loading pcd " + file_path.string() + " failed";
                    ROS_ERROR_STREAM(msg.c_str());
                    return;
                }

                // add points to cloud_input_
                for(auto& point : temp) {
                    cloud_input_->push_back(point);
                }
            }
        }
        else {
            // point cloud does not have ring information (just using x, y, z)

            pcl::PointCloud<pcl::PointXYZ> temp;

            // Compute size of aggregated point cloud
            size_t total_points = 0;
            for(size_t i = pc_frame_start; i <= pc_frame_end; i++) {
                // load PCD file
                std::stringstream ss;
                ss << std::setw(8) << std::setfill('0') << i << ".pcd";
                fs::path file_path = pc_dir / ss.str();
                if(pcl::io::loadPCDFile<pcl::PointXYZ>(file_path.string(), temp) == -1) {
                    std::string msg = "Loading pcd " + file_path.string() + " failed";
                    ROS_ERROR_STREAM(msg.c_str());
                    return;
                }

                total_points += temp.size();
            }

            // Reserve space in cloud_input_
            cloud_input_->reserve(total_points);
            for(size_t i = pc_frame_start; i <= pc_frame_end; i++) {
                // load PCD file
                std::stringstream ss;
                ss << std::setw(8) << std::setfill('0') << i << ".pcd";
                fs::path file_path = pc_dir / ss.str();
                if(pcl::io::loadPCDFile<pcl::PointXYZ>(file_path.string(), temp) == -1) {
                    std::string msg = "Loading pcd " + file_path.string() + " failed";
                    ROS_ERROR_STREAM(msg.c_str());
                    return;
                }

                // add points to cloud_input_
                for(auto& point : temp) {
                    Common::Point p;
                    p.x = point.x;
                    p.y = point.y;
                    p.z = point.z;
                    // undefined ring
                    p.ring = 0xFFFF;
                    
                    cloud_input_->push_back(p);
                }
            }
        }
        
        ROS_INFO("Loaded %zu points", cloud_input_->size());
    }
};

typedef std::shared_ptr<DataPreprocess> DataPreprocessPtr;

#endif // DATA_PREPROCESS_HPP