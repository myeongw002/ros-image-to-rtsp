#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>
#include <thread>
#include <mutex>
#include <atomic>
#include <string>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <Eigen/Dense>
#include <fstream>
#include <sstream>
#include "rtsp_image_sender/projection.hpp"

int WIDTH = 640;
int HEIGHT = 480;
int FPS = 30;
std::string RTSP_URL = "rtsp://localhost:8554/stream";
std::string TOPIC = "/camera1/image_color/compressed";
std::string TOPIC_TYPE = "compressed"; // "image" or "compressed"

// Projection parameters
std::string INTRINSIC_PATH = "";
std::string EXTRINSIC_PATH = "";
std::string PCD_PATH = "";
bool ENABLE_PROJECTION = false;

// Projection data
cv::Mat intrinsic_matrix;
cv::Mat distortion_coeffs;
Eigen::Matrix4d transform_matrix;
pcl::PointCloud<pcl::PointXYZ>::Ptr pcd_data;

std::mutex img_mutex;
cv::Mat latest_img;
std::atomic<bool> running(true);

void loadConfig(const std::string& path) {
    YAML::Node config = YAML::LoadFile(path);
    WIDTH = config["width"].as<int>();
    HEIGHT = config["height"].as<int>();
    FPS = config["fps"].as<int>();
    RTSP_URL = config["rtsp_url"].as<std::string>();
    if (config["topic"])
        TOPIC = config["topic"].as<std::string>();
    if (config["topic_type"])
        TOPIC_TYPE = config["topic_type"].as<std::string>();
    ROS_INFO("Config loaded: %dx%d @ %d fps, topic: %s, type: %s -> %s",
        WIDTH, HEIGHT, FPS, TOPIC.c_str(), TOPIC_TYPE.c_str(), RTSP_URL.c_str());
}

// Load camera intrinsic parameters from CSV file
bool loadIntrinsics(const std::string& path) {
    std::ifstream file(path);
    if (!file.is_open()) {
        ROS_ERROR("Cannot open intrinsics file: %s", path.c_str());
        return false;
    }
    
    std::string line;
    if (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string cell;
        std::vector<double> values;
        
        while (std::getline(ss, cell, ',')) {
            values.push_back(std::stod(cell));
        }
        
        if (values.size() >= 9) {
            intrinsic_matrix = cv::Mat::eye(3, 3, CV_64F);
            intrinsic_matrix.at<double>(0, 0) = values[0]/2; // fx
            intrinsic_matrix.at<double>(1, 1) = values[3]/2; // fy
            intrinsic_matrix.at<double>(0, 2) = values[2]/2; // cx
            intrinsic_matrix.at<double>(1, 2) = values[4]/2; // cy
            
            distortion_coeffs = cv::Mat::zeros(5, 1, CV_64F);
            if (values.size() >= 14) {
                distortion_coeffs.at<double>(0, 0) = values[5];  // k1
                distortion_coeffs.at<double>(1, 0) = values[6];  // k2
                distortion_coeffs.at<double>(2, 0) = values[7];  // p1
                distortion_coeffs.at<double>(3, 0) = values[8];  // p2
                if (values.size() >= 15) {
                    distortion_coeffs.at<double>(4, 0) = values[9];  // k3
                }
            }
            
            ROS_INFO("Intrinsics loaded: fx=%.2f, fy=%.2f, cx=%.2f, cy=%.2f", 
                     values[0], values[3], values[2], values[4]);
            return true;
        }
    }
    
    ROS_ERROR("Invalid intrinsics file format");
    return false;
}

// Load extrinsic transform matrix from text file
bool loadExtrinsics(const std::string& path) {
    std::ifstream file(path);
    if (!file.is_open()) {
        ROS_ERROR("Cannot open extrinsics file: %s", path.c_str());
        return false;
    }
    
    transform_matrix = Eigen::Matrix4d::Identity();
    
    std::string line;
    int row = 0;
    while (std::getline(file, line) && row < 4) {
        std::stringstream ss(line);
        std::string cell;
        int col = 0;
        
        while (std::getline(ss, cell, ',') && col < 4) {
            transform_matrix(row, col) = std::stod(cell);
            col++;
        }
        row++;
    }
    
    ROS_INFO("Extrinsics loaded successfully");
    return true;
}

// Load PCD file
bool loadPCD(const std::string& path) {
    pcd_data.reset(new pcl::PointCloud<pcl::PointXYZ>);
    
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(path, *pcd_data) == -1) {
        ROS_ERROR("Cannot load PCD file: %s", path.c_str());
        return false;
    }
    
    ROS_INFO("PCD loaded: %zu points", pcd_data->size());
    return true;
}

// Callback for sensor_msgs/Image
void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    try {
        cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
        cv::Mat img = cv_ptr->image;
        if (img.empty()) {
            ROS_WARN("Image topic: decode failed");
            return;
        }
        if (img.cols != WIDTH || img.rows != HEIGHT)
            cv::resize(img, img, cv::Size(WIDTH, HEIGHT));
        
        // Apply projection if enabled
        if (ENABLE_PROJECTION && pcd_data && !pcd_data->empty()) {
            img = Projection::pcd_projection_pcl(img, pcd_data, intrinsic_matrix, 
                                                distortion_coeffs, transform_matrix);
        }
        
        std::lock_guard<std::mutex> lock(img_mutex);
        latest_img = img.clone();
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}

// Callback for sensor_msgs/CompressedImage
void compressedImageCallback(const sensor_msgs::CompressedImageConstPtr& msg) {
    try {
        cv::Mat img = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
        if (img.empty()) {
            ROS_WARN("CompressedImage topic: decode failed");
            return;
        }
        if (img.cols != WIDTH || img.rows != HEIGHT)
            cv::resize(img, img, cv::Size(WIDTH, HEIGHT));
        
        // Apply projection if enabled
        if (ENABLE_PROJECTION && pcd_data && !pcd_data->empty()) {
            img = Projection::pcd_projection_pcl(img, pcd_data, intrinsic_matrix, 
                                                distortion_coeffs, transform_matrix);
        }
        
        std::lock_guard<std::mutex> lock(img_mutex);
        latest_img = img.clone();
    } catch (cv::Exception& e) {
        ROS_ERROR("cv::imdecode exception: %s", e.what());
    }
}

void ffmpegWriter() {
    char ffmpeg_cmd[1024];
    snprintf(ffmpeg_cmd, sizeof(ffmpeg_cmd),
        "ffmpeg -y -f rawvideo -pix_fmt bgr24 -s %dx%d -r %d -i - "
        "-c:v libx264 -pix_fmt yuv420p -preset veryfast -tune zerolatency "
        "-g %d -keyint_min %d -sc_threshold 0 -bf 0 -profile:v baseline "
        "-f rtsp \"%s\"",
        WIDTH, HEIGHT, FPS, FPS, FPS, RTSP_URL.c_str()
    );

    FILE* pipe = popen(ffmpeg_cmd, "w");
    if (!pipe) {
        ROS_ERROR("Failed to start FFmpeg process");
        return;
    }
    std::vector<uchar> zero_frame(WIDTH * HEIGHT * 3, 0);

    ros::Rate rate(FPS);
    while (ros::ok() && running.load()) {
        cv::Mat img;
        {
            std::lock_guard<std::mutex> lock(img_mutex);
            if (!latest_img.empty()) {
                img = latest_img.clone();
            }
        }
        if (img.empty()) {
            fwrite(zero_frame.data(), 1, zero_frame.size(), pipe);
        } else {
            fwrite(img.data, 1, WIDTH * HEIGHT * 3, pipe);
        }
        fflush(pipe);
        rate.sleep();
    }
    pclose(pipe);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "proj_image_to_rtsp_cpp");
    ros::NodeHandle nh("~");

    std::string config_path;
    nh.param<std::string>("config", config_path, "config.yaml");
    loadConfig(config_path);

    // Load projection parameters if provided
    nh.param<std::string>("intrinsic", INTRINSIC_PATH, "");
    nh.param<std::string>("extrinsic", EXTRINSIC_PATH, "");
    nh.param<std::string>("roi", PCD_PATH, "");
    
    // Initialize projection if all parameters are provided
    if (!INTRINSIC_PATH.empty() && !EXTRINSIC_PATH.empty() && !PCD_PATH.empty()) {
        ROS_INFO("Loading projection parameters...");
        
        if (loadIntrinsics(INTRINSIC_PATH) && 
            loadExtrinsics(EXTRINSIC_PATH) && 
            loadPCD(PCD_PATH)) {
            ENABLE_PROJECTION = true;
            ROS_INFO("Projection enabled");
        } else {
            ROS_WARN("Failed to load projection parameters, projection disabled");
        }
    } else {
        ROS_INFO("Projection parameters not provided, projection disabled");
    }

    ros::Subscriber sub;
    if (TOPIC_TYPE == "image") {
        sub = nh.subscribe(TOPIC, 1, imageCallback);
        ROS_INFO("Subscribing as sensor_msgs/Image");
    } else {
        sub = nh.subscribe(TOPIC, 1, compressedImageCallback);
        ROS_INFO("Subscribing as sensor_msgs/CompressedImage");
    }

    std::thread writer_thread(ffmpegWriter);

    ROS_INFO("RTSP streaming node with projection (C++) started");
    ros::spin();

    running = false;
    writer_thread.join();
    return 0;
}

