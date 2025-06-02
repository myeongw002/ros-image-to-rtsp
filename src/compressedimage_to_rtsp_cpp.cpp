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

int WIDTH = 640;
int HEIGHT = 480;
int FPS = 30;
std::string RTSP_URL = "rtsp://localhost:8554/stream";
std::string TOPIC = "/camera1/image_color/compressed";
std::string TOPIC_TYPE = "compressed"; // "image" or "compressed"

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

// sensor_msgs/Image 콜백
void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    try {
        cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
        cv::Mat img = cv_ptr->image;
        if (img.empty()) {
            ROS_WARN("Image 토픽: 디코딩 실패");
            return;
        }
        if (img.cols != WIDTH || img.rows != HEIGHT)
            cv::resize(img, img, cv::Size(WIDTH, HEIGHT));
        std::lock_guard<std::mutex> lock(img_mutex);
        latest_img = img.clone();
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge 예외: %s", e.what());
    }
}

// sensor_msgs/CompressedImage 콜백
void compressedImageCallback(const sensor_msgs::CompressedImageConstPtr& msg) {
    try {
        cv::Mat img = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
        if (img.empty()) {
            ROS_WARN("CompressedImage 토픽: 디코딩 실패");
            return;
        }
        if (img.cols != WIDTH || img.rows != HEIGHT)
            cv::resize(img, img, cv::Size(WIDTH, HEIGHT));
        std::lock_guard<std::mutex> lock(img_mutex);
        latest_img = img.clone();
    } catch (cv::Exception& e) {
        ROS_ERROR("cv::imdecode 예외: %s", e.what());
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
        ROS_ERROR("FFmpeg 프로세스 실행 실패");
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
    ros::init(argc, argv, "compressedimage_to_rtsp_cpp");
    ros::NodeHandle nh("~");

    std::string config_path;
    nh.param<std::string>("config", config_path, "config.yaml");
    loadConfig(config_path);

    ros::Subscriber sub;
    if (TOPIC_TYPE == "image") {
        sub = nh.subscribe(TOPIC, 1, imageCallback);
        ROS_INFO("sensor_msgs/Image 타입 구독");
    } else {
        sub = nh.subscribe(TOPIC, 1, compressedImageCallback);
        ROS_INFO("sensor_msgs/CompressedImage 타입 구독");
    }

    std::thread writer_thread(ffmpegWriter);

    ROS_INFO("RTSP 송출 노드(C++) 시작");
    ros::spin();

    running = false;
    writer_thread.join();
    return 0;
}

