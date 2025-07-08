#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <Eigen/Dense>
#include <vector>
#include <cmath>
#include <algorithm>





namespace Projection{
    cv::Mat pcd_projection_pcl(
        const cv::Mat& img_in,
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& pcd,
        const cv::Mat& intrinsic,              // 3x3 (CV_64F)
        const cv::Mat& distortion,             // Nx1 (CV_64F)
        const Eigen::Matrix4d& transform,      // 4x4
        int point_size = 3,
        const cv::Scalar& color = cv::Scalar(-1, -1, -1), // if all <0: use distance colormap
        int num_bins = 6,
        int colormap = cv::COLORMAP_TURBO,
        double gamma = 0.9
    );
}