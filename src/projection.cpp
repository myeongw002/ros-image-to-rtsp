#include "rtsp_image_sender/projection.hpp"



namespace Projection{

    cv::Mat pcd_projection_pcl(
        const cv::Mat& img_in,
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& pcd,
        const cv::Mat& intrinsic,              // 3x3 (CV_64F)
        const cv::Mat& distortion,             // Nx1 (CV_64F)
        const Eigen::Matrix4d& transform,      // 4x4
        int point_size ,
        const cv::Scalar& color , // if all <0: use distance colormap
        int num_bins ,
        int colormap ,
        double gamma 
    ) {
        if (pcd->empty())
            return img_in.clone();

        // 1. Transform point cloud to camera coordinates
        std::vector<Eigen::Vector3d> pts_cam;
        for (const auto& pt : pcd->points) {
            Eigen::Vector4d pt_h(pt.x, pt.y, pt.z, 1.0);
            Eigen::Vector4d pt_cam = transform * pt_h;
            if (pt_cam(2) > 0)
                pts_cam.emplace_back(pt_cam.head<3>());
        }
        if (pts_cam.empty())
            return img_in.clone();

        // 2. Distance calculation and color mapping
        std::vector<double> dists(pts_cam.size());
        for (size_t i = 0; i < pts_cam.size(); ++i)
            dists[i] = pow(pts_cam[i].norm(), gamma);

        // Normalize distances to 0~255
        double min_dist = *std::min_element(dists.begin(), dists.end());
        double max_dist = *std::max_element(dists.begin(), dists.end());
        std::vector<uchar> dist_norm(pts_cam.size());
        for (size_t i = 0; i < dists.size(); ++i)
            dist_norm[i] = static_cast<uchar>(255.0 * (dists[i] - min_dist) / (max_dist - min_dist + 1e-12));

        // Step effect (binning)
        int step = 256 / num_bins;
        std::vector<uchar> dist_quant(pts_cam.size());
        for (size_t i = 0; i < dists.size(); ++i)
            dist_quant[i] = (dist_norm[i] / step) * step + step / 2;

        // Apply colormap
        cv::Mat dist_quant_mat(dist_quant.size(), 1, CV_8U, dist_quant.data());
        cv::Mat colors_map;
        cv::applyColorMap(dist_quant_mat, colors_map, colormap);

        // 3. Project and render
        cv::Mat img_out = img_in.clone();
        for (size_t i = 0; i < pts_cam.size(); ++i) {
            std::vector<cv::Point3f> objectPoints = {
                cv::Point3f(static_cast<float>(pts_cam[i](0)),
                            static_cast<float>(pts_cam[i](1)),
                            static_cast<float>(pts_cam[i](2)))
            };
            std::vector<cv::Point2f> imagePoints;
            cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64F);
            cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64F);
            cv::projectPoints(objectPoints, rvec, tvec, intrinsic, distortion, imagePoints);
            int x = static_cast<int>(imagePoints[0].x);
            int y = static_cast<int>(imagePoints[0].y);

            // Check if inside image
            if (x >= 0 && x < img_out.cols && y >= 0 && y < img_out.rows) {
                cv::Scalar draw_color;
                if (color[0] < 0)
                    draw_color = cv::Scalar(colors_map.at<cv::Vec3b>(i, 0)[0], colors_map.at<cv::Vec3b>(i, 0)[1], colors_map.at<cv::Vec3b>(i, 0)[2]);
                else
                    draw_color = color;
                cv::circle(img_out, cv::Point(x, y), point_size, draw_color, -1);
            }
        }
        return img_out;
    }


}


