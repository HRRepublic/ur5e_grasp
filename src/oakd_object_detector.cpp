#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <geometry_msgs/PointStamped.h>
#include <iostream>
#include <map>
#include <vector>
#include <string>
#include <thread>

// 深度获取函数
float getDepth(const cv::Mat& depth_image, int u, int v) {
    if (u >= 0 && u < depth_image.cols && v >= 0 && v < depth_image.rows) {
        return depth_image.at<uint16_t>(v, u) / 1000.0; // 深度以毫米存储，转换为米
    }
    return 0.0;
}

class ObjectDetector {
public:
    ObjectDetector() : current_color_("red") {
        // 订阅 RGB 图像和深度图像
        image_sub_ = nh_.subscribe("/stereo_inertial_publisher/color/image", 1, &ObjectDetector::imageCallback, this);
        depth_sub_ = nh_.subscribe("/stereo_inertial_publisher/stereo/depth", 1, &ObjectDetector::depthCallback, this);
        camera_info_sub_ = nh_.subscribe("/stereo_inertial_publisher/color/camera_info", 1, &ObjectDetector::cameraInfoCallback, this);

        // 发布检测结果
        point_pub_ = nh_.advertise<geometry_msgs::PointStamped>("/target_position", 1);

        // 相机内参（自己标的）
        fx_ = 547.7535765826173;
        fy_ = 529.4238482170311;
        cx_ = 657.0123315602384;
        cy_ = 404.37890956268967;

        // 定义颜色范围（根据需要调整）
        color_ranges_ = {
            {"red", {cv::Scalar(160, 150, 100), cv::Scalar(180, 255, 255)}},
            {"yellow", {cv::Scalar(20, 100, 100), cv::Scalar(30, 255, 255)}},
            {"blue", {cv::Scalar(100, 150, 0), cv::Scalar(140, 255, 255)}}
        };
    }

    void cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg) {
        fx_ = msg->K[0];
        fy_ = msg->K[4];
        cx_ = msg->K[2];
        cy_ = msg->K[5];
        // ROS_INFO("Camera Info: fx=%f, fy=%f, cx=%f, cy=%f", fx_, fy_, cx_, cy_);
    }

    void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
        try {
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            image_ = cv_ptr->image;
            processImages();
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
        }
    }

    void depthCallback(const sensor_msgs::ImageConstPtr& msg) {
        try {
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
            depth_image_ = cv_ptr->image;
            processImages();
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
        }
    }

    void processImages() {
        if (image_.empty() || depth_image_.empty()) {
            return;
        }

        // 转换为 HSV 颜色空间
        cv::Mat hsv;
        cv::cvtColor(image_, hsv, cv::COLOR_BGR2HSV);

        std::map<std::string, std::tuple<float, float, float>> results;

        const std::string color = current_color_;
        const cv::Scalar& lower = color_ranges_[color].first;
        const cv::Scalar& upper = color_ranges_[color].second;

        // 创建掩码
        cv::Mat mask;
        cv::inRange(hsv, lower, upper, mask);

        // 去噪声
        cv::morphologyEx(mask, mask, cv::MORPH_OPEN, cv::Mat::ones(3, 3, CV_8U));

        // 检测轮廓
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        // 绘制轮廓并显示
        cv::Mat output;
        image_.copyTo(output);
        cv::drawContours(output, contours, -1, cv::Scalar(0, 255, 0), 2);

        for (const auto& contour : contours) {
            // 计算质心
            cv::Moments M = cv::moments(contour);
            if (M.m00 > 0) {
                int u = static_cast<int>(M.m10 / M.m00);
                int v = static_cast<int>(M.m01 / M.m00);

                // 绘制质心
                cv::circle(output, cv::Point(u, v), 5, cv::Scalar(0, 255, 0), -1);
                cv::imshow("Detected Contours", output);
                cv::waitKey(1);

                // 获取深度值
                // float Z = getDepth(depth_image_, u, v);

                // 计算深度值
                float square_size = 0.019; // 正方形边长，单位：米
                float focal_length = (fx_ + fy_) / 2.0; // 近似焦距
                float contour_area = cv::contourArea(contour);
                float Z = (square_size * focal_length) / std::sqrt(contour_area);
                if (Z > 0) {
                    float X = (u - cx_) * Z / fx_;
                    float Y = (v - cy_) * Z / fy_;
                    results[color] = std::make_tuple(X, Y, Z);

                    // 发布检测结果
                    geometry_msgs::PointStamped point_msg;
                    point_msg.header.stamp = ros::Time::now();
                    point_msg.header.frame_id = "camera_link";
                    point_msg.point.x = X;
                    point_msg.point.y = Y;
                    point_msg.point.z = Z;
                    point_pub_.publish(point_msg);
                }
            }
        }

        // 打印结果
        for (const auto& result : results) {
            const std::string& color = result.first;
            float X, Y, Z;
            std::tie(X, Y, Z) = result.second;

            ROS_INFO("Detected %s object at (X: %f, Y: %f, Z: %f)", color.c_str(), X, Y, Z);
        }
    }

    void setColor(const std::string& color) {
        if (color_ranges_.find(color) != color_ranges_.end()) {
            current_color_ = color;
            ROS_INFO("Color set to %s", color.c_str());
        } else {
            ROS_WARN("Color %s not found in color ranges", color.c_str());
        }
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber image_sub_;
    ros::Subscriber depth_sub_;
    ros::Subscriber camera_info_sub_;
    ros::Publisher point_pub_;
    cv::Mat image_;
    cv::Mat depth_image_;
    float fx_, fy_, cx_, cy_;
    std::map<std::string, std::pair<cv::Scalar, cv::Scalar>> color_ranges_;
    std::string current_color_;
};

void keyboardListener(ObjectDetector& detector) {
    char input;
    while (ros::ok()) {
        std::cin >> input;
        if (input == 'r') {
            detector.setColor("red");
        } else if (input == 'y') {
            detector.setColor("yellow");
        } else if (input == 'b') {
            detector.setColor("blue");
        }
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "oakd_object_detector_node");
    ObjectDetector detector;
    std::thread keyboard_thread(keyboardListener, std::ref(detector));
    ros::spin();
    return 0;
}