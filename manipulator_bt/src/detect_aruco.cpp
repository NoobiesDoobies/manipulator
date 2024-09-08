#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/bool.hpp>

#include <opencv2/aruco.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>  // Include cv_bridge header

class ArucoDetectorNode : public rclcpp::Node
{
public:
    ArucoDetectorNode() : Node("aruco_detector_node")
    {
        // Initialize camera parameters
        camera_matrix_ = (cv::Mat_<double>(3, 3) << 337.2084410968044, 0.0, 320.5,
                                                    0.0, 337.2084410968044, 200.5,
                                                    0.0, 0.0, 1.0);
        dist_coeffs_ = (cv::Mat_<double>(1, 5) << 0.0, 0.0, 0.0, 0.0, 0.0);

        // Create publisher for is_detected topic
        is_detected_publisher_ = this->create_publisher<std_msgs::msg::Bool>("/aruco_detect/is_detected", 10);

        // Create publisher for pose topic
        pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/aruco_detect/object_pose", 10);

        // Create publisher for processed image topic
        image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/aruco_detect/image", 10);

        // Create subscriber for image_raw topic
        image_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
                "omni_bot/front/image_raw", 10, std::bind(&ArucoDetectorNode::imageCallback, this, std::placeholders::_1));

        // depth_image_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
        //     "omni_bot/front/depth/image_raw", 10, std::bind(&ArucoDetectorNode::depthImageCallback, this, std::placeholders::_1));

        depth_image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/aruco_detect/depth/image", 10);
    }

private:
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        // Convert sensor_msgs::Image to cv::Mat
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        // Resize image if too large
        cv::Mat resized_image;
        float scale = 1;
        cv::resize(cv_ptr->image, resized_image, cv::Size(), scale, scale);  // Resizing by 50%

        // Detect ArUco markers
        std::vector<int> marker_ids;
        std::vector<std::vector<cv::Point2f>> marker_corners;
        cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::makePtr<cv::aruco::DetectorParameters>();
        cv::Ptr<cv::aruco::Dictionary> dictionary = cv::makePtr<cv::aruco::Dictionary>(cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50));
        cv::aruco::detectMarkers(resized_image, dictionary, marker_corners, marker_ids, parameters);

        // Draw detected markers on the image
        cv::aruco::drawDetectedMarkers(resized_image, marker_corners, marker_ids);

        // Draw the center coordinate of each marker
        for (size_t i = 0; i < marker_corners.size(); ++i)
        {
            cv::Point2f center(0, 0);
            for (const auto& corner : marker_corners[i])
            {
                center += corner;
            }
            center *= (1.0 / marker_corners[i].size());
            object_center_ = center;  // Scale up the center coordinate
            cv::circle(resized_image, center, 5, cv::Scalar(0, 0, 255), -1);  // Draw a red circle at the center

            // Define the 3D coordinates of the marker corners
            std::vector<cv::Point3f> marker_corners_3d = {
                cv::Point3f(-0.05, 0.05, 0),  // Top-left corner
                cv::Point3f(0.05, 0.05, 0),   // Top-right corner
                cv::Point3f(0.05, -0.05, 0),  // Bottom-right corner
                cv::Point3f(-0.05, -0.05, 0)  // Bottom-left corner
            };

            // Estimate the pose of the marker
            cv::Mat rvec, tvec;
            bool success = cv::solvePnP(marker_corners_3d, marker_corners[i], camera_matrix_, dist_coeffs_, rvec, tvec);

            if (success)
            {
                
                // Convert the pose to a ROS message
                geometry_msgs::msg::PoseStamped pose_msg;
                pose_msg.header.stamp = this->now();
                pose_msg.header.frame_id = "camera_frame";

                // Convert rotation vector to quaternion
                cv::Mat rotation_matrix;
                cv::Rodrigues(rvec, rotation_matrix);
                cv::Mat quaternion = cv::Mat::zeros(4, 1, CV_64F);
                cv::Rodrigues(rotation_matrix, quaternion);

                pose_msg.pose.position.x = tvec.at<double>(0);
                pose_msg.pose.position.y = tvec.at<double>(1);
                pose_msg.pose.position.z = tvec.at<double>(2);

                pose_msg.pose.orientation.x = quaternion.at<double>(0);
                pose_msg.pose.orientation.y = quaternion.at<double>(1);
                pose_msg.pose.orientation.z = quaternion.at<double>(2);
                pose_msg.pose.orientation.w = quaternion.at<double>(3);

                // Publish the pose
                pose_publisher_->publish(pose_msg);
            }
        }

        // Publish is_detected
        std_msgs::msg::Bool is_detected;
        is_detected.data = !marker_ids.empty();
        is_detected_publisher_->publish(is_detected);

        // Convert cv::Mat back to sensor_msgs::Image and publish
        sensor_msgs::msg::Image::SharedPtr processed_image_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", resized_image).toImageMsg();
        image_publisher_->publish(*processed_image_msg);
    }

    void depthImageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
        }
        catch (cv_bridge::Exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        // Debugging: Log image dimensions and type
        // RCLCPP_INFO(this->get_logger(), "Depth image received: width=%d, height=%d, encoding=%s",
        //             msg->width, msg->height, msg->encoding.c_str());

        // Resize depth image
        cv::Mat resized_image;
        float scale = 1;

        cv::resize(cv_ptr->image, resized_image, cv::Size(), scale, scale);  // Resizing by 50%

        // Get depth information of the center of the detected object
        if (object_center_.x != 0 && object_center_.y != 0)
        {
            cv::Point2i center = object_center_;
            // Use 3x3 kernel to get the average depth value
            cv::Mat depth_roi = resized_image(cv::Rect(center.x - 1, center.y - 1, 3, 3));
            // Print out the depth values
            for (int i = 0; i < depth_roi.rows; i++)
            {
                // for (int j = 0; j < depth_roi.cols; j++)
                // {
                //     RCLCPP_INFO(this->get_logger(), "%f", depth_roi.at<float>(i, j));
                // }
            }
            cv::Scalar mean_depth = cv::mean(depth_roi);
            // RCLCPP_INFO(this->get_logger(), "Mean depth at center: %f", mean_depth[0]);
        }

        // Draw the center coordinate of each marker
        cv::circle(resized_image, object_center_, 5, cv::Scalar(0, 0, 255), -1);  // Draw a red circle at the center

        // Convert cv::Mat back to sensor_msgs::Image and publish
        sensor_msgs::msg::Image::SharedPtr processed_image_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "32FC1", resized_image).toImageMsg();
        depth_image_publisher_->publish(*processed_image_msg);
    }

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr is_detected_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_image_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_image_subscriber_;
    cv::Point2f object_center_;
    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArucoDetectorNode>());
    rclcpp::shutdown();
    return 0;
}