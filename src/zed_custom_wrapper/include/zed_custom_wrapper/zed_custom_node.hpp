#ifndef ZED_CUSTOM_WRAPPER__ZED_CUSTOM_NODE_HPP_
#define ZED_CUSTOM_WRAPPER__ZED_CUSTOM_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sl/Camera.hpp>
#include <zed_msgs/msg/objects_stamped.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <interfaces/msg/map.hpp>
#include <interfaces/msg/map_object.hpp>
#include <vision_msgs/msg/bounding_box3_d.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <image_transport/image_transport.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <thread>
#include <chrono>
#include <atomic>
#include <memory>
#include <map>
#include <vector>

namespace zed_custom_wrapper
{

class ZedCustomNode : public rclcpp::Node
{
public:
    /**
     * @brief Constructor for ZedCustomNode
     */
    ZedCustomNode();

    /**
     * @brief Initialize the ZED camera and start processing
     */
    void initialize();

    /**
     * @brief Destructor - cleanup resources
     */
    ~ZedCustomNode();

private:
    /**
     * @brief Initialize ZED camera with configured parameters
     */
    void initZed();

    /**
     * @brief Main grab loop running in separate thread
     */
    void grabLoop();

    /**
     * @brief Publish image data
     * @param mat ZED Mat containing image data
     * @param pub Image transport publisher
     * @param frame_id Frame ID for the image
     * @param timestamp Timestamp for the message
     */
    void publishImage(
        sl::Mat& mat,
        image_transport::Publisher& pub,
        const std::string& frame_id,
        rclcpp::Time timestamp);

    /**
     * @brief Publish detected objects
     * @param sl_objects ZED Objects structure
     * @param frame_id Frame ID for the objects
     * @param timestamp Timestamp for the message
     */
    void publishObjects(
        sl::Objects& sl_objects,
        const std::string& frame_id,
        rclcpp::Time timestamp);

    /**
     * @brief Publish pose, odometry, and TF transform
     * @param pose ZED Pose structure
     * @param timestamp Timestamp for the messages
     */
    void publishPose(
        sl::Pose& pose,
        rclcpp::Time timestamp);

    /**
     * @brief Publish Map message with tracked objects
     * @param sl_objects ZED Objects structure
     * @param timestamp Timestamp for the message
     */
    void publishMap(
        sl::Objects& sl_objects,
        rclcpp::Time timestamp);

    /**
     * @brief Publish RViz markers for visualizing tracked objects
     */
    void publishMapMarkers();

    /**
     * @brief Find the closest existing object of the same class within distance threshold
     * @param cls Object class ID
     * @param position Current object position
     * @return Index of closest object, or -1 if none found within threshold
     */
    int findClosestObject(int cls, const sl::float3& position);

    // ZED Camera instance
    sl::Camera zed_;

    // Threading
    std::thread grab_thread_;
    std::atomic<bool> running_;

    // Publishers
    image_transport::Publisher pub_rgb_;
    image_transport::Publisher pub_depth_;
    rclcpp::Publisher<zed_msgs::msg::ObjectsStamped>::SharedPtr pub_objects_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_pose_;
    rclcpp::Publisher<interfaces::msg::Map>::SharedPtr pub_map_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_markers_;
    
    // TF Broadcaster
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // Object tracking state
    // Vector of tracked objects (index represents unique object position in map)
    std::vector<interfaces::msg::MapObject> map_objects_;
    
    // Tracking parameters
    double matching_distance_threshold_;
    std::map<int, int> expected_object_counts_;  // class_id -> expected count
};

}  // namespace zed_custom_wrapper

#endif  // ZED_CUSTOM_WRAPPER__ZED_CUSTOM_NODE_HPP_
