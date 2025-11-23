#include "zed_custom_wrapper/zed_custom_node.hpp"
#include <sensor_msgs/image_encodings.hpp>
#include <cmath>
#include <set>
#include <fstream>

using namespace std::chrono_literals;

namespace zed_custom_wrapper
{

ZedCustomNode::ZedCustomNode() : Node("zed_custom_node") {
    // Parameters
    this->declare_parameter("onnx_model_path", "");
    this->declare_parameter("svo_path", ""); // Add SVO path parameter
    this->declare_parameter("resolution", "HD720"); // HD720, HD1080, HD2K, VGA
    this->declare_parameter("fps", 30);
    this->declare_parameter("mapping_enabled", true);
    
    this->declare_parameter("matching_distance_threshold", 1.0);  // meters - distance threshold for matching objects
    this->declare_parameter("expected_object_counts", std::vector<int64_t>{});
}

void ZedCustomNode::initialize() {
    // Load tracking parameters
    matching_distance_threshold_ = this->get_parameter("matching_distance_threshold").as_double();
    
    // Load expected object counts
    auto counts_param = this->get_parameter("expected_object_counts").as_integer_array();
    if (counts_param.size() % 2 == 0) {
        for (size_t i = 0; i < counts_param.size(); i += 2) {
            int class_id = static_cast<int>(counts_param[i]);
            int count = static_cast<int>(counts_param[i + 1]);
            expected_object_counts_[class_id] = count;
            RCLCPP_INFO(this->get_logger(), "Expected %d object(s) of class %d", count, class_id);
        }
    }
    
    RCLCPP_INFO(this->get_logger(), "Matching distance threshold: %.2f meters", matching_distance_threshold_);
    
    // Initialize ZED
    initZed();

    // Publishers
    image_transport::ImageTransport it(this->shared_from_this());
    pub_rgb_ = it.advertise("zed/rgb/image_rect_color", 1);
    pub_depth_ = it.advertise("zed/depth/depth_registered", 1);
    pub_objects_ = this->create_publisher<zed_msgs::msg::ObjectsStamped>("zed/objects", 1);
    pub_odom_ = this->create_publisher<nav_msgs::msg::Odometry>("zed/odom", 1);
    pub_pose_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("zed/pose", 1);
    pub_map_ = this->create_publisher<interfaces::msg::Map>("zed/map", 1);
    pub_markers_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("zed/map_markers", 1);
    
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // Start grab thread
    running_ = true;
    grab_thread_ = std::thread(&ZedCustomNode::grabLoop, this);
}

ZedCustomNode::~ZedCustomNode() {
    running_ = false;
    if (grab_thread_.joinable()) {
        grab_thread_.join();
    }
    zed_.close();
}

void ZedCustomNode::initZed() {
        sl::InitParameters init_params;
        init_params.coordinate_units = sl::UNIT::METER;
        init_params.coordinate_system = sl::COORDINATE_SYSTEM::RIGHT_HANDED_Z_UP_X_FWD;
        init_params.depth_mode = sl::DEPTH_MODE::NEURAL; // "Neural Light" requested
        
        // Force GPU usage (GPU ID 0 for Jetson Nano)
        init_params.sdk_gpu_id = 0;
        init_params.sdk_verbose = 1; // Enable verbose logging to see GPU usage

        std::string svo_path = this->get_parameter("svo_path").as_string();
        if (!svo_path.empty()) {
            init_params.input.setFromSVOFile(svo_path.c_str());
            RCLCPP_INFO(this->get_logger(), "Using SVO file: %s", svo_path.c_str());
        }

        std::string resolution = this->get_parameter("resolution").as_string();
        if (resolution == "HD1080") init_params.camera_resolution = sl::RESOLUTION::HD1080;
        else if (resolution == "HD2K") init_params.camera_resolution = sl::RESOLUTION::HD2K;
        else if (resolution == "VGA") init_params.camera_resolution = sl::RESOLUTION::VGA;
        else init_params.camera_resolution = sl::RESOLUTION::HD720;

        init_params.camera_fps = this->get_parameter("fps").as_int();

        RCLCPP_INFO(this->get_logger(), "Opening ZED with GPU ID: %d", init_params.sdk_gpu_id);
        
        sl::ERROR_CODE err = zed_.open(init_params);
        if (err != sl::ERROR_CODE::SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open ZED: %s", sl::toString(err).c_str());
            throw std::runtime_error("Failed to open ZED");
        }
        
        RCLCPP_INFO(this->get_logger(), "ZED Camera opened successfully on GPU %d", init_params.sdk_gpu_id);

        // Enable Positional Tracking (required for OD and Mapping)
        sl::PositionalTrackingParameters tracking_params;
        tracking_params.enable_area_memory = true;
        err = zed_.enablePositionalTracking(tracking_params);
        if (err != sl::ERROR_CODE::SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "Failed to enable tracking: %s", sl::toString(err).c_str());
        }

        // Enable Object Detection with Custom Model
        std::string onnx_path = this->get_parameter("onnx_model_path").as_string();
        if (!onnx_path.empty()) {
            // Check if file exists
            std::ifstream onnx_file(onnx_path);
            if (!onnx_file.good()) {
                RCLCPP_ERROR(this->get_logger(), "ONNX model file not found: %s", onnx_path.c_str());
                RCLCPP_ERROR(this->get_logger(), "Object Detection will be DISABLED");
            } else {
                RCLCPP_INFO(this->get_logger(), "Attempting to enable Object Detection with ONNX model: %s", onnx_path.c_str());
                
                sl::ObjectDetectionParameters od_params;
                od_params.enable_tracking = true;
                od_params.enable_segmentation = false;
                
                // Use CUSTOM_YOLOLIKE_BOX_OBJECTS for YOLO models
                od_params.detection_model = sl::OBJECT_DETECTION_MODEL::CUSTOM_YOLOLIKE_BOX_OBJECTS;
                
                // Set the custom ONNX file path - MUST be set BEFORE calling enableObjectDetection
                od_params.custom_onnx_file = sl::String(onnx_path.c_str());
                
                // Set the input size - YOLOv8n default is 640x640
                od_params.custom_onnx_dynamic_input_shape = sl::Resolution(640, 640);
                
                // CRITICAL: Force GPU inference (FP16 precision for Jetson Nano)
                od_params.allow_reduced_precision_inference = true; // Use FP16 instead of FP32 for better performance
                
                // Limit max range to reduce processing load
                od_params.max_range = 10.0f; // meters
                
                RCLCPP_INFO(this->get_logger(), "Object Detection params:");
                RCLCPP_INFO(this->get_logger(), "  - Model file: %s", od_params.custom_onnx_file.get());
                RCLCPP_INFO(this->get_logger(), "  - Input shape: %dx%d", 
                    od_params.custom_onnx_dynamic_input_shape.width,
                    od_params.custom_onnx_dynamic_input_shape.height);
                RCLCPP_INFO(this->get_logger(), "  - Reduced precision (FP16): %s", 
                    od_params.allow_reduced_precision_inference ? "enabled" : "disabled");
                RCLCPP_INFO(this->get_logger(), "  - Max range: %.1f meters", od_params.max_range);
                
                err = zed_.enableObjectDetection(od_params);
                if (err != sl::ERROR_CODE::SUCCESS) {
                    RCLCPP_ERROR(this->get_logger(), "Failed to enable Object Detection: %s", sl::toString(err).c_str());
                    RCLCPP_ERROR(this->get_logger(), "Make sure your ONNX model is compatible with ZED SDK");
                    RCLCPP_ERROR(this->get_logger(), "Refer to: https://www.stereolabs.com/docs/object-detection/custom-od");
                } else {
                    RCLCPP_INFO(this->get_logger(), "Object Detection enabled successfully!");
                    RCLCPP_INFO(this->get_logger(), "Running on GPU with FP16 precision for optimal Jetson Nano performance");
                }
            }
        } else {
            RCLCPP_WARN(this->get_logger(), "No ONNX model path provided. Object Detection disabled.");
        }

        // Enable Spatial Mapping
        if (this->get_parameter("mapping_enabled").as_bool()) {
            sl::SpatialMappingParameters mapping_params;
            mapping_params.resolution_meter = 0.05;
            mapping_params.range_meter = 10.0;
            mapping_params.save_texture = false;
            err = zed_.enableSpatialMapping(mapping_params);
            if (err != sl::ERROR_CODE::SUCCESS) {
                RCLCPP_ERROR(this->get_logger(), "Failed to enable Spatial Mapping: %s", sl::toString(err).c_str());
            } else {
                RCLCPP_INFO(this->get_logger(), "Spatial Mapping enabled");
            }
        }
}

void ZedCustomNode::grabLoop() {
    sl::RuntimeParameters runtime_params;
    sl::Objects objects;
    sl::Mat image, depth;
    sl::Pose pose;
    
    RCLCPP_INFO(this->get_logger(), "Grab loop started - waiting for frames...");

    while (running_ && rclcpp::ok()) {
        if (zed_.grab(runtime_params) == sl::ERROR_CODE::SUCCESS) {
            // Retrieve Data
            zed_.retrieveImage(image, sl::VIEW::LEFT);
            zed_.retrieveMeasure(depth, sl::MEASURE::DEPTH);
            
            // Retrieve Pose
            zed_.getPosition(pose, sl::REFERENCE_FRAME::WORLD);
            
            // Retrieve Objects
            sl::ERROR_CODE obj_err = zed_.retrieveObjects(objects, sl::ObjectDetectionRuntimeParameters());
            
            // Debug: Log object retrieval status
            static int frame_count = 0;
            if (++frame_count % 30 == 0) {  // Log every 30 frames (~1 second at 30fps)
                RCLCPP_INFO(this->get_logger(), "Frame %d: Object retrieval status: %s, Objects detected: %zu", 
                    frame_count, sl::toString(obj_err).c_str(), objects.object_list.size());
            }

            // Publish Data
            auto timestamp = this->now();
            publishImage(image, pub_rgb_, "zed_left_camera_frame", timestamp);
            publishImage(depth, pub_depth_, "zed_left_camera_frame", timestamp); // Depth is usually float32
            publishObjects(objects, "zed_left_camera_frame", timestamp);
            publishPose(pose, timestamp);
            publishMap(objects, timestamp);
            publishMapMarkers();  // Visualize tracked objects in RViz

        } else {
            std::this_thread::sleep_for(1ms);
        }
    }
}

void ZedCustomNode::publishImage(sl::Mat& mat, image_transport::Publisher& pub, const std::string& frame_id, rclcpp::Time timestamp) {
    sensor_msgs::msg::Image msg;
    msg.header.stamp = timestamp;
    msg.header.frame_id = frame_id;
    msg.height = mat.getHeight();
    msg.width = mat.getWidth();
    
    int channels = mat.getChannels();
    if (channels == 4) {
        msg.encoding = sensor_msgs::image_encodings::BGRA8;
        msg.step = mat.getStepBytes();
        msg.data.resize(msg.step * msg.height);
        memcpy(msg.data.data(), mat.getPtr<sl::uchar1>(), msg.step * msg.height);
    } else if (channels == 1) {
         // Assuming float depth
        msg.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
        msg.step = mat.getStepBytes();
        msg.data.resize(msg.step * msg.height);
        memcpy(msg.data.data(), mat.getPtr<sl::uchar1>(), msg.step * msg.height);
    }
    
    pub.publish(msg);
}

void ZedCustomNode::publishObjects(sl::Objects& sl_objects, const std::string& frame_id, rclcpp::Time timestamp) {
    zed_msgs::msg::ObjectsStamped msg;
    msg.header.stamp = timestamp;
    msg.header.frame_id = frame_id;

    if (sl_objects.object_list.size() > 0) {
        RCLCPP_INFO(this->get_logger(), "Detected %zu object(s)", sl_objects.object_list.size());
    }

    for (auto& sl_obj : sl_objects.object_list) {
        zed_msgs::msg::Object obj;
        obj.label = sl::toString(sl_obj.label);
        obj.label_id = sl_obj.raw_label;
        obj.confidence = sl_obj.confidence;
        
        // Check for valid position values
        if (!std::isfinite(sl_obj.position.x) || !std::isfinite(sl_obj.position.y) || !std::isfinite(sl_obj.position.z)) {
            RCLCPP_WARN(this->get_logger(), "Skipping object with invalid position (NaN or Inf)");
            continue;
        }
        
        // Log detected object details
        RCLCPP_INFO(this->get_logger(), 
            "Object detected - Label: %s (ID: %d), Confidence: %.2f%%, Position: [%.2f, %.2f, %.2f]",
            obj.label.c_str(), 
            obj.label_id, 
            obj.confidence * 100.0,
            sl_obj.position.x,
            sl_obj.position.y,
            sl_obj.position.z);
        
        // Position
        obj.position[0] = sl_obj.position.x;
        obj.position[1] = sl_obj.position.y;
        obj.position[2] = sl_obj.position.z;
        
        // Dimensions
        obj.dimensions_3d[0] = sl_obj.dimensions.x;
        obj.dimensions_3d[1] = sl_obj.dimensions.y;
        obj.dimensions_3d[2] = sl_obj.dimensions.z;

        // Bounding Box 2D
        for (int i = 0; i < 4; i++) {
            zed_msgs::msg::Keypoint2Di kp;
            kp.kp[0] = sl_obj.bounding_box_2d[i].x;
            kp.kp[1] = sl_obj.bounding_box_2d[i].y;
            obj.bounding_box_2d.corners[i] = kp;
        }

        // Bounding Box 3D
        for (int i = 0; i < 8; i++) {
            zed_msgs::msg::Keypoint3D kp;
            kp.kp[0] = sl_obj.bounding_box[i].x;
            kp.kp[1] = sl_obj.bounding_box[i].y;
            kp.kp[2] = sl_obj.bounding_box[i].z;
            obj.bounding_box_3d.corners[i] = kp;
        }
        
        msg.objects.push_back(obj);
    }
    pub_objects_->publish(msg);
}

void ZedCustomNode::publishPose(sl::Pose& pose, rclcpp::Time timestamp) {
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = timestamp;
    t.header.frame_id = "map";
    t.child_frame_id = "zed_base_link";

    sl::Translation trans = pose.getTranslation();
    sl::Orientation quat = pose.getOrientation();

    t.transform.translation.x = trans.x;
    t.transform.translation.y = trans.y;
    t.transform.translation.z = trans.z;
    t.transform.rotation.x = quat.ox;
    t.transform.rotation.y = quat.oy;
    t.transform.rotation.z = quat.oz;
    t.transform.rotation.w = quat.ow;

    tf_broadcaster_->sendTransform(t);

    // Publish Odometry
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = timestamp;
    odom.header.frame_id = "map";
    odom.child_frame_id = "zed_base_link";
    odom.pose.pose.position.x = trans.x;
    odom.pose.pose.position.y = trans.y;
    odom.pose.pose.position.z = trans.z;
    odom.pose.pose.orientation.x = quat.ox;
    odom.pose.pose.orientation.y = quat.oy;
    odom.pose.pose.orientation.z = quat.oz;
    odom.pose.pose.orientation.w = quat.ow;
    pub_odom_->publish(odom);

    // Publish PoseStamped
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp = timestamp;
    pose_msg.header.frame_id = "map";
    pose_msg.pose = odom.pose.pose;
    pub_pose_->publish(pose_msg);
}

int ZedCustomNode::findClosestObject(int cls, const sl::float3& position) {
    int closest_idx = -1;
    double min_distance = matching_distance_threshold_;
    
    // Search through existing objects of the same class
    for (size_t i = 0; i < map_objects_.size(); ++i) {
        const auto& obj = map_objects_[i];
        
        // Only consider objects of the same class
        if (obj.cls != cls) {
            continue;
        }
        
        // Calculate Euclidean distance
        double dx = obj.bbox.center.position.x - position.x;
        double dy = obj.bbox.center.position.y - position.y;
        double dz = obj.bbox.center.position.z - position.z;
        double distance = std::sqrt(dx*dx + dy*dy + dz*dz);
        
        // Keep track of closest object within threshold
        if (distance < min_distance) {
            min_distance = distance;
            closest_idx = i;
        }
    }
    
    return closest_idx;
}

void ZedCustomNode::publishMap(sl::Objects& sl_objects, rclcpp::Time timestamp) {
    // Create Map message
    interfaces::msg::Map map_msg;
    
    // Process each detected object
    for (auto& sl_obj : sl_objects.object_list) {
        // Skip objects with invalid positions
        if (!std::isfinite(sl_obj.position.x) || 
            !std::isfinite(sl_obj.position.y) || 
            !std::isfinite(sl_obj.position.z)) {
            continue;
        }
        
        int class_id = sl_obj.raw_label;  // YOLO class ID
        
        // Skip if this class is not in expected_object_counts (filter unwanted classes)
        if (!expected_object_counts_.empty() && 
            expected_object_counts_.find(class_id) == expected_object_counts_.end()) {
            RCLCPP_DEBUG(this->get_logger(), 
                "Ignoring detected object of class %d (not in expected_object_counts)", class_id);
            continue;
        }
        
        // Create MapObject with current detection
        interfaces::msg::MapObject map_obj;
        map_obj.cls = class_id;
        
        // Fill bounding box
        map_obj.bbox.center.position.x = sl_obj.position.x;
        map_obj.bbox.center.position.y = sl_obj.position.y;
        map_obj.bbox.center.position.z = sl_obj.position.z;
        
        // Set orientation (identity quaternion for now)
        map_obj.bbox.center.orientation.w = 1.0;
        map_obj.bbox.center.orientation.x = 0.0;
        map_obj.bbox.center.orientation.y = 0.0;
        map_obj.bbox.center.orientation.z = 0.0;
        
        // Set bounding box dimensions from ZED
        map_obj.bbox.size.x = sl_obj.dimensions.x;
        map_obj.bbox.size.y = sl_obj.dimensions.y;
        map_obj.bbox.size.z = sl_obj.dimensions.z;
        
        // Find closest existing object of same class within threshold
        int closest_idx = findClosestObject(class_id, sl_obj.position);
        
        if (closest_idx >= 0) {
            // Update existing object (static object moved slightly or same position)
            map_objects_[closest_idx] = map_obj;
            RCLCPP_DEBUG(this->get_logger(), 
                "Updated existing object at index %d, Class=%d, Distance match", 
                closest_idx, class_id);
        } else {
            // Check if we've reached the expected count for this class
            int current_count = 0;
            for (const auto& obj : map_objects_) {
                if (obj.cls == class_id) {
                    current_count++;
                }
            }
            
            // Get expected count for this class
            auto it = expected_object_counts_.find(class_id);
            int expected_count = (it != expected_object_counts_.end()) ? it->second : 0;
            
            // Only add if we haven't reached the expected count
            if (current_count < expected_count) {
                // Add new object
                size_t new_index = map_objects_.size();
                map_objects_.push_back(map_obj);
                RCLCPP_INFO(this->get_logger(), 
                    "New static object: Index=%zu, Class=%d, Position=[%.2f, %.2f, %.2f]", 
                    new_index, class_id, 
                    sl_obj.position.x, sl_obj.position.y, sl_obj.position.z);
            } else {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                    "Detected object of class %d but already have %d/%d expected. Ignoring.",
                    class_id, current_count, expected_count);
            }
        }
    }
    
    // Publish current map state
    map_msg.objects = map_objects_;
    
    // Set map bounds (can be calculated from spatial mapping data if needed)
    // For now, use a default large bounding box
    map_msg.map_bounds.center.position.x = 0.0;
    map_msg.map_bounds.center.position.y = 0.0;
    map_msg.map_bounds.center.position.z = 0.0;
    map_msg.map_bounds.center.orientation.w = 1.0;
    map_msg.map_bounds.size.x = 100.0;  // 100m x 100m x 10m default
    map_msg.map_bounds.size.y = 100.0;
    map_msg.map_bounds.size.z = 10.0;
    
    pub_map_->publish(map_msg);
    
    if (map_objects_.size() > 0) {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                            "Publishing map with %zu tracked object(s)", map_objects_.size());
    }
}

void ZedCustomNode::publishMapMarkers() {
    visualization_msgs::msg::MarkerArray marker_array;
    
    // YOLO class names (COCO dataset)
    static const std::vector<std::string> class_names = {
        "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck", "boat",
        "traffic light", "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat", "dog",
        "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella",
        "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball", "kite",
        "baseball bat", "baseball glove", "skateboard", "surfboard", "tennis racket", "bottle",
        "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple", "sandwich",
        "orange", "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair", "couch",
        "potted plant", "bed", "dining table", "toilet", "tv", "laptop", "mouse", "remote",
        "keyboard", "cell phone", "microwave", "oven", "toaster", "sink", "refrigerator", "book",
        "clock", "vase", "scissors", "teddy bear", "hair drier", "toothbrush"
    };
    
    // Create a marker for each tracked object
    for (size_t i = 0; i < map_objects_.size(); ++i) {
        const auto& obj = map_objects_[i];
        
        // Create bounding box marker (CUBE)
        visualization_msgs::msg::Marker bbox_marker;
        bbox_marker.header.frame_id = "zed_base_link";
        bbox_marker.header.stamp = this->now();
        bbox_marker.ns = "map_objects_bbox";
        bbox_marker.id = i * 2;  // Even IDs for bounding boxes
        bbox_marker.type = visualization_msgs::msg::Marker::CUBE;
        bbox_marker.action = visualization_msgs::msg::Marker::ADD;
        
        // Position and orientation from bounding box
        bbox_marker.pose.position = obj.bbox.center.position;
        bbox_marker.pose.orientation = obj.bbox.center.orientation;
        
        // Size from bounding box
        bbox_marker.scale = obj.bbox.size;
        
        // Color based on tracking ID (unique color per object)
        // Use HSV to RGB conversion for distinct colors
        float hue = (i * 137.5f) / 360.0f;  // Golden angle for good color distribution
        bbox_marker.color.r = std::abs(std::sin(hue * 6.28f));
        bbox_marker.color.g = std::abs(std::sin((hue + 0.333f) * 6.28f));
        bbox_marker.color.b = std::abs(std::sin((hue + 0.666f) * 6.28f));
        bbox_marker.color.a = 0.3;  // Semi-transparent
        
        bbox_marker.lifetime = rclcpp::Duration::from_seconds(0.5);
        marker_array.markers.push_back(bbox_marker);
        
        // Create text label marker
        visualization_msgs::msg::Marker text_marker;
        text_marker.header.frame_id = "zed_base_link";
        text_marker.header.stamp = this->now();
        text_marker.ns = "map_objects_label";
        text_marker.id = i * 2 + 1;  // Odd IDs for labels
        text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        text_marker.action = visualization_msgs::msg::Marker::ADD;
        
        // Position text above the bounding box
        text_marker.pose.position = obj.bbox.center.position;
        text_marker.pose.position.z += obj.bbox.size.z / 2.0 + 0.3;  // 30cm above box
        text_marker.pose.orientation.w = 1.0;
        
        // Text content: class name and tracking ID
        std::string class_name = "unknown";
        if (obj.cls >= 0 && obj.cls < static_cast<int>(class_names.size())) {
            class_name = class_names[obj.cls];
        }
        text_marker.text = class_name + " [ID:" + std::to_string(i) + "]";
        
        // Text size and color
        text_marker.scale.z = 0.2;  // Text height
        text_marker.color.r = 1.0;
        text_marker.color.g = 1.0;
        text_marker.color.b = 1.0;
        text_marker.color.a = 1.0;
        
        text_marker.lifetime = rclcpp::Duration::from_seconds(0.5);
        marker_array.markers.push_back(text_marker);
    }
    
    // If no objects, send a delete-all marker to clear old markers
    if (map_objects_.empty()) {
        visualization_msgs::msg::Marker delete_marker;
        delete_marker.header.frame_id = "zed_base_link";
        delete_marker.header.stamp = this->now();
        delete_marker.ns = "map_objects_bbox";
        delete_marker.id = 0;
        delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
        marker_array.markers.push_back(delete_marker);
    }
    
    pub_markers_->publish(marker_array);
}

}  // namespace zed_custom_wrapper

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<zed_custom_wrapper::ZedCustomNode>();
    node->initialize();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
