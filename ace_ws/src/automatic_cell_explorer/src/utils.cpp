
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include "automatic_cell_explorer/utils.hpp" 

void publishRays(const std::vector<RayInfo>& rays,
                 rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher) 
{
    
    visualization_msgs::msg::MarkerArray marker_array;

    for (size_t i = 0; i < rays.size(); ++i) {
        
        visualization_msgs::msg::Marker start_marker, end_marker;

        
        start_marker.header.frame_id = end_marker.header.frame_id = "world"; 
        start_marker.ns = end_marker.ns = "raycasting";
        start_marker.action = end_marker.action = visualization_msgs::msg::Marker::ADD;
        
        start_marker.type = end_marker.type = visualization_msgs::msg::Marker::SPHERE;

        // Set the scale for the points (size of the sphere)
        start_marker.scale.x = start_marker.scale.y = start_marker.scale.z = 0.1;  
        end_marker.scale.x = end_marker.scale.y = end_marker.scale.z = 0.1;        

    
        start_marker.color.a = 1.0;
        start_marker.color.r = 1.0; // Red for the start point
        start_marker.color.g = 0.0;
        start_marker.color.b = 0.0;

        end_marker.color.a = 1.0;
        if (rays[i].hit_unknown) {
            end_marker.color.r = 0.0;
            end_marker.color.g = 1.0;
            end_marker.color.b = 0.0; // Green for unknown space
        } else {
            end_marker.color.r = 0.0;
            end_marker.color.g = 1.0;
            end_marker.color.b = 1.0; // For free and occupied space
        }

        // Set unique IDs for the markers
        start_marker.id = i * 2;        // Even ID for start point
        end_marker.id = i * 2 + 1;      // Odd ID for end point

        // Set the positions for the start and end points
        start_marker.pose.position.x = rays[i].start.x();
        start_marker.pose.position.y = rays[i].start.y();
        start_marker.pose.position.z = rays[i].start.z();

        end_marker.pose.position.x = rays[i].end.x();
        end_marker.pose.position.y = rays[i].end.y();
        end_marker.pose.position.z = rays[i].end.z();

        marker_array.markers.push_back(start_marker);
        marker_array.markers.push_back(end_marker);
    }

    publisher->publish(marker_array);
}


sensor_msgs::msg::PointCloud2 convertOctomapToPointCloud2(const std::shared_ptr<octomap::OcTree>& octree) {

    sensor_msgs::msg::PointCloud2 cloud_msg;
    cloud_msg.header.frame_id = "world";  
    cloud_msg.header.stamp = rclcpp::Clock().now();
    sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
    modifier.setPointCloud2FieldsByString(1, "xyz");

    size_t point_count = 0;
    for (auto it = octree->begin_leafs(); it != octree->end_leafs(); ++it) {
        if (octree->isNodeOccupied(*it)) {
            
            if (point_count >= modifier.size()) {
                modifier.resize(point_count + 1);
            }
    
            const octomap::point3d& point = it.getCoordinate();

            sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
            sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
            sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");

            iter_x += point_count;
            iter_y += point_count;
            iter_z += point_count;

            *iter_x = point.x();
            *iter_y = point.y();
            *iter_z = point.z();

            ++point_count;
        }
    }
    cloud_msg.width = point_count;  
    cloud_msg.height = 1;  
    cloud_msg.is_dense = true; 

    return cloud_msg;
}

void printTransform(const Eigen::Isometry3d& transform) {
    // Extract the translation (position)
    Eigen::Vector3d translation = transform.translation();
    std::cout << "Translation (x, y, z): " 
              << translation.x() << ", " 
              << translation.y() << ", " 
              << translation.z() << std::endl;

    // Extract the rotation as a quaternion
    Eigen::Quaterniond rotation(transform.rotation());
    std::cout << "Rotation (quaternion): "
              << "w: " << rotation.w() << ", "
              << "x: " << rotation.x() << ", "
              << "y: " << rotation.y() << ", "
              << "z: " << rotation.z() << std::endl;

    // Optional: Print rotation as Euler angles if preferred
    Eigen::Vector3d euler_angles = transform.rotation().eulerAngles(0, 1, 2); // roll (X), pitch (Y), yaw (Z)
    std::cout << "Rotation (Euler angles): "
              << "roll: " << euler_angles.x() << ", "
              << "pitch: " << euler_angles.y() << ", "
              << "yaw: " << euler_angles.z() << std::endl;
}

#include <visualization_msgs/msg/marker.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <Eigen/Geometry>

void publish_fov_marker(const Eigen::Isometry3d& camera_pose, double fov_x, double fov_y, double range, std::shared_ptr<rclcpp::Node> node_) {
    // Create the ROS node (if not created already)
    
    static auto marker_pub = node_->create_publisher<visualization_msgs::msg::Marker>("camera_fov_marker", 10);

    // Create the marker
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "world";  // Change as per your setup
    marker.header.stamp = node_->now();
    marker.ns = "camera_fov";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose = tf2::toMsg(camera_pose);  // Camera pose in the map frame
    marker.scale.x = 0.02;  // Line thickness

    // Set color for the FOV lines
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0f;

    // Camera FOV calculations
    double half_fov_x = fov_x / 2.0;
    double half_fov_y = fov_y / 2.0;

    // Near plane points at a distance of 'range' from the camera
    Eigen::Vector3d camera_origin = camera_pose.translation();
    Eigen::Vector3d top_left(-range * tan(half_fov_x), range * tan(half_fov_y), range);
    Eigen::Vector3d top_right(range * tan(half_fov_x), range * tan(half_fov_y), range);
    Eigen::Vector3d bottom_left(-range * tan(half_fov_x), -range * tan(half_fov_y), range);
    Eigen::Vector3d bottom_right(range * tan(half_fov_x), -range * tan(half_fov_y), range);

    // Create FOV lines (camera origin to frustum corners)
    geometry_msgs::msg::Point p_origin, p1, p2, p3, p4;
    
    // Origin
    p_origin.x = p_origin.y = p_origin.z = 0.0;

    // Corner points of the frustum
    p1.x = top_left.x();
    p1.y = top_left.y();
    p1.z = top_left.z();

    p2.x = top_right.x();
    p2.y = top_right.y();
    p2.z = top_right.z();

    p3.x = bottom_left.x();
    p3.y = bottom_left.y();
    p3.z = bottom_left.z();

    p4.x = bottom_right.x();
    p4.y = bottom_right.y();
    p4.z = bottom_right.z();

    // Add lines from the camera origin to the FOV corners
    marker.points.push_back(p_origin); marker.points.push_back(p1);  // Origin -> Top left
    marker.points.push_back(p_origin); marker.points.push_back(p2);  // Origin -> Top right
    marker.points.push_back(p_origin); marker.points.push_back(p3);  // Origin -> Bottom left
    marker.points.push_back(p_origin); marker.points.push_back(p4);  // Origin -> Bottom right

    // Add lines to connect the FOV corners (for visualizing the frustum)
    marker.points.push_back(p1); marker.points.push_back(p2);  // Top left -> Top right
    marker.points.push_back(p3); marker.points.push_back(p4);  // Bottom left -> Bottom right
    marker.points.push_back(p1); marker.points.push_back(p3);  // Top left -> Bottom left
    marker.points.push_back(p2); marker.points.push_back(p4);  // Top right -> Bottom right

    // Publish the marker
    marker_pub->publish(marker);
}
