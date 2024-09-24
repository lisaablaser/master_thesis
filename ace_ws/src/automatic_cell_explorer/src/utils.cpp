#include <visualization_msgs/msg/marker.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <Eigen/Geometry>

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
        start_marker.scale.x = start_marker.scale.y = start_marker.scale.z = 0.1;  
        end_marker.scale.x = end_marker.scale.y = end_marker.scale.z = 0.1;        

        start_marker.color.a = 1.0;
        start_marker.color.r = 1.0; 
        start_marker.color.g = 0.0;
        start_marker.color.b = 0.0;

        end_marker.color.a = 1.0;
        if (rays[i].hit_unknown) {
            end_marker.color.r = 0.0;
            end_marker.color.g = 1.0;
            end_marker.color.b = 0.0; 
        } else {
            end_marker.color.r = 0.0;
            end_marker.color.g = 1.0;
            end_marker.color.b = 1.0; 
        }

        start_marker.id = i * 2;        
        end_marker.id = i * 2 + 1;      

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
    Eigen::Vector3d translation = transform.translation();
    std::cout << "Translation (x, y, z): " 
              << translation.x() << ", " 
              << translation.y() << ", " 
              << translation.z() << std::endl;

    Eigen::Quaterniond rotation(transform.rotation());
    std::cout << "Rotation (quaternion): "
              << "w: " << rotation.w() << ", "
              << "x: " << rotation.x() << ", "
              << "y: " << rotation.y() << ", "
              << "z: " << rotation.z() << std::endl;
    Eigen::Vector3d euler_angles = transform.rotation().eulerAngles(0, 1, 2);
    std::cout << "Rotation (Euler angles): "
              << "roll: " << euler_angles.x() << ", "
              << "pitch: " << euler_angles.y() << ", "
              << "yaw: " << euler_angles.z() << std::endl;
}


void publish_fov_marker(std::shared_ptr<rclcpp::Node> node_, rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher, const Eigen::Isometry3d& camera_pose, double fov_x, double fov_y) {
    
    double range = 0.3;
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "world"; 
    marker.header.stamp = node_->now();
    marker.ns = "camera_fov";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose = tf2::toMsg(camera_pose);  
    marker.scale.x = 0.02;  // Line thickness

    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0f;

    double half_fov_x = fov_x / 2.0;
    double half_fov_y = fov_y / 2.0;

    Eigen::Vector3d top_left(range, -range * tan(half_fov_y), range * tan(half_fov_x));     
    Eigen::Vector3d top_right(range, range * tan(half_fov_y), range * tan(half_fov_x));     
    Eigen::Vector3d bottom_left(range, -range * tan(half_fov_y), -range * tan(half_fov_x)); 
    Eigen::Vector3d bottom_right(range, range * tan(half_fov_y), -range * tan(half_fov_x)); 

    geometry_msgs::msg::Point p_origin, p1, p2, p3, p4;
 
    p_origin.x = 0.0;
    p_origin.y = 0.0;
    p_origin.z = 0.0;

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

    marker.points.push_back(p_origin); marker.points.push_back(p1);  
    marker.points.push_back(p_origin); marker.points.push_back(p2);  
    marker.points.push_back(p_origin); marker.points.push_back(p3);  
    marker.points.push_back(p_origin); marker.points.push_back(p4);  

    marker.points.push_back(p1); marker.points.push_back(p2); 
    marker.points.push_back(p3); marker.points.push_back(p4); 
    marker.points.push_back(p1); marker.points.push_back(p3);  
    marker.points.push_back(p2); marker.points.push_back(p4); 

    publisher->publish(marker);
}
