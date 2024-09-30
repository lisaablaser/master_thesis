#include <visualization_msgs/msg/marker.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <Eigen/Geometry>

#include <sensor_msgs/point_cloud2_iterator.hpp>
#include "automatic_cell_explorer/visualize.hpp" 


void publishRays(const std::vector<RayInfo>& rays,
                 MarkerAPublisher publisher) 
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
        if (rays[i].node_state == NodeState::Unknown) {
            end_marker.color.r = 0.0;
            end_marker.color.g = 1.0;
            end_marker.color.b = 0.0; 
        }
        if (rays[i].node_state == NodeState::Occupied) {
            end_marker.color.r = 1.0;
            end_marker.color.g = 0.0;
            end_marker.color.b = 0.0; 
        }
        if (rays[i].node_state == NodeState::Free) {
            end_marker.color.r = 0.0;
            end_marker.color.g = 0.0;
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


void visualizeNbvFov(const Nbv & nbv, double fov_x, double fov_y, MarkerPublisher publisher) {
    
    Eigen::Isometry3d camera_pose = nbv.pose;
    double range = 0.3;
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "world"; 
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


void visualizeNbvCandidatesPose(
    const NbvCandidates& nbv_candidates,
    MarkerAPublisher marker_pub)
{
    /*
        Visualize all nbv candidate poses as arrows
    */
    visualization_msgs::msg::MarkerArray marker_array;

    int id = 0;
    for (const auto& nbv : nbv_candidates.nbv_candidates) {
        // Create a marker to represent the NBV pose
        visualization_msgs::msg::Marker marker;

        // Set marker properties
        marker.header.frame_id = "world";
        //marker.header.stamp = node->get_clock()->now();
        marker.ns = "nbv_candidates";
        marker.id = id++;
        marker.type = visualization_msgs::msg::Marker::ARROW; // You can also use CUBE or AXIS

        // Set the marker scale (size of the arrow)
        marker.scale.x = 0.2;  // Arrow length
        marker.scale.y = 0.05; // Arrow width
        marker.scale.z = 0.05; // Arrow height

        // Set the color (red for arrows)
        marker.color.r = 1.0f;
        marker.color.g = 0.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0f;

        // Convert Eigen::Isometry3d to geometry_msgs::msg::Pose
        geometry_msgs::msg::Pose pose_msg = tf2::toMsg(nbv.pose);
        marker.pose = pose_msg;

        // Add marker to the array
        marker_array.markers.push_back(marker);
    }

    // Publish the marker array to visualize in RViz
    marker_pub->publish(marker_array);
}

void visualizeNbvRayView(
    const Nbv& nbv,
    MarkerAPublisher marker_pub)
{
    visualization_msgs::msg::MarkerArray marker_array;
    int marker_id = 0;  // Unique ID for each marker

    const RayView& ray_view = nbv.ray_view;

    for (const auto& ray : ray_view.rays) {
        // Create a marker for each ray
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "world";  // Ensure this matches RViz's fixed frame
        marker.ns = "ray_view";
        marker.id = marker_id++;  // Unique marker ID
        marker.type = visualization_msgs::msg::Marker::LINE_LIST;  // Lines to visualize rays

        // Set the scale of the line (width of the ray)
        marker.scale.x = 0.02;  // Line thickness

        // Set color based on whether the ray hit an unknown region
        if (ray.node_state == NodeState::Unknown) {
            marker.color.r = 1.0f;  // Red for unknown hit
            marker.color.g = 0.0f;
            marker.color.b = 0.0f;
        } else {
            marker.color.r = 0.0f;
            marker.color.g = 1.0f;  // Green for known hit
            marker.color.b = 0.0f;
        }
        marker.color.a = 1.0f;  // Fully opaque

        // Create two points for the start and end of the ray
        geometry_msgs::msg::Point start_point;
        start_point.x = ray.start.x();
        start_point.y = ray.start.y();
        start_point.z = ray.start.z();

        geometry_msgs::msg::Point end_point;
        end_point.x = ray.end.x();
        end_point.y = ray.end.y();
        end_point.z = ray.end.z();

        // Add the points to the marker
        marker.points.push_back(start_point);  // Add start point
        marker.points.push_back(end_point);    // Add end point

        // Add this marker to the marker array
        marker_array.markers.push_back(marker);
    }

    // Publish the marker array to visualize in RViz
    marker_pub->publish(marker_array);
}

void visualizeNbvCandidatesFOV(
    const NbvCandidates& nbv_candidates,
    MarkerAPublisher marker_pub)
{
    visualization_msgs::msg::MarkerArray marker_array;
    int marker_id = 0;  

    for (const auto& nbv : nbv_candidates.nbv_candidates) {
        
        visualization_msgs::msg::Marker fov_marker;
        fov_marker.header.frame_id = "world";
        fov_marker.ns = "camera_fov";
        fov_marker.id = marker_id++;
        fov_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
        fov_marker.action = visualization_msgs::msg::Marker::ADD;
        fov_marker.pose = tf2::toMsg(nbv.pose);
        fov_marker.scale.x = 0.02;  
        fov_marker.color.r = 0.0f;
        fov_marker.color.g = 1.0f;
        fov_marker.color.b = 0.0f;
        fov_marker.color.a = 1.0f;

        
        double range = 0.3;
        double fov_x = 64.0;  
        double fov_y = 36.0;  

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

        p1.x = top_left.x(); p1.y = top_left.y(); p1.z = top_left.z();
        p2.x = top_right.x(); p2.y = top_right.y(); p2.z = top_right.z();
        p3.x = bottom_left.x(); p3.y = bottom_left.y(); p3.z = bottom_left.z();
        p4.x = bottom_right.x(); p4.y = bottom_right.y(); p4.z = bottom_right.z();

      
        fov_marker.points.push_back(p_origin); fov_marker.points.push_back(p1);  
        fov_marker.points.push_back(p_origin); fov_marker.points.push_back(p2);  
        fov_marker.points.push_back(p_origin); fov_marker.points.push_back(p3);  
        fov_marker.points.push_back(p_origin); fov_marker.points.push_back(p4);  

        fov_marker.points.push_back(p1); fov_marker.points.push_back(p2); 
        fov_marker.points.push_back(p3); fov_marker.points.push_back(p4); 
        fov_marker.points.push_back(p1); fov_marker.points.push_back(p3);  
        fov_marker.points.push_back(p2); fov_marker.points.push_back(p4); 

        marker_array.markers.push_back(fov_marker);
    }

    marker_pub->publish(marker_array);
}
