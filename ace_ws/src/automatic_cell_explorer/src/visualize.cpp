#include <visualization_msgs/msg/marker.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <Eigen/Geometry>

#include <sensor_msgs/point_cloud2_iterator.hpp>
#include "automatic_cell_explorer/constants.hpp"
#include "automatic_cell_explorer/visualize.hpp" 



void visualizeRayDots(const Nbv & nbv,
                 MarkerAPublisher publisher) 
{
    std::vector<RayInfo> rays = nbv.ray_view.rays;
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


void visualizeNbvFov(const Nbv & nbv, MarkerPublisher publisher) {
    
    Eigen::Isometry3d camera_pose = nbv.pose;
    double range = 0.3;
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "world"; 
    marker.ns = "camera_fov";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose = tf2::toMsg(camera_pose);  
    marker.scale.x = 0.02; 

    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0f;


    double half_fov_h = FOV_H * M_PI / 360.0;
    double half_fov_v = FOV_V * M_PI / 360.0;
    
    Eigen::Vector3d top_left(range, -range * tan(half_fov_h), range * tan(half_fov_v));     
    Eigen::Vector3d top_right(range, range * tan(half_fov_h), range * tan(half_fov_v));     
    Eigen::Vector3d bottom_left(range, -range * tan(half_fov_h), -range * tan(half_fov_v)); 
    Eigen::Vector3d bottom_right(range, range * tan(half_fov_h), -range * tan(half_fov_v)); 

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
    for (const auto& nbv : nbv_candidates) {
        visualization_msgs::msg::Marker marker;

        marker.header.frame_id = "world";
        marker.ns = "nbv_candidates";
        marker.id = id++;
        marker.type = visualization_msgs::msg::Marker::ARROW; 

        marker.scale.x = 0.2; 
        marker.scale.y = 0.05; 
        marker.scale.z = 0.05; 


        marker.color.r = 1.0f;
        marker.color.g = 0.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0f;

        geometry_msgs::msg::Pose pose_msg = tf2::toMsg(nbv.pose);
        marker.pose = pose_msg;

        marker_array.markers.push_back(marker);
    }

    marker_pub->publish(marker_array);
}

void visualizeNbvRayView(
    const Nbv& nbv,
    MarkerAPublisher marker_pub)
{
    visualization_msgs::msg::MarkerArray marker_array;
    int marker_id = 0; 

    const RayView& ray_view = nbv.ray_view;

    for (const auto& ray : ray_view.rays) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "world";  
        marker.ns = "ray_view";
        marker.id = marker_id++;  
        marker.type = visualization_msgs::msg::Marker::LINE_LIST; 

        marker.scale.x = 0.02; 
      
        if (ray.node_state == NodeState::Unknown) {
            marker.color.r = 0.0f; 
            marker.color.g = 1.0f;
            marker.color.b = 0.0f;
        } else {
            marker.color.r = 0.0f;
            marker.color.g = 1.0f;  
            marker.color.b = 0.0f;
        }
        marker.color.a = 1.0f;  

        geometry_msgs::msg::Point start_point;
        start_point.x = ray.start.x();
        start_point.y = ray.start.y();
        start_point.z = ray.start.z();

        geometry_msgs::msg::Point end_point;
        end_point.x = ray.end.x();
        end_point.y = ray.end.y();
        end_point.z = ray.end.z();

        marker.points.push_back(start_point);  
        marker.points.push_back(end_point);   

        
        marker_array.markers.push_back(marker);
    }

    marker_pub->publish(marker_array);
}

void visualizeNbvCandidatesFOV(
    const NbvCandidates& nbv_candidates,
    MarkerAPublisher marker_pub)
{
    visualization_msgs::msg::MarkerArray marker_array;
    int marker_id = 0;  

    for (const auto& nbv : nbv_candidates) {
        
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

        double half_fov_h = FOV_H * M_PI / 360.0;
        double half_fov_v = FOV_V * M_PI / 360.0;
        
        Eigen::Vector3d top_left(range, -range * tan(half_fov_h), range * tan(half_fov_v));     
        Eigen::Vector3d top_right(range, range * tan(half_fov_h), range * tan(half_fov_v));     
        Eigen::Vector3d bottom_left(range, -range * tan(half_fov_h), -range * tan(half_fov_v)); 
        Eigen::Vector3d bottom_right(range, range * tan(half_fov_h), -range * tan(half_fov_v)); 

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


void visualizeClusters(const std::vector<Cluster>& clusters, MarkerAPublisher marker_pub) {
    visualization_msgs::msg::MarkerArray marker_array;
    int id = 0;

    // Predefined color list for clusters
    std::vector<std::tuple<float, float, float>> color_list = {
        {1.0f, 0.0f, 0.0f},  // Red
        {0.0f, 1.0f, 0.0f},  // Green
        {0.0f, 0.0f, 1.0f},  // Blue
        {1.0f, 1.0f, 0.0f},  // Yellow
        {1.0f, 0.0f, 1.0f},  // Magenta
        {0.0f, 1.0f, 1.0f},  // Cyan
        {0.5f, 0.0f, 0.5f},  // Purple
        {0.5f, 0.5f, 0.5f}   // Gray
    };

    size_t num_colors = color_list.size();
    
    for (size_t cluster_idx = 0; cluster_idx < clusters.size(); ++cluster_idx) {
        const auto& cluster = clusters[cluster_idx];

        // Select a color for this cluster from the list
        auto [r, g, b] = color_list[cluster_idx % num_colors];

        // Visualize cluster points as cubes (voxel size 0.1)
        visualization_msgs::msg::Marker cube_marker;
        cube_marker.header.frame_id = "world";
        cube_marker.header.stamp = rclcpp::Clock().now();
        cube_marker.ns = "clusters";
        cube_marker.id = id++;
        cube_marker.type = visualization_msgs::msg::Marker::CUBE_LIST;
        cube_marker.action = visualization_msgs::msg::Marker::ADD;
        cube_marker.scale.x = 0.1;  // Cube size (voxel size)
        cube_marker.scale.y = 0.1;
        cube_marker.scale.z = 0.1;

        // Set the color for this cluster
        cube_marker.color.r = r;
        cube_marker.color.g = g;
        cube_marker.color.b = b;
        cube_marker.color.a = 1.0;

        for (const auto& point : cluster.points) {
            geometry_msgs::msg::Point p;
            p.x = point.x();
            p.y = point.y();
            p.z = point.z();
            cube_marker.points.push_back(p);
        }

        marker_array.markers.push_back(cube_marker);

        // Visualize frontiers as dots (smaller spheres)
        visualization_msgs::msg::Marker frontier_marker;
        frontier_marker.header.frame_id = "world";
        frontier_marker.header.stamp = rclcpp::Clock().now();
        frontier_marker.ns = "frontiers";
        frontier_marker.id = id++;
        frontier_marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
        frontier_marker.action = visualization_msgs::msg::Marker::ADD;
        frontier_marker.scale.x = 0.05;  // Dot size for frontiers
        frontier_marker.scale.y = 0.05;
        frontier_marker.scale.z = 0.05;

        // Use the same color for the frontiers as the cluster
        frontier_marker.color.r = r;
        frontier_marker.color.g = g;
        frontier_marker.color.b = b;
        frontier_marker.color.a = 1.0;

        for (const auto& frontier : cluster.frontiers) {
            geometry_msgs::msg::Point f;
            f.x = frontier.x();
            f.y = frontier.y();
            f.z = frontier.z();
            frontier_marker.points.push_back(f);
        }

        marker_array.markers.push_back(frontier_marker);

    }

    marker_pub->publish(marker_array);
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