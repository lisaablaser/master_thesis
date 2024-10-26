#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

sensor_msgs::msg::PointCloud2::SharedPtr filterPointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr& input_cloud) {
    // Convert ROS PointCloud2 to PCL format
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*input_cloud, *pcl_cloud);

    // Filter points within the desired bounds
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(pcl_cloud);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(-10.0, 10.0);  // Set x limits
    pass.filter(*pcl_cloud);

    pass.setFilterFieldName("y");
    pass.setFilterLimits(-10.0, 10.0);  // Set y limits
    pass.filter(*pcl_cloud);

    pass.setFilterFieldName("z");
    pass.setFilterLimits(0.0, 5.0);     // Set z limits
    pass.filter(*pcl_cloud);

    // Convert PCL format back to ROS PointCloud2
    sensor_msgs::msg::PointCloud2::SharedPtr filtered_cloud(new sensor_msgs::msg::PointCloud2);
    pcl::toROSMsg(*pcl_cloud, *filtered_cloud);
    filtered_cloud->header = input_cloud->header;

    return filtered_cloud;
}

//USage
//sensor_msgs::msg::PointCloud2::SharedPtr filtered_cloud = filterPointCloud(msg->cloud);