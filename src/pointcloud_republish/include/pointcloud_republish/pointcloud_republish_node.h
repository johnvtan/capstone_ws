#ifndef __POINTCLOUD_REPUBLISH_NODE__
#define __POINTCLOUD_REPUBLISH_NODE__

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

class PointCloudRepublishNode 
{
public:
    PointCloudRepublishNode(void);
    ~PointCloudRepublishNode(void);
    void run(void);
private:
    void pointcloud_cb(const PointCloud::ConstPtr& msg);
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Publisher pub_;
    pcl::VoxelGrid<pcl::PCLPointCloud2> voxel_filter_;
};

#endif

