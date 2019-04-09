#include <pointcloud_republish/pointcloud_republish_node.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "pointcloud_republish_node");
    PointCloudRepublishNode node;
    node.run();
    ros::shutdown();
    return 0;
}

PointCloudRepublishNode::PointCloudRepublishNode(void) : 
    nh_("~")
{
    std::string input_topic;
    std::string output_topic;
    float leaf_size;

    nh_.param<std::string>("input_topic", input_topic, "/camera/depth/points");
    nh_.param<std::string>("output_topic", output_topic, "/camera/depth/points_downsampled");
    nh_.param<float>("leaf_size", leaf_size, 0.1);

    ROS_INFO("Input: %s, output: %s", input_topic.c_str(), output_topic.c_str());

    sub_ = nh_.subscribe<PointCloud>(input_topic, 1, &PointCloudRepublishNode::pointcloud_cb, this);
    pub_ = nh_.advertise<PointCloud>(output_topic, 1);

    voxel_filter_.setLeafSize(leaf_size, leaf_size, leaf_size);
}

PointCloudRepublishNode::~PointCloudRepublishNode(void) {

}

void PointCloudRepublishNode::run(void) {
    ros::spin();
}

void PointCloudRepublishNode::pointcloud_cb(const PointCloud::ConstPtr& msg) {
    if (msg->points.size() == 0) {
        return;
    }

    // voxelize cloud
    PointCloud::Ptr cloud_filtered(new PointCloud);
    pcl::PCLPointCloud2::Ptr cloud2(new pcl::PCLPointCloud2);
    pcl::PCLPointCloud2::Ptr filtered_cloud2(new pcl::PCLPointCloud2);
    pcl::toPCLPointCloud2(*msg, *cloud2);
    voxel_filter_.setInputCloud(cloud2);
    voxel_filter_.filter(*filtered_cloud2);
    pcl::fromPCLPointCloud2(*filtered_cloud2, *cloud_filtered);

    // publish
    cloud_filtered->header.frame_id = msg->header.frame_id;
    cloud_filtered->header.stamp = msg->header.stamp;

    for (int i = 0; i < cloud_filtered->points.size(); i++) {
        pcl::PointXYZ pt = cloud_filtered->points[i];
        cloud_filtered->points[i].x = -1 * pt.x;
        cloud_filtered->points[i].z = -1 * pt.z;
    }
    pub_.publish(cloud_filtered);
}

