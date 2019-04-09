#ifndef __PLANE_SEGMENTATION_H__
#define __PLANE_SEGMENTATION_H__

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <pcl_ros/point_cloud.h> 
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/common/geometry.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/octree/octree_search.h>
#include <pcl/surface/gp3.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <dynamic_reconfigure/server.h>
#include <landing_site_detection/LSDConfig.h>

#include <string>
#include <deque>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

typedef struct _ScoredPlane
{
    double depth_score;
    double surface_area;
    double rectangular_density;
} ScoredPlane;

class PlaneSegmentation 
{
public:
    PlaneSegmentation(void);
    ~PlaneSegmentation(void);
    void run(void);
private:
    void pointcloud_callback(const PointCloud::ConstPtr& msg);
    void dyn_rec_callback(landing_site_detection::LSDConfig &config, uint32_t level);
    PointCloud::Ptr remove_outliers(const PointCloud::Ptr& pointcloud);
    pcl::PointXYZ compute_centroid(const PointCloud::Ptr& pointcloud);
    double get_rectangular_area(const PointCloud::Ptr& pointcloud);
    double get_surface_area(const PointCloud::Ptr& pointcloud);
    double find_polygon_area(const PointCloud::Ptr& pointcloud, const pcl::PolygonMesh& polygons);
    PointCloud::Ptr extract_cloud(const PointCloud::Ptr& pointcloud, const pcl::PointIndices::Ptr& indices, bool remove);

    double get_depth_confidence_score(const PointCloud::Ptr pointcloud, double max_distance, double min_distance,
                                      double& min_depth_confidence_score, double& max_depth_confidence_score);
    double get_flatness_score(const PointCloud::Ptr& pointcloud, const pcl::ModelCoefficients::Ptr coefficients,
                              double& min_distance, double& max_distance);
    double get_steepness_score(const PointCloud::Ptr& pointcloud, double& min_score, double& max_score);
    double get_mean_depth(const PointCloud::Ptr pointcloud);

    PointCloud::Ptr downsample_organized(const PointCloud::ConstPtr& pointcloud, int scale);

    uint32_t colors[1000];
    std::deque< std::vector<pcl::PointXYZ>> centroid_buffer;

    int frame_count;
    ros::NodeHandle nh;
    ros::Subscriber pointcloud_sub;
    ros::Publisher plane_pointcloud_pub;
    ros::Publisher colored_plane_pub;
    ros::Publisher landing_site_pose_pub;

    dynamic_reconfigure::Server<landing_site_detection::LSDConfig> dyn_rec_server;

    bool publish_pointclouds;

    // parameters
    double voxel_filter_leaf_size;
    double octree_resolution;
    int octree_search_thresh;
    double octree_search_radius;
    double surface_area_thresh;
    double density_score_weight;
    double depth_score_weight;
    double surface_area_score_weight;

    pcl::VoxelGrid<pcl::PCLPointCloud2> voxel_filter;
    pcl::SACSegmentation<pcl::PointXYZ> plane_segmenter;
    std::vector<pcl::PointXYZ> search_points;
};
#endif
