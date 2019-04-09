#include <landing_site_detection/landing_site_detection.h>
#include <stdlib.h> 
#include <cmath>
#include <assert.h>

int main(int argc, char **argv) {
    srand(100);
    ros::init(argc, argv, "plane_segmentation_node");
    PlaneSegmentation plane_segmenter;
    plane_segmenter.run();
    return 0;
}

PlaneSegmentation::PlaneSegmentation(void) : 
    nh("~")
{
    std::string input_topic;
    std::string output_topic;

    nh.param<std::string>("input_topic", input_topic, "/voxblox_node/surface_pointcloud");
    nh.param<std::string>("output_topic", output_topic, "landing_site");
    nh.param<bool>("publish_pointclouds", publish_pointclouds, true);

    pointcloud_sub = nh.subscribe<PointCloud>(input_topic, 1, &PlaneSegmentation::pointcloud_callback, this);
    landing_site_pose_pub = nh.advertise<geometry_msgs::PoseStamped>(output_topic, 1);
    if (publish_pointclouds) {
        plane_pointcloud_pub = nh.advertise<PointCloud>("landing_sites", 1);
        colored_plane_pub = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB>>("planes_colored", 1);
    }

    // initialize parameters
    voxel_filter_leaf_size = 0.1;
    octree_resolution = 0.1;
    octree_search_thresh = 40;
    octree_search_radius = 0.5;
    surface_area_thresh = 0.4;
    density_score_weight = 0.0;
    depth_score_weight = 1.0;
    surface_area_score_weight = 0.0;

    dyn_rec_server.setCallback(boost::bind(&PlaneSegmentation::dyn_rec_callback, this, _1, _2));

    // voxel filter settings
    voxel_filter.setLeafSize(voxel_filter_leaf_size, voxel_filter_leaf_size, voxel_filter_leaf_size);

    // plane segmentation settings
    plane_segmenter.setOptimizeCoefficients(true);
    plane_segmenter.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
    plane_segmenter.setAxis(Eigen::Vector3f(0.0f, 0.0f, 1.0f));
    plane_segmenter.setEpsAngle(0.17f); // 15 degrees in radians
    plane_segmenter.setMethodType(pcl::SAC_RANSAC);
    plane_segmenter.setMaxIterations(1000);
    plane_segmenter.setDistanceThreshold(0.1f);

    for (int i = 0; i < 1000; i++) {
        colors[i] = rand();
    }
}

PlaneSegmentation::~PlaneSegmentation(void) {

}

void PlaneSegmentation::run(void) {
    ros::spin();
}

void PlaneSegmentation::dyn_rec_callback(landing_site_detection::LSDConfig &config, uint32_t level) {
    voxel_filter_leaf_size = config.leaf_size;
    octree_resolution = config.octree_resolution;
    octree_search_thresh = config.octree_search_thresh;
    octree_search_radius = config.octree_search_radius;
    surface_area_thresh = config.surface_area_thresh;
    density_score_weight = config.density_score_weight;
    depth_score_weight = config.depth_score_weight;
    surface_area_score_weight = config.surface_area_score_weight;
    ROS_WARN("***************Dynamic reconfigure update*******************");
    ROS_WARN("Voxel filter leaf size: %lf", voxel_filter_leaf_size);
    ROS_WARN("Octree resolution: %lf", octree_resolution);
    ROS_WARN("Octree search thresh: %d", octree_search_thresh);
    ROS_WARN("Octree search radius: %lf", octree_search_radius);
    ROS_WARN("Surface area thresh: %lf", surface_area_thresh);
    ROS_WARN("Density score weight: %lf", density_score_weight);
    ROS_WARN("Depth score weight: %lf", depth_score_weight);
    ROS_WARN("Surface area score weight: %lf", surface_area_score_weight);
}

void PlaneSegmentation::pointcloud_callback(const PointCloud::ConstPtr& msg) 
{
    PointCloud::Ptr cloud_filtered(new PointCloud);
    if (msg->points.size() == 0) {
        return;
    }
    pcl::PCLPointCloud2::Ptr cloud2(new pcl::PCLPointCloud2), filtered_cloud2(new pcl::PCLPointCloud2);
    pcl::toPCLPointCloud2(*msg, *cloud2);

    voxel_filter.setInputCloud(cloud2);
    voxel_filter.filter(*filtered_cloud2);

    // convert to PointCloud
    pcl::fromPCLPointCloud2(*filtered_cloud2, *cloud_filtered);

    cloud_filtered->header.frame_id = msg->header.frame_id;
    pcl_conversions::toPCL(ros::Time::now(), cloud_filtered->header.stamp);

    if (cloud_filtered->points.size() == 0) {
        return;
    }
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices); 
    
    int num_points = (int)cloud_filtered->points.size();
    PointCloud::Ptr removed(new PointCloud);
    PointCloud::Ptr all_planar(new PointCloud);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr all_planar_colored(new pcl::PointCloud<pcl::PointXYZRGB>);

    std::vector<PointCloud::Ptr> extracted_planes;
    std::vector<ScoredPlane> plane_scores;
    std::vector<pcl::PointXYZ> search_points;
    int plane_count = 0;

    geometry_msgs::PoseStamped best_pose;
    double best_score = 0;

    // first, extract all the candidate 1m radius landing sites
    while (cloud_filtered->points.size() > 0) {

        // get plane model
        plane_segmenter.setInputCloud(cloud_filtered);
        plane_segmenter.segment(*inliers, *coefficients);
        if (inliers->indices.size() == 0) {
            //ROS_WARN("Could not find plane in this model");
            break;
        }

        PointCloud::Ptr plane_raw = extract_cloud(cloud_filtered, inliers, false);

        // extract points within standard deviation
        PointCloud::Ptr plane = remove_outliers(plane_raw);
        *all_planar += *plane;

        std::vector<bool> used_points(plane->points.size(), false);

        pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree(octree_resolution);
        octree.setInputCloud(plane);
        octree.addPointsFromInputCloud();

        for (unsigned int i = 0; i < plane->points.size(); i++) {
            if (used_points[i] == true) {
                continue;
            }
            std::vector<int> points_idx;
            std::vector<float> points_radius_sq_dist;
            if (octree.radiusSearch(plane->points.at(i), octree_search_radius, points_idx, points_radius_sq_dist) >= octree_search_thresh) {
                PointCloud::Ptr candidate(new PointCloud);
                for (int j = 0; j < points_idx.size(); j++) {
                    candidate->points.push_back(plane->points.at(points_idx[j]));
                }

                // only accept candidate if surface area is above threshold
                if (get_surface_area(candidate) >= surface_area_thresh) {
                    extracted_planes.push_back(candidate);
                    for (int j = 0; j < points_idx.size(); j++) {
                        used_points[points_idx[j]] = true;
                    }
                }
            }
        }
        removed = extract_cloud(cloud_filtered, inliers, true);
        cloud_filtered.swap(removed);
    }

    plane_count = 0;

    // scoring
    plane_scores.resize(extracted_planes.size());
    double max_depth = 0.0;
    double min_depth = 10000.0f;

    double max_surface_area = 0.0;
    double min_surface_area = 10000.0f;

    double max_density = 0.0;
    double min_density = 10000.0f;
    for (int i = 0; i < extracted_planes.size(); i++) {
        PointCloud::Ptr plane = extracted_planes[i];
        plane_scores[i].depth_score = compute_centroid(plane).z;
        plane_scores[i].surface_area = get_surface_area(plane);
        plane_scores[i].rectangular_density = plane->points.size() / get_rectangular_area(plane);

        if (plane_scores[i].depth_score > max_depth) {
            max_depth = plane_scores[i].depth_score;
        }

        if (plane_scores[i].surface_area > max_surface_area) {
            max_surface_area = plane_scores[i].surface_area;
        }

        if (plane_scores[i].rectangular_density > max_density) {
            max_density = plane_scores[i].rectangular_density;
        }

        if (plane_scores[i].depth_score < min_depth) {
            min_depth = plane_scores[i].depth_score;
        }

        if (plane_scores[i].surface_area < min_surface_area) {
            min_surface_area = plane_scores[i].surface_area;
        }

        if (plane_scores[i].rectangular_density < min_density) {
            min_density = plane_scores[i].rectangular_density;
        }
    }

    for (int i = 0; i < extracted_planes.size(); i++) {
        PointCloud::Ptr plane = extracted_planes.at(i);
        ScoredPlane scores = plane_scores.at(i);
        double depth_score = ((scores.depth_score - min_depth) / (max_depth - min_depth));
        double area_score = (scores.surface_area - min_surface_area) / (max_surface_area - min_surface_area);
        double density_score = (scores.rectangular_density - min_density) / (max_density - min_density);
        
        double total_score = depth_score_weight * depth_score + surface_area_score_weight * area_score + density_score_weight * density_score;
        
        ROS_INFO("----------------Plane %d-------------------", i);
        ROS_INFO("Rectangular Density: %lf", density_score);
        ROS_INFO("Surface Area: %lf", area_score);
        ROS_INFO("Height: %lf", depth_score);
        ROS_INFO("Total score: %lf", total_score);

        if (total_score > best_score) {
            best_score = total_score;
            pcl::PointXYZ centroid = compute_centroid(plane);
            best_pose.pose.position.x = centroid.x;
            best_pose.pose.position.y = centroid.y;
            best_pose.pose.position.z = centroid.z;
        }
        for (int j = 0; j < plane->points.size(); j++) {
            pcl::PointXYZRGB colored_point;
            pcl::PointXYZ point = plane->points.at(j);
            colored_point.x = point.x;
            colored_point.y = point.y;
            colored_point.z = point.z;

            colored_point.rgb = colors[plane_count]; 
            all_planar_colored->points.push_back(colored_point);
        }
        assert(all_planar->points.size() > 0);
        plane_count++;
        ROS_INFO("************Published**************");
    }

    // purge dead planes, decrement lifespans of existing ones
    if (all_planar->points.size() > 0 && publish_pointclouds) {
        ROS_INFO("--------------------------------------------------");
        ROS_INFO("               PUBlISHING POINTCLOUDS             ");
        ROS_INFO("--------------------------------------------------");
        all_planar_colored->header.frame_id = msg->header.frame_id;
        all_planar_colored->header.stamp = msg->header.stamp;
        colored_plane_pub.publish(all_planar_colored);

        all_planar->header.frame_id = msg->header.frame_id;
        all_planar->header.stamp = msg->header.stamp;
        plane_pointcloud_pub.publish(all_planar);
    }

    pcl_conversions::fromPCL(msg->header.stamp, best_pose.header.stamp);
    best_pose.header.frame_id = msg->header.frame_id;
    landing_site_pose_pub.publish(best_pose);
}

PointCloud::Ptr PlaneSegmentation::remove_outliers(const PointCloud::Ptr& pointcloud) {
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> remover;
    PointCloud::Ptr filtered(new PointCloud);
    remover.setInputCloud(pointcloud);
    remover.setMeanK(50);
    remover.setStddevMulThresh(1.0);
    remover.filter(*filtered);
    return filtered;
}

pcl::PointXYZ PlaneSegmentation::compute_centroid(const PointCloud::Ptr& pointcloud) {
    // calculate centroid of the plane
    pcl::CentroidPoint<pcl::PointXYZ> curr_centroid;
    for (int i = 0; i < pointcloud->points.size(); i++) {
        pcl::PointXYZ point = pointcloud->points.at(i);
        curr_centroid.add(point);
    }
    pcl::PointXYZ centroid_point;
    curr_centroid.get(centroid_point);
    return centroid_point;
}

double PlaneSegmentation::get_rectangular_area(const PointCloud::Ptr& pointcloud) {
    // Find (min_x, min_y), (max_x, max_y) to get a rectangular area of the pointcloud
    auto max_x_iter = std::max_element(pointcloud->points.begin(), pointcloud->points.end(), 
                                       [](pcl::PointXYZ a, pcl::PointXYZ b) -> bool
                                       {
                                            return a.x < b.x;
                                       });
    auto max_y_iter = std::max_element(pointcloud->points.begin(), pointcloud->points.end(), 
                                       [](pcl::PointXYZ a, pcl::PointXYZ b) -> bool
                                       {
                                            return a.y < b.y;
                                       });
    auto min_x_iter = std::min_element(pointcloud->points.begin(), pointcloud->points.end(), 
                                       [](pcl::PointXYZ a, pcl::PointXYZ b) -> bool
                                       {
                                            return a.x < b.x;
                                       });
    auto min_y_iter = std::min_element(pointcloud->points.begin(), pointcloud->points.end(), 
                                       [](pcl::PointXYZ a, pcl::PointXYZ b) -> bool
                                       {
                                            return a.y < b.y;
                                       });
 
    return ((*max_x_iter).x - (*min_x_iter).x) * ((*max_y_iter).y - (*min_y_iter).y);
}

double PlaneSegmentation::get_surface_area(const PointCloud::Ptr& pointcloud) {
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr kd_tree(new pcl::search::KdTree<pcl::PointXYZ>);

    // First, estimate normals
    kd_tree->setInputCloud(pointcloud);
    normal_estimator.setInputCloud(pointcloud);
    normal_estimator.setSearchMethod(kd_tree);
    normal_estimator.setKSearch(20);
    normal_estimator.compute(*normals);

    // concatenate XYZ and normal fields
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
    pcl::concatenateFields(*pointcloud, *normals, *cloud_with_normals);

    // initialize search tree and objects for greedy triangulation
    pcl::search::KdTree<pcl::PointNormal>::Ptr normal_kd_tree(new pcl::search::KdTree<pcl::PointNormal>);
    normal_kd_tree->setInputCloud(cloud_with_normals);
    pcl::GreedyProjectionTriangulation<pcl::PointNormal> greedy_projection;
    pcl::PolygonMesh triangles;

    // greedy projection parameters
    greedy_projection.setSearchRadius(0.25);
    greedy_projection.setMu(2.5);
    greedy_projection.setMaximumNearestNeighbors(100);
    greedy_projection.setMaximumSurfaceAngle(M_PI / 4);
    greedy_projection.setMinimumAngle(M_PI/18);
    greedy_projection.setMaximumAngle(2 * M_PI / 3);
    greedy_projection.setNormalConsistency(false);

    greedy_projection.setInputCloud(cloud_with_normals);
    greedy_projection.setSearchMethod(normal_kd_tree);
    greedy_projection.reconstruct(triangles);

    // convert mesh to pointcloud

    // publish mesh_pub for debug
    /*
    mesh_cloud->header.frame_id = "guidance";
    pcl_conversions::toPCL(ros::Time::now(), mesh_cloud->header.stamp);
    mesh_pub.publish(mesh_cloud);
    */
    return find_polygon_area(pointcloud, triangles);
 }

double PlaneSegmentation::find_polygon_area(const PointCloud::Ptr& cloud, const pcl::PolygonMesh& mesh) {
   // then calculate the area of the mesh cloud
    int index1, index2, index3;
    double x1, x2, x3, y1, y2, y3, z1, z2, z3;
    double a, b, c, q;
    double area = 0;
    for (int i = 0; i < mesh.polygons.size(); ++i) {
        index1 = mesh.polygons[i].vertices[0];
        index2 = mesh.polygons[i].vertices[1];
        index3 = mesh.polygons[i].vertices[2];
        
        x1 = cloud->points[index1].x;
        y1 = cloud->points[index1].y;
        z1 = cloud->points[index1].z;

        x2 = cloud->points[index2].x;
        y2 = cloud->points[index2].y;
        z2 = cloud->points[index2].z;

        x3 = cloud->points[index3].x;
        y3 = cloud->points[index3].y;
        z3 = cloud->points[index3].z;

        // heron's formula
        a=sqrt(std::pow((x1-x2),2)+std::pow((y1-y2),2)+std::pow((z1-z2),2));
        b=sqrt(std::pow((x1-x3),2)+std::pow((y1-y3),2)+std::pow((z1-z3),2));
        c=sqrt(std::pow((x3-x2),2)+std::pow((y3-y2),2)+std::pow((z3-z2),2));
        q=(a+b+c)/2;

        area=area+std::sqrt(q*(q-a)*(q-b)*(q-c));
    }
    return area;
}

PointCloud::Ptr PlaneSegmentation::extract_cloud(const PointCloud::Ptr& pointcloud,
                                                 const pcl::PointIndices::Ptr& indices,
                                                 bool remove)
{
    PointCloud::Ptr return_cloud(new PointCloud);
    pcl::ExtractIndices<pcl::PointXYZ> extracter;

    extracter.setInputCloud(pointcloud);
    extracter.setIndices(indices);
    extracter.setNegative(remove);
    extracter.filter(*return_cloud);
    return return_cloud;
}

double PlaneSegmentation::get_depth_confidence_score(const PointCloud::Ptr pointcloud, double max_distance, double min_distance,
                                                     double& min_depth_confidence_score, double& max_depth_confidence_score) {

    double average_score = 0.0;
    int count = 0;
    for (int i = 0; i < pointcloud->points.size(); i++) {
        pcl::PointXYZ point = pointcloud->points.at(i);
        if (!std::isnan(point.z)) {
            double score = 1 - ((std::pow(point.z, 2) - std::pow(min_distance, 2)) / (std::pow(max_distance, 2)));
            average_score += score;
            count++;
            if (score < min_depth_confidence_score) {
                min_depth_confidence_score = score;
            }
            if (score > max_depth_confidence_score) {
                max_depth_confidence_score = score;
            }
        }
    }
    return average_score / count; 
}

double PlaneSegmentation::get_mean_depth(const PointCloud::Ptr pointcloud) {
    double avg_depth = 0.0;
    int count = 0;
    for (int i = 0; i < pointcloud->points.size(); i++) {
        pcl::PointXYZ point = pointcloud->points.at(i);
        if (!std::isnan(point.z)) {
            avg_depth += point.z;
            count++;
        }
    }
    return avg_depth / count;
}

double PlaneSegmentation::get_flatness_score(const PointCloud::Ptr& pointcloud, const pcl::ModelCoefficients::Ptr coefficients,
                                             double& min_distance, double& max_distance) {
    double mean_distance = 0;
    for (int i = 0; i < pointcloud->points.size(); i++) {
        // get point
        pcl::PointXYZ point = pointcloud->points.at(i);

        // compute distance
        double c1 = coefficients->values[0];
        double c2 = coefficients->values[1];
        double c3 = coefficients->values[2];
        double c4 = coefficients->values[3];

        double d = pcl::pointToPlaneDistance<pcl::PointXYZ>(point, c1, c2, c3, c4); // in meters
        //std::cout << point << std::endl;
        //ROS_INFO("Distance to plane: %lf", d);
        
        if (d < min_distance) {
            min_distance = d;
        }

        if (d > max_distance) {
            max_distance = d;
        }

        mean_distance += d;
    }
    mean_distance /= pointcloud->points.size();
    return mean_distance;
}

// TODO not really working? Figure out why later
double PlaneSegmentation::get_steepness_score(const PointCloud::Ptr& pointcloud, double& min_score, double& max_score) {
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    ne.setRadiusSearch(0.5);
    ne.setSearchMethod(tree);
    ne.setInputCloud(pointcloud);
    ne.setViewPoint(0, 0, 0);

    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>());
    ne.compute(*normals);
       
    double avg_steepness_score = 0;
    double avg_theta = 0;
    int count = 0;
    for (int i = 0; i < normals->points.size(); i++) {
        double z_component = normals->points.at(i).normal[2];
        if (!std::isnan(z_component)) {
            if (z_component < 0) {
                z_component *= -1;
            }
            // use curvature?
            double theta = acos(z_component);
            double curr_score = exp(-1 * std::pow(theta, 2) / (2 * std::pow(10, 2)));
            if (curr_score < min_score) {
                min_score = curr_score;
            }
            if (curr_score > max_score) {
                max_score = curr_score;
            }
            avg_theta += theta;
            avg_steepness_score += curr_score;
            count++;
        }
    }
    return avg_steepness_score / count;
}
