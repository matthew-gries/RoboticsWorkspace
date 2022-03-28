#include <filesystem>
#include <iostream>
#include <vector>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/search/kdtree.h>


const std::string PCD_FILE_PATH = (std::filesystem::path(__FILE__).parent_path().parent_path() / "assets/").string();

int main(int argc, char** argv) {

    /**
     * @brief Read in the point cloud from the given file
     * 
     */

    if (argc != 2) {
        std::cerr << "Usage: ./PclNormal <path_to_pcd>" << std::endl;
        return 1;
    }

    std::string path = PCD_FILE_PATH + argv[1];

    std::cout << "Attempting to read from path: " << path << std::endl;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    int success = pcl::PCDReader().read(path, *cloud);

    if (success != 0) {
        std::cerr << "Could not read PCD file! File path: " << path << std::endl;
        return -1; 
    }

    /**
     * @brief Segment the point cloud into planes, should be able to find the cardboard
     * 
     */

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::copyPointCloud(*cloud, *cloud_filtered);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered_temp(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::copyPointCloud(*cloud, *cloud_filtered_temp);

    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    // Optional
    seg.setOptimizeCoefficients(true);
    // Mandatory
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.05);
    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);

    pcl::ExtractIndices<pcl::PointXYZRGB> extract;

    int i = 0, nr_points = (int)cloud->size();

    while (cloud_filtered->size() > 0.3 * nr_points) {

        seg.setInputCloud(cloud_filtered);
        seg.segment(*inliers, *coefficients);

        if (inliers->indices.size() == 0) {
            PCL_ERROR("Could not estimate a planar model for the given dataset.\n");
            return (-1);
        }

        extract.setInputCloud(cloud_filtered);
        extract.setIndices(inliers);
        extract.filter(*cloud_filtered_temp);

        cloud_filtered.swap(cloud_filtered_temp);
    }

    for (int i = 0; i < cloud_filtered->points.size(); i++) {
        cloud_filtered->points[i].r = 0;
        cloud_filtered->points[i].g = 255;
        cloud_filtered->points[i].b = 0;
    }

    /**
     * @brief Perform cluster extraction to get the cluster representing the cardboard
     * 
     */

    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
    tree->setInputCloud(cloud_filtered);
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
    ec.setClusterTolerance(0.02);
    ec.setMinClusterSize(10);
    ec.setMaxClusterSize(250000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud_filtered);
    ec.extract(cluster_indices);

    // Find the largest cluster
    int size = -1;
    size_t largest_idx = 0;
    for (size_t i = 0; i < cluster_indices.size(); i++) {
        if ((int)cluster_indices[i].indices.size() > size) {
            size = cluster_indices[i].indices.size();
            largest_idx = i;
        }
    }
    // std::cerr << largest_idx << std::endl;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cardboard_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    for (const auto& idx : cluster_indices[largest_idx].indices) {
        cardboard_cloud->push_back((*cloud_filtered)[idx]);
    }

    // /**
    //  * @brief Uncomment if you just want to visualize the pointclouds 
    //  * 
    //  */
    // pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    // viewer->setBackgroundColor(0, 0, 0);
    // viewer->addPointCloud<pcl::PointXYZRGB>(cloud, "all");
    // viewer->addPointCloud<pcl::PointXYZRGB>(cardboard_cloud, "plane");
    // viewer->addCoordinateSystem(1.0);
    // viewer->initCameraParameters();

    // while (!viewer->wasStopped()) {
    //     viewer->spinOnce(100);
    // }

    // return 0;

    // Create the normal estimation class, and pass the input dataset to it
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
    ne.setInputCloud(cardboard_cloud);

    // Create an empty kdtree representation, and pass it to the normal estimation object.
    // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr ne_tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
    ne.setSearchMethod(ne_tree);

    // Output datasets
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);

    // Use all neighbors in a sphere of radius 3cm
    ne.setRadiusSearch(0.03);

    // Compute the features
    ne.compute(*cloud_normals);

    // Visualize
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    viewer->addPointCloud<pcl::PointXYZRGB>(cloud, "all");
    viewer->addPointCloud<pcl::PointXYZRGB>(cardboard_cloud, "plane");
    viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal>(cardboard_cloud, cloud_normals, 10, 0.05, "normals");
    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();

    while (!viewer->wasStopped()) {
        viewer->spinOnce(100);
    }

    return 0;
}
