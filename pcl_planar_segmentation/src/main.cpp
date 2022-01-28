#include <filesystem>
#include <iostream>
#include <algorithm>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/cloud_viewer.h>


const std::string PCD_FILE_PATH = (std::filesystem::path(__FILE__).parent_path().parent_path() / "assets" / "table_scene_lms400.pcd").string();

int main() {

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // Fill in cloud data
    cloud->width = 15;
    cloud->height = 1;
    cloud->points.resize(cloud->width * cloud->height);

    // Generate data
    for (auto& point : *cloud) {
        point.x = 1024 * rand() / (RAND_MAX + 1.0f);
        point.y = 1024 * rand() / (RAND_MAX + 1.0f);
        point.z = 1.0;
    }

    // Set some outliers
    (*cloud)[0].z = 2.0;
    (*cloud)[3].z = -2.0;
    (*cloud)[6].z = 4.0;

    std::cerr << "Point cloud data: " << cloud->size() << " points" << std::endl;
    for (const auto& point: *cloud) {
        std::cerr << "  " << point.x << " "
                          << point.y << " "
                          << point.z << std::endl; 
    }

    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    // Create segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // Optional
    seg.setOptimizeCoefficients(true);
    // Mandatory
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setModelType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.01);

    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);

    if (inliers->indices.size() == 0) {
        PCL_ERROR("Could not estimate a planar model for the given dataset.\n");
        return (-1);
    }

    std::cerr << "Model coefficients: " << coefficients->values[0] << " "
                                        << coefficients->values[1] << " "
                                        << coefficients->values[2] << " "
                                        << coefficients->values[3] << std::endl;

    std::cerr << "Model inliers: " << inliers->indices.size() << std::endl;
    for (const auto& idx: inliers->indices) {
        std::cerr << idx << "   " << cloud->points[idx].x << " "
                                  << cloud->points[idx].y << " "
                                  << cloud->points[idx].z << std::endl;
    }

    // Make a 3D RGB PointCloud to display from the inliers
    pcl::visualization::CloudViewer viewer("Planar Segmentation");
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudVisualized(new pcl::PointCloud<pcl::PointXYZRGB>);

    // Fill in cloud data
    cloudVisualized->width = 15;
    cloudVisualized->height = 1;
    cloudVisualized->points.resize(cloud->width * cloud->height);

    for (size_t i = 0; i < cloudVisualized->points.size(); i++) {
        cloudVisualized->points[i].x = cloud->points[i].x;
        cloudVisualized->points[i].y = cloud->points[i].y;
        cloudVisualized->points[i].z = cloud->points[i].z;
        if (std::find(inliers->indices.begin(), inliers->indices.end(), i) != std::end(inliers->indices)) {
            cloudVisualized->points[i].r = 0;
            cloudVisualized->points[i].g = 255;
            cloudVisualized->points[i].b = 0;
        } else {
            cloudVisualized->points[i].r = 255;
            cloudVisualized->points[i].g = 0;
            cloudVisualized->points[i].b = 0;
        }
    }

    viewer.showCloud(cloudVisualized);

    while (!viewer.wasStopped()) {}

    return 0;
}