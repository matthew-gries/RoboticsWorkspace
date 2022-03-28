#include <filesystem>
#include <iostream>
#include <algorithm>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>


const std::string PCD_FILE_PATH = (std::filesystem::path(__FILE__).parent_path().parent_path() / "assets" / "test2.pcd").string();

int main() {

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PCLPointCloud2::Ptr cloudBlob(new pcl::PCLPointCloud2);

    pcl::PCDReader reader;
    reader.read(PCD_FILE_PATH, *cloudBlob);
    pcl::fromPCLPointCloud2(*cloudBlob, *cloud);

    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    // Create segmentation object
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    // Optional
    seg.setOptimizeCoefficients(true);
    // Mandatory
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setModelType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.05);

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
        cloud->points[idx].r = 0;
        cloud->points[idx].g = 255;
        cloud->points[idx].b = 0;
    }

    // Visualize
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
    viewer->addPointCloud<pcl::PointXYZRGB>(cloud, rgb, "sample cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();

    while (!viewer->wasStopped()) {
        viewer->spinOnce(100);
    }

    return 0;
}