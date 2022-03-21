#include <filesystem>
#include <iostream>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>


const std::string PCD_FILE_PATH = (std::filesystem::path(__FILE__).parent_path().parent_path() / "assets" / "test.pcd").string();

int main() {

    std::cout << "Attempting to read from path: " << PCD_FILE_PATH << std::endl;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    int success = pcl::PCDReader().read(PCD_FILE_PATH, *cloud);

    if (success != 0) {
        std::cerr << "Could not read PCD file! File path: " << PCD_FILE_PATH << std::endl;
        return -1; 
    }

    // Create the normal estimation class, and pass the input dataset to it
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud (cloud);

    // Create an empty kdtree representation, and pass it to the normal estimation object.
    // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ> ());
    ne.setSearchMethod (tree);

    // Output datasets
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);

    // Use all neighbors in a sphere of radius 3cm
    ne.setRadiusSearch (0.03);

    // Compute the features
    ne.compute (*cloud_normals);

    // TODO visualize cloud_normals and cloud together to see normals

    // pcl::visualization::CloudViewer viewer("Cloud Viewer");

    // viewer.showCloud(cloud);
    // viewer.runOnVisualizationThreadOnce(cloudViewerOneOff);
    // viewer.runOnVisualizationThread(cloudViewerLoopCallback);
    // while (!viewer.wasStopped()) {
    //     // Do some sort of processing here
    // }

    return 0;
}
