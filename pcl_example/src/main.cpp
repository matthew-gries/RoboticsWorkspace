#include <filesystem>
#include <iostream>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>

const std::string PCD_FILE_PATH = (std::filesystem::path(__FILE__).parent_path().parent_path() / "assets" / "room_scan1.pcd").string();

void cloudViewerOneOff(pcl::visualization::PCLVisualizer& viewer) {
    std::cout << "Running one-off function...\n";
    viewer.setBackgroundColor(1.0, 1.0, 1.0);
    pcl::PointXYZ o;
    o.x = 1.0;
    o.y = 0;
    o.z = 0;
    viewer.addSphere(o, 0.25, "sphere", 0);
}

void cloudViewerLoopCallback(pcl::visualization::PCLVisualizer& viewer) {
    static unsigned count = 0;
    std::stringstream ss;
    ss << "Once per viewer loop: " << count++;
    viewer.removeShape("text", 0);
    viewer.addText(ss.str(), 200, 300, "text", 0);
}

int main() {

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    int success = pcl::PCDReader().read(PCD_FILE_PATH, *cloud);

    if (success != 0) {
        std::cerr << "Could not read PCD file! File path: " << PCD_FILE_PATH << std::endl;
        return -1; 
    }

    pcl::visualization::CloudViewer viewer("Cloud Viewer");

    viewer.showCloud(cloud);
    viewer.runOnVisualizationThreadOnce(cloudViewerOneOff);
    viewer.runOnVisualizationThread(cloudViewerLoopCallback);
    while (!viewer.wasStopped()) {
        // Do some sort of processing here
    }

    return 0;
}