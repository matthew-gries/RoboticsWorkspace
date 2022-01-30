#include <filesystem>
#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/visualization/cloud_viewer.h>


const std::string PCD_FILE_PATH = (std::filesystem::path(__FILE__).parent_path().parent_path() / "assets" / "table_scene_lms400.pcd").string();

int main() {

    pcl::PCLPointCloud2::Ptr cloud_blob(new pcl::PCLPointCloud2), cloud_filtered_blob(new pcl::PCLPointCloud2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>), cloud_p(new pcl::PointCloud<pcl::PointXYZ>), cloud_f(new pcl::PointCloud<pcl::PointXYZ>);

    // Fill in cloud data
    pcl::PCDReader reader;
    reader.read(PCD_FILE_PATH, *cloud_blob);

    std::cerr << "PointCloud before filtering: " << cloud_blob->width * cloud_blob->height << " data points." << std::endl;

    // Create the filtering object: downsample the dataset using a leaf size of 1cm. This makes the segmentation loop faster
    // This filter basically makes a set of tiny 3D boxes in spaces and downsamples all points in each box to their centroid
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud(cloud_blob);
    sor.setLeafSize(0.01f, 0.01f, 0.01f);
    sor.filter(*cloud_filtered_blob);
  
    // Convert to the templated PointCloud
    pcl::fromPCLPointCloud2(*cloud_filtered_blob, *cloud_filtered);
  
    std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height << " data points." << std::endl;
  
    // Write the downsampled version to disk
    pcl::PCDWriter writer;
    writer.write<pcl::PointXYZ>("table_scene_lms400_downsampled.pcd", *cloud_filtered, false);
  
    // Set up the segmentation component
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(1000);
    seg.setDistanceThreshold(0.01);
  
    // Create the filtering object
    pcl::ExtractIndices<pcl::PointXYZ> extract;
  
    int i = 0, nr_points = (int)cloud_filtered->points.size();
    // While 30% of the original cloud is still there
    // When we start, cloud_filtered has the whole (downsampled) PointCloud
    while (cloud_filtered->size() > 0.3 * nr_points) {
      // Segment the largest planar component from the remaining cloud
      seg.setInputCloud(cloud_filtered);
      seg.segment (*inliers, *coefficients);

      if (inliers->indices.size() == 0) {
        std::cerr << "Could not esitmate a planar model for the given dataset." << std::endl;
        break;
      }

      extract.setInputCloud(cloud_filtered);
      extract.setIndices(inliers);
      extract.setNegative(false);
      extract.filter (*cloud_p);
      std::cerr << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points. " << std::endl;

      std::stringstream ss;
      ss << "table_scene_lms400_plane_" << i << ".pcd";
      writer.write<pcl::PointXYZ>(ss.str(), *cloud_p, false); 

      extract.setNegative(true);
      extract.filter(*cloud_f);
      cloud_filtered.swap(cloud_f);

      i++;
    }

    std::cerr << "Visualizing..." << std::endl;

    // Make a 3D RGB PointCloud to display from the inliers
    pcl::visualization::CloudViewer viewer("Planar Segmentation");
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudVisualized(new pcl::PointCloud<pcl::PointXYZRGB>);

    // Fill in cloud data
    cloudVisualized->width = cloud_filtered->width;
    cloudVisualized->height = cloud_filtered->height;
    cloudVisualized->points.resize(cloud_filtered->width * cloud_filtered->height);

    for (size_t i = 0; i < cloudVisualized->points.size(); i++) {
        cloudVisualized->points[i].x = cloud_filtered->points[i].x;
        cloudVisualized->points[i].y = cloud_filtered->points[i].y;
        cloudVisualized->points[i].z = cloud_filtered->points[i].z;
        if (std::find(inliers->indices.begin(), inliers->indices.end(), i) != std::end(inliers->indices)) {
            cloudVisualized->points[i].r = 0;
            cloudVisualized->points[i].g = 255;
            cloudVisualized->points[i].b = 0;
        } else {
            cloudVisualized->points[i].r = 0;
            cloudVisualized->points[i].g = 0;
            cloudVisualized->points[i].b = 0;
        }
    }

    viewer.showCloud(cloudVisualized);

    while (!viewer.wasStopped()) {}

    return 0;
}