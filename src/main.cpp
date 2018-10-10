
/// c/c++
#include <iostream>
#include <cstdlib>
#include <ctime>
#include <algorithm>
#include <random>

///boost
#include <boost/thread/thread.hpp>

/// pcl
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_handlers.h>
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

/// icp
#include "./icp/icp.h"

int num_of_points = 100;
int num_of_tests = 1000;

int dim = 2;
float noise_sigma = 0.01;
float translation = 0.1;
float rotation = 0.1;


int main (int argc, char** argv)
{
    if (argc != 2)
    {
        std::cout << "\n\n\n[Arguments error]" << std::endl;
        std::cout << "Check arguments!" << std::endl;
        std::cout << "  ex) ./icp_demo [data_file_pat]\n\n\n";

        exit(1);
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr results_pcl_icp (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr results_layered_icp (new pcl::PointCloud<pcl::PointXYZ>);

    
    /// Load source point cloud data
  
    if (pcl::io::loadPCDFile<pcl::PointXYZ> (argv[1], *cloud_source) == -1) //* load the file
    {
      PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
      return (-1);
    }

    std::cout << "Loaded "
              << cloud_source->width * cloud_source->height
              << " data points from test_pcd.pcd with the following fields: "
              << std::endl;

    
    /// Generate target point cloud data
    Eigen::Affine3f transformation = Eigen::Affine3f::Identity();
    srand( (unsigned int)time(NULL) );
    float trans_x = ( (float)( rand() % 10000 ) ) / (1000.f);
    float trans_y = ( (float)( rand() % 10000 ) ) / (1000.f);

    transformation.translation() << trans_x, trans_x, 0.f;

    float theta = ( (float)( rand() % 100 ) ) / (100.f);
    transformation.rotate (Eigen::AngleAxisf (theta, Eigen::Vector3f::UnitZ()));

    pcl::transformPointCloud (*cloud_source, *cloud_target, transformation);

    auto rng = std::default_random_engine {};
    std::shuffle(cloud_target->begin(), cloud_target->end(), rng);

    cloud_target->erase(cloud_target->end() - 500, cloud_target->end());


    /// registration using pcl-icp        
    double icp_max_dist_corrs_prev_ = 100;
    double icp_max_num_iter_prev_ = 100;
    double icp_max_tf_diff_prev_ = 1e-6;
    double icp_max_eculi_dist_diff_prev_ = 0.01;

    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp_pcl;
    // Set the input source and target
    icp_pcl.setInputSource (cloud_source);
    icp_pcl.setInputTarget (cloud_target);    
    // Set the max correspondence distance (e.g., correspondences with higher distances will be ignored)
    icp_pcl.setMaxCorrespondenceDistance (icp_max_dist_corrs_prev_);
    // Set the maximum number of iterations (criterion 1)
    icp_pcl.setMaximumIterations (icp_max_num_iter_prev_);
    // Set the transformation epsilon (criterion 2)
    // "Maximum allowable difference between two consecutive transformations."
    icp_pcl.setTransformationEpsilon (icp_max_tf_diff_prev_);
    // Set the euclidean distance difference epsilon (criterion 3)
    // "Maximum allowed Euclidean error between two consecutive steps in the ICP loop."
    icp_pcl.setEuclideanFitnessEpsilon (icp_max_eculi_dist_diff_prev_);
    
    // Perform the alignment
    icp_pcl.align (*results_pcl_icp);
    Eigen::Matrix4f trans_pcl_icp = icp_pcl.getFinalTransformation ();
    std::cout << "cloud_target size: " << cloud_target->size() << std::endl;
     
    /// icp 2d
    LayeredICP icp;
    icp.SetSource(cloud_source);
    icp.SetTarget(cloud_target);
    icp.Align(*results_layered_icp);
    Eigen::Matrix4f trans_layered_icp = icp.GetFinalTransformation();
    std::cout << "main-1-trans_layered_icp: " << trans_layered_icp << std::endl;
    
    std::cout << "transformation: " << std::endl << transformation.matrix() << std::endl;
    std::cout << "trans_pcl_icp: " << std::endl << trans_pcl_icp << std::endl;
    std::cout << "trans_layered_icp: " << std::endl << trans_layered_icp << std::endl;

    std::cout << "cloud_target size: " << cloud_target->size() << std::endl;

    pcl::visualization::PCLVisualizer* viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    // Define R,G,B colors for the point cloud
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_handler_source (cloud_source, 20, 230, 20); // green
    // We add the point cloud to the viewer and pass the color handler
    viewer->addPointCloud (cloud_source, cloud_handler_source, "source");
    
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_handler_target (cloud_target, 230, 20, 20); // Red
    viewer->addPointCloud (cloud_target, cloud_handler_target, "target");

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_handler_pcl_icp (results_pcl_icp, 20, 20, 230); // blue
    viewer->addPointCloud (results_pcl_icp, cloud_handler_pcl_icp, "pcl_icp");

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_handler_layered_icp (results_layered_icp, 20, 230, 230); // cyan
    viewer->addPointCloud (results_layered_icp, cloud_handler_layered_icp, "layered_icp");
        
    
    viewer->addCoordinateSystem (1.0, "cloud", 0);
    viewer->setBackgroundColor(0.05, 0.05, 0.05, 0); // Setting background to a dark grey
    viewer->initCameraParameters ();

    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "source");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "target");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "pcl_icp");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "layered_icp");

    viewer->setPosition(800, 400); // Setting visualiser window position

    /// Display the visualiser until 'q' key is pressed
    while (!viewer->wasStopped ()) 
    { 
      viewer->spinOnce ();
    }
}

