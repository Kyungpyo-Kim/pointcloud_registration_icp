#include <iostream>
#include <vector>

/// pcl
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/common/distances.h>
#include <pcl/common/io.h>
#include <pcl/search/pcl_search.h>



class LayeredICP
{
public:
    LayeredICP();
    ~LayeredICP();

public:
    void SetSource(pcl::PointCloud<pcl::PointXYZ>::Ptr source);
    void SetTarget(pcl::PointCloud<pcl::PointXYZ>::Ptr target);
    void Align(pcl::PointCloud<pcl::PointXYZ>& result);

public:
    Eigen::Matrix4f GetFinalTransformation(void);

private:
    void icp2d(
        pcl::PointCloud<pcl::PointXY>::Ptr source,
        pcl::PointCloud<pcl::PointXY>::Ptr target,
        Eigen::Matrix4f& init_pose_results);

    void transformPointCloud2d (
        const pcl::PointCloud<pcl::PointXY> cloud_in, 
        pcl::PointCloud<pcl::PointXY> &cloud_out,
        Eigen::Matrix4f transform);

private:
    pcl::PointCloud<pcl::PointXYZ>::Ptr source_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr target_;
    pcl::PointCloud<pcl::PointXY>::Ptr source2d_;
    pcl::PointCloud<pcl::PointXY>::Ptr target2d_;

    Eigen::Matrix4f final_transformation_;
};
