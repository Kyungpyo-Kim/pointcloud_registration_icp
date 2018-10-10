
#include "icp.h"


LayeredICP::LayeredICP()
:source_(new pcl::PointCloud<pcl::PointXYZ>),
target_(new pcl::PointCloud<pcl::PointXYZ>),
source2d_(new pcl::PointCloud<pcl::PointXY>),
target2d_(new pcl::PointCloud<pcl::PointXY>)
{std::cout << "icp--0-final_transformation_: " << final_transformation_ << std::endl;
    final_transformation_ = Eigen::Matrix4f::Identity();std::cout << "icp--1-final_transformation_: " << final_transformation_ << std::endl;
};


LayeredICP::~LayeredICP(){};


void LayeredICP::SetSource(pcl::PointCloud<pcl::PointXYZ>::Ptr source)
{ 
    source_->clear();
    pcl::copyPointCloud(*source, *source_);

    source2d_->clear();
    
    for (auto it_pc = source_->begin(); it_pc != source_->end(); it_pc++)
    {
        pcl::PointXY p2d;
        p2d.x = it_pc->x;
        p2d.y = it_pc->y;
        source2d_->push_back(p2d);
    }
}


void LayeredICP::SetTarget(pcl::PointCloud<pcl::PointXYZ>::Ptr target)
{
    target_->clear();
    pcl::copyPointCloud(*target, *target_);

    target2d_->clear();

    std::cout << "set target" << std::endl;
    
    for (auto it_pc = target_->begin(); it_pc != target_->end(); it_pc++)
    {
        pcl::PointXY p2d;
        p2d.x = it_pc->x;
        p2d.y = it_pc->y;
        target2d_->push_back(p2d);
    }

    std::cout << "target size: " << target2d_ ->size() << std::endl;
}


void LayeredICP::Align(pcl::PointCloud<pcl::PointXYZ>& result){
    
    std::cout << "icp-1-final_transformation_: " << final_transformation_ << std::endl;
    icp2d(source2d_, target2d_, final_transformation_);
    std::cout << "icp-2-final_transformation_: " << final_transformation_ << std::endl;

    result.clear();

    pcl::transformPointCloud(*source_, result, final_transformation_);
    std::cout << "icp-3-final_transformation_: " << final_transformation_ << std::endl;
}


Eigen::Matrix4f LayeredICP::GetFinalTransformation(void) { return final_transformation_; }


void LayeredICP::icp2d(
    pcl::PointCloud<pcl::PointXY>::Ptr source,
    pcl::PointCloud<pcl::PointXY>::Ptr target,
    Eigen::Matrix4f& init_pose_results)
{
    /// paramters
    int iteration = 10;
    float outlier_dist = 1.0;

    pcl::PointCloud<pcl::PointXY>::Ptr source_origin (new pcl::PointCloud<pcl::PointXY>);
    pcl::PointCloud<pcl::PointXY>::Ptr source_trans (new pcl::PointCloud<pcl::PointXY>);
    pcl::copyPointCloud(*source, *source_origin);
    pcl::copyPointCloud(*source, *source_trans);

    transformPointCloud2d (*source_trans, *source_trans, init_pose_results);

    for (size_t i = 0; i < iteration; ++i)
    {
        /// * find correspondance using KDTree based NN search
        pcl::search::Search<pcl::PointXY>* kdtree = new pcl::search::KdTree<pcl::PointXY> ();
        kdtree->setInputCloud(source);

        std::vector< std::vector<float> > dists;
        std::vector< std::vector<int> > indices;
        
        int no_of_neighbors = 1;

        kdtree->nearestKSearchT (*target, std::vector<int>(),no_of_neighbors,indices,dists);

        std::cout << std::endl;

        pcl::PointCloud<pcl::PointXY>::Ptr src (new pcl::PointCloud<pcl::PointXY>);
        pcl::PointCloud<pcl::PointXY>::Ptr trg (new pcl::PointCloud<pcl::PointXY>);
    
        for (size_t j = 0; j < indices.size(); ++j)
        {
            std::cout << indices[j].size() << "," ;
        }

        std::cout << std::endl;
        std::cout << "indices size: " << indices.size() << std::endl;
        std::cout << "source size: " << source->size() << std::endl;
        std::cout << "target size: " << target->size() << std::endl;


        /// * find optimized transformation using SVD


    }
}

void LayeredICP::transformPointCloud2d (
    const pcl::PointCloud<pcl::PointXY> cloud_in, 
    pcl::PointCloud<pcl::PointXY> &cloud_out,
    Eigen::Matrix4f transform)
{
    pcl::PointCloud<pcl::PointXY> cloud_in_copy;
    pcl::copyPointCloud(cloud_in, cloud_in_copy);

    cloud_out.clear();

    for (size_t i = 0; i < cloud_in_copy.size(); ++i)
    {
        pcl::PointXY p2d;
        p2d.x = transform(0,0) * cloud_in_copy[i].x + transform(0,1) * cloud_in_copy[i].y + transform(0,3);
        p2d.y = transform(1,0) * cloud_in_copy[i].x + transform(1,1) * cloud_in_copy[i].y + transform(1,3);

        cloud_out.push_back(p2d);
    }
}
