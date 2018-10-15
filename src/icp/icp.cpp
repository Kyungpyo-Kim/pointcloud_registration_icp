
#include "icp.h"


LayeredICP::LayeredICP()
:source_(new pcl::PointCloud<pcl::PointXYZ>),
target_(new pcl::PointCloud<pcl::PointXYZ>),
source2d_(new pcl::PointCloud<pcl::PointXY>),
target2d_(new pcl::PointCloud<pcl::PointXY>)
{
    final_transformation_ = Eigen::Matrix4f::Identity();
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
    
    float outlier_dist = 100.0;
    float convergence_criterion_transformation_epsilon_translation = 1e-5;
    float convergence_criterion_transformation_epsilon_rotation = 1e-8;
    float convergence_criterion_euclidean_distance_difference_epsilon = 0.1;
    float convergence_criterion_euclidean_distance_error = 0.02;
    bool convergence;
    int iteration = 100;

    icp2d(source2d_,
        target2d_,
        outlier_dist,
        convergence_criterion_transformation_epsilon_translation,
        convergence_criterion_transformation_epsilon_rotation,
        convergence_criterion_euclidean_distance_difference_epsilon,
        convergence_criterion_euclidean_distance_error,
        final_transformation_,
        convergence,
        iteration);

    std::cout << "convergence: " << convergence << std::endl;
    std::cout << "iteration: " << iteration << std::endl;
    std::cout << "convergence_criterion_eucliean_distance_error: " << convergence_criterion_euclidean_distance_error << std::endl;
    std::cout << "icp-2-final_transformation_: " << std::endl << final_transformation_ << std::endl;

    result.clear();

    pcl::transformPointCloud(*source_, result, final_transformation_);
    std::cout << "icp-3-final_transformation_: " << std::endl << final_transformation_ << std::endl;
}


Eigen::Matrix4f LayeredICP::GetFinalTransformation(void) { return final_transformation_; }


void LayeredICP::icp2d(
    const pcl::PointCloud<pcl::PointXY>::Ptr source,
    const pcl::PointCloud<pcl::PointXY>::Ptr target,
    const float outlier_rejection_dist,
    const float convergence_criterion_transformation_epsilon_translation,
    const float convergence_criterion_transformation_epsilon_rotation,
    const float convergence_criterion_euclidean_distance_difference_epsilon,
    float & convergence_criterion_euclidean_distance_error,
    Eigen::Matrix4f& init_pose_results,
    bool & convergence,
    int & iteration)
{
    /// paramters
    // int iteration = 50;
    // float outlier_dist = 100.0;
    // float convergence_criterion_transformation_epsilon_translation = 1e-5;
    // float convergence_criterion_transformation_epsilon_rotation = 1e-8;
    // float convergence_criterion_eucliean_distance_difference_epsilon = 0.1;
    // float convergence_criterion_eucliean_distance_error = 0.1;

    // enum methods {
    //     SVD = 0,
    //     DQ,
    //     LM
    // };
    // methods method = 0;

    convergence = false;

    pcl::PointCloud<pcl::PointXY>::Ptr source_trans (new pcl::PointCloud<pcl::PointXY>);
        
    Eigen::Matrix4f& out_transformation = init_pose_results;
    transformPointCloud2d (*source, *source_trans, out_transformation);
    
    /// * find correspondance using KDTree based NN search
    pcl::search::Search<pcl::PointXY>* kdtree = new pcl::search::KdTree<pcl::PointXY> ();
    kdtree->setInputCloud(target);

    static Eigen::Matrix4f transformation_prev = init_pose_results;
    static float prev_rmse = 0;

    for (size_t i = 0; i < iteration; ++i)
    {
        /// * find correspondance using KDTree based NN search
        std::vector< std::vector<float> > dists;
        std::vector< std::vector<int> > indices;
        
        int no_of_neighbors = 1;

        kdtree->nearestKSearchT (*source_trans, std::vector<int>(),no_of_neighbors,indices,dists);

        pcl::PointCloud<pcl::PointXY>::Ptr src (new pcl::PointCloud<pcl::PointXY>);
        pcl::PointCloud<pcl::PointXY>::Ptr trg (new pcl::PointCloud<pcl::PointXY>);
    
        for (size_t j = 0; j < indices.size(); ++j)
        {
            if (indices[j].size() != 1)
            {
                std::cerr << "\n\n[Error! Fail to fine correspondance]" << std::endl;
                return;
            }
            /// * reject outliers
            if (outlier_rejection_dist*outlier_rejection_dist > dists[j][0])
            {
                trg->push_back(target->at(indices[j][0]));
                src->push_back(source_trans->at(j));
            }            
        }

        if (trg->size() < 10 || src->size() < 10)
        {
            std::cerr << "\n\n[Warning! No correspondance, release the parameters!]" << std::endl;
            std::cerr << "indices size: " << indices.size() << std::endl;
            std::cerr << "trg size: " << trg->size() << std::endl;
            std::cerr << "src size: " << src->size() << std::endl;
            return;
        }

        /// * find optimized transformation using SVD
        Eigen::Matrix4f transformation_est;
        estimateTransformationSVD(*src, *trg, transformation_est);
        out_transformation = transformation_est * out_transformation;

        /// update source points
        source_trans -> clear();
        
        transformPointCloud2d (*source, *source_trans, out_transformation);

        /// Check convergence
        Eigen::Matrix4f transformation_epsilon;
        transformation_epsilon = out_transformation - transformation_prev;

        float transformation_epsilon_translation = pow(transformation_epsilon(0,3),2) + pow(transformation_epsilon(1,3),2);
        float transformation_epsilon_rotation = pow(transformation_epsilon(0,0),2) + pow(transformation_epsilon(0,1),2)
                                        + pow(transformation_epsilon(1,0),2) + pow(transformation_epsilon(1,1),2);

        pcl::PointCloud<pcl::PointXY>::Ptr src_trans (new pcl::PointCloud<pcl::PointXY>);
        transformPointCloud2d (*src, *src_trans, transformation_est);

        float euclidean_dist_epsilon = 0.0;

        float rmse = 0.0;

        size_t num_of_points = src->size();
        
        for (size_t j = 0; j < num_of_points; ++j)
        {
            rmse += pow(src_trans->at(j).x - trg->at(j).x, 2);
            rmse += pow(src_trans->at(j).y - trg->at(j).y, 2);

            euclidean_dist_epsilon += pow(src_trans->at(j).x - src->at(j).x, 2);
            euclidean_dist_epsilon += pow(src_trans->at(j).x - src->at(j).x, 2);
        }
        rmse /= (float)num_of_points;
        euclidean_dist_epsilon /= (float)num_of_points;

        float rmse_epsilon = pow(rmse - prev_rmse, 2);

        // std::cout << "transformation_epsilon_translation: " << transformation_epsilon_translation << std::endl;
        // std::cout << "transformation_epsilon_rotation: " << transformation_epsilon_rotation << std::endl;
        // std::cout << "rmse: " << rmse << std::endl;
        // std::cout << "rmse_epsilon: " << rmse_epsilon << std::endl;

        if (convergence_criterion_transformation_epsilon_translation > transformation_epsilon_translation &&
        convergence_criterion_transformation_epsilon_rotation > transformation_epsilon_rotation &&
        convergence_criterion_euclidean_distance_difference_epsilon > euclidean_dist_epsilon && 
        convergence_criterion_euclidean_distance_error > rmse)
        {
            convergence = true;
            iteration = i;
            convergence_criterion_euclidean_distance_error = rmse;
            break;
        }
        
        transformation_prev = out_transformation;
    }

    init_pose_results = out_transformation;
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


void LayeredICP::estimateTransformationSVD(
    const pcl::PointCloud<pcl::PointXY> &cloud_src,
    const pcl::PointCloud<pcl::PointXY> &cloud_trg,
    Eigen::Matrix4f &transformation_matrix)
{
    transformation_matrix = Eigen::Matrix4f::Identity();
    /// Find optimized transformation using SVD
    ///   (reference: https://github.com/ClayFlannigan/icp, PCL)
    
    /// check size of the point clouds
    size_t nr_points = cloud_src.points.size ();
    if (cloud_trg.points.size () != nr_points)
    {
        std::cerr << "[pcl::TransformationEstimationSVD::estimateRigidTransformation] Number or points in source differs than target!" << std::endl;
        return;
    }

    /// Compute centroid for translation
    pcl::PointXY center_src;
    pcl::PointXY center_trg;

    computeCentroid2d(cloud_src, center_src);
    computeCentroid2d(cloud_trg, center_trg);

    Eigen::Matrix<float, 2, 1> centeroid_src, centeroid_trg;
    centeroid_src(0,0)  = center_src.x;
    centeroid_src(1,0)  = center_src.y;
    centeroid_trg(0,0)  = center_trg.x;
    centeroid_trg(1,0)  = center_trg.y;

    // std::cout << "centeroid_src, centeroid_trg" << std::endl;
    // std::cout << centeroid_src << std::endl;
    // std::cout << centeroid_trg << std::endl;
    
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> cloud_src_demean, cloud_trg_demean;

    cloud_src_demean = Eigen::Matrix<float, 2, Eigen::Dynamic>::Zero (2, nr_points);
    cloud_trg_demean = Eigen::Matrix<float, 2, Eigen::Dynamic>::Zero (2, nr_points);

    auto it_cloud_src = cloud_src.begin();
    for (size_t i_cloud = 0; i_cloud < nr_points; ++i_cloud)
    {
        cloud_src_demean(0, i_cloud) = it_cloud_src->x - center_src.x;
        cloud_src_demean(1, i_cloud) = it_cloud_src->y - center_src.y;
        it_cloud_src++;
    }

    auto it_cloud_trg = cloud_trg.begin();
    for (size_t i_cloud = 0; i_cloud < nr_points; ++i_cloud)
    {
        cloud_trg_demean(0, i_cloud) = it_cloud_trg->x - center_trg.x;
        cloud_trg_demean(1, i_cloud) = it_cloud_trg->y - center_trg.y;
        it_cloud_trg++;
    }

    /// Compute rotation matrix using SVD
    /// * Assemble the correlation matrix H = source * target'
    Eigen::Matrix<float, 2, 2> H = (cloud_src_demean * cloud_trg_demean.transpose ());

    /// * Compute the Singular Value Decomposition
    Eigen::JacobiSVD<Eigen::Matrix<float, 2, 2> > svd (H, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix<float, 2, 2> u = svd.matrixU ();
    Eigen::Matrix<float, 2, 2> v = svd.matrixV ();

    /// * Compute R = V * U'
    if (u.determinant () * v.determinant () < 0)
    {
        for (int x = 0; x < 2; ++x)
        v (x, 1) *= -1;
    }

    Eigen::Matrix<float, 2, 2> R = v * u.transpose ();

    // Return the correct translation
    transformation_matrix.topLeftCorner (2, 2) = R;

    // std::cout << "R" << std::endl << R << std::endl;
    // std::cout << "centeroid_src" << std::endl << centeroid_src << std::endl;
    const Eigen::Matrix<float, 2, 1> Rc (R * centeroid_src);
    // std::cout << "transformation_matrix" << std::endl << transformation_matrix << std::endl;
    transformation_matrix(0,3) = centeroid_trg(0,0) - Rc(0,0);
    transformation_matrix(1,3) = centeroid_trg(1,0) - Rc(1,0);
    // std::cout << "transformation_matrix" << std::endl << transformation_matrix << std::endl;
}


void LayeredICP::computeCentroid2d(
    const pcl::PointCloud<pcl::PointXY> &in_cloud, 
    pcl::PointXY &out_centroid)
{
    out_centroid.x = 0.0;
    out_centroid.y = 0.0;

    size_t num = in_cloud.size();

    for (size_t i = 0; i < num ; ++i)
    {
        out_centroid.x += in_cloud[i].x;
        out_centroid.y += in_cloud[i].y;
    }

    out_centroid.x /= (float)num;
    out_centroid.y /= (float)num;
}