#ifndef INFORMARION_MATRIX_CALCULATOR_HPP
#define INFORMARION_MATRIX_CALCULATOR_HPP

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/search/kdtree.h>
#include <pcl/common/transforms.h>
#include <Eigen/Dense>

class InformationMatrixCalculator{
public:
    typedef pcl::PointXYZI PointT;
    InformationMatrixCalculator(){
        
        ros::param::get("~use_const_info_matrix",_use_const_info_matrix);
        ros::param::get("~const_stddev_x",_const_stddev_x);
        ros::param::get("~const_stddev_q",_const_stddev_q);
        
        ros::param::get("~min_stddev_x",_min_stddev_x);
        ros::param::get("~max_stddev_x",_max_stddev_x);
        ros::param::get("~min_stddev_q",_min_stddev_q);
        ros::param::get("~max_stddev_q",_max_stddev_q);

        ros::param::get("~gain",_gain);
        ros::param::get("~maxRange",_maxRange);
        ros::param::get("~fitness_score_thresh",_fitness_score_thresh);
        std::cout << "Information matrix calculator!" << std::endl;
    }
    Eigen::MatrixXd calc_information_matrix(const pcl::PointCloud<PointT>::ConstPtr& cloud1,const pcl::PointCloud<PointT>::ConstPtr& cloud2,const Eigen::Isometry3d& relpose) const {
        if(_use_const_info_matrix){
            Eigen::MatrixXd info = Eigen::MatrixXd::Identity(6,6);
            info.topLeftCorner(3,3).array() /= _const_stddev_x;
            info.bottomRightCorner(3,3).array() /= _const_stddev_q;
            return info;
        }
        double fitness_score = calcu_fitness_score(cloud1,cloud2,relpose);
        
        double min_var_x = std::pow(_min_stddev_x,2);
        double max_var_x = std::pow(_max_stddev_x,2);
        double min_var_q = std::pow(_min_stddev_q,2);
        double max_var_q = std::pow(_max_stddev_q,2);

        float w_x = weight(_gain, _fitness_score_thresh, min_var_x, max_var_x, fitness_score);
        float w_q = weight(_gain, _fitness_score_thresh, min_var_q, max_var_q, fitness_score);

        Eigen::MatrixXd info = Eigen::MatrixXd::Identity(6,6);
        info.topLeftCorner(3,3).array() /= w_x;
        info.bottomRightCorner(3,3).array() /= w_q;
        return info;
    }
    double weight(double gain,double xmax,double ymin,double ymax,double x) const {
        double y = (1.0 - std::exp(-gain*x))/(1.0-std::exp(-gain*xmax));
        return ymin + (ymax - ymin) * y;
    }
    ~InformationMatrixCalculator(){
    }
private:
    double calcu_fitness_score(const pcl::PointCloud<PointT>::ConstPtr& cloud1,const pcl::PointCloud<PointT>::ConstPtr& cloud2,const Eigen::Isometry3d& relpose) const {
        //kd tree
        pcl::search::KdTree<PointT>::Ptr tree_(new pcl::search::KdTree<PointT>());
        tree_->setInputCloud(cloud1);
        
        //transform cloud2 using relpose
        pcl::PointCloud<PointT> transformedCloud;
        pcl::transformPointCloud(*cloud2,transformedCloud,relpose.cast<float>());
        
        std::vector<int> nn_indices (1);
        std::vector<float> nn_dists (1);
        //iterate each point in source point cloud
        double fitness_score = 0.0f;
        int num = 0;
        for(int i = 0; i < transformedCloud.points.size();i++){
            tree_->nearestKSearch (transformedCloud.points[i], 1, nn_indices, nn_dists);    //search nearest point
            if( nn_dists[0] <= _maxRange  ){
                fitness_score += nn_dists[0];
                num++;
            }
        }
        if( num > 0 )
            return fitness_score/(1.0*num);
        else
            return 2*_maxRange;
    }
private:
    bool _use_const_info_matrix;
    double _const_stddev_x;
    double _const_stddev_q;

    double _min_stddev_x;
    double _max_stddev_x;
    double _min_stddev_q;
    double _max_stddev_q;

    double _gain;
    double _maxRange;
    double _fitness_score_thresh;
};

#endif