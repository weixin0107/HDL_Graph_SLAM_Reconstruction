#ifndef KEYFRAME_HPP
#define KEYFRAME_HPP

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <Eigen/Dense>
#include <g2o/types/slam3d/vertex_se3.h>

namespace g2o {
  class VertexSE3;
}

class Keyframe{     //pose node
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef pcl::PointXYZI PointT;
    using Ptr = std::shared_ptr<Keyframe>;
    Keyframe(const ros::Time& stamp, const Eigen::Isometry3d& odom, double accum_distance, const pcl::PointCloud<PointT>::ConstPtr& cloud):_stamp(stamp),
    _odom(odom),_accum_dist(accum_distance),_cloud(cloud),_node(nullptr){
    }

    void dump(const std::string& directory){
    }
    
    ~Keyframe(){};
public:
    ros::Time _stamp;
    Eigen::Isometry3d _odom;
    double _accum_dist;
    pcl::PointCloud<PointT>::ConstPtr _cloud;

    boost::optional<Eigen::Vector4f> _floorCoeffs;
    g2o::VertexSE3* _node;                           // node instance
};

class KeyframeSnapshot{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef pcl::PointXYZI PointT;
    using Ptr = std::shared_ptr<KeyframeSnapshot>;
    KeyframeSnapshot(const Eigen::Isometry3d& pose,const pcl::PointCloud<PointT>::ConstPtr& cloud):_pose(pose),_cloud(cloud){
    }
    KeyframeSnapshot(const Keyframe::Ptr& key) : _pose(key->_node->estimate()),
    _cloud(key->_cloud){
    }
public:
    Eigen::Isometry3d _pose;
    pcl::PointCloud<PointT>::ConstPtr _cloud;
};

#endif