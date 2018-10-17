#ifndef KEYFRAME_UPDATER_HPP
#define KEYFRAME_UPDATER_HPP

#include <ros/ros.h>
#include <Eigen/Dense>
#include <Eigen/Core>


class KeyFrameUpdater{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    KeyFrameUpdater():_isFirst(true),_prev_trans( Eigen::Isometry3d::Identity() ){
        ros::param::get("~keyframe_delta_trans",_keyframe_delta_trans);
        ros::param::get("~keyframe_delta_angle",_keyframe_delta_angle);
        _accum_dist = 0.0;
    }
    bool update(const Eigen::Isometry3d& pose){
        if( _isFirst ){     //The first frame is always set as keyframe
            _prev_trans = pose;
            _isFirst = false;
            return true;
        }
        //std::cout << "dfdfd" << std::endl;
        //std::cout << pose.linear() << std::endl;
        //std::cout << pose.translation() << std::endl;
        Eigen::Isometry3d delta = _prev_trans.inverse()*pose;
        double dx = delta.translation().norm();
        double da = std::acos(Eigen::Quaterniond(delta.linear()).w());
        //too close
        if( dx < _keyframe_delta_trans && da < _keyframe_delta_angle ){
            return false;
        }
        //update keyframe
        _prev_trans = pose;
        _accum_dist += dx;
        return true;
    }
    double getAccumDist(){
        return _accum_dist;
    }
private:
    double _keyframe_delta_trans;
    double _keyframe_delta_angle;

    bool _isFirst;
    double _accum_dist;
    Eigen::Isometry3d _prev_trans;
};

#endif