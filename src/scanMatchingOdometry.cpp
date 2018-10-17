#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_broadcaster.h>

#include <Eigen/Core>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/registration.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/ndt.h>
#include <../include/hdl_graph_slam_reconstruction/gicp_omp.h>
#include <../include/hdl_graph_slam_reconstruction/gicp_omp_impl.hpp>
#include <../include/hdl_graph_slam_reconstruction/ndt_omp.h>
#include <../include/hdl_graph_slam_reconstruction/ndt_omp_impl.hpp>

#include <iostream>
#include <string>
#include <time.h>

class ScanMatchingOdometry{
public:
    typedef pcl::PointXYZI PointT;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    ScanMatchingOdometry(){
        std::cout << "scan matching odometry" << std::endl;
        initialize_param();
    }
private:
    void initialize_param(){
        //sub and pub
        _subPoints = _nh.subscribe("/filtered_points",32,&ScanMatchingOdometry::cloudMsgsHandler,this);
        _pubOdom = _nh.advertise<nav_msgs::Odometry>("/odom",32);
        _read_until_pub = _nh.advertise<std_msgs::Header>("/scan_matching_odometry/read_until",32);
        //parameters
        ros::param::get("~keyframe_delta_trans",_keyframe_delta_trans);
        ros::param::get("~keyframe_delta_angle",_keyframe_delta_angle);
        ros::param::get("~keyframe_delta_time",_keyframe_delta_time);
        ros::param::get("~odom_frame_id",_odom_frame_id);
        ros::param::get("~max_acceptable_trans",_max_acceptable_trans);
        ros::param::get("~max_acceptable_angle",_max_acceptable_angle);
        /*
        std::cout << "_keyframe_delta_trans: " << _keyframe_delta_trans << std::endl;
        std::cout << "_keyframe_delta_angle: " << _keyframe_delta_angle << std::endl;
        std::cout << "_keyframe_delta_time: " << _keyframe_delta_time << std::endl;
        std::cout << "_max_acceptable_trans: " << _max_acceptable_trans << std::endl;
        std::cout << "_max_acceptable_angle: " << _max_acceptable_angle << std::endl;
        std::cout << "_odom_frame_id: " << _odom_frame_id << std::endl;
        */
        //select downsample method: VOXELGRID, APPROX_VOXELGRID
        boost::shared_ptr<pcl::VoxelGrid<PointT>> voxelgrid(new pcl::VoxelGrid<PointT>());
        double downsample_resolution;
        ros::param::get("~downsample_resolution",downsample_resolution);
        //std::cout << "downsmaple_resolution: " << downsample_resolution << std::endl;
        voxelgrid->setLeafSize(downsample_resolution, downsample_resolution, downsample_resolution);
        _downsample_filter = voxelgrid;
        //select registration method: ICP, GICP, NDT, NDT_OMP
        std::string registration_method;
        ros::param::get("~registration_method",registration_method);
        _registration = select_registration_method(registration_method);
    }
    void cloudMsgsHandler(const sensor_msgs::PointCloud2ConstPtr& src_cloud){
        if( !ros::ok() ){
            return;
        }
        ROS_INFO("new filtered cloud is coming!");
        pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
        pcl::fromROSMsg(*src_cloud,*cloud);
        
        Eigen::Matrix4f pose = matching(src_cloud->header.stamp,cloud);
        std::cout << pose << std::endl;
        publish_odometry(src_cloud->header.stamp,src_cloud->header.frame_id,pose);
        
    }
    Eigen::Matrix4f matching(const ros::Time& stamp, const pcl::PointCloud<PointT>::ConstPtr& cloud){
        //none keyframe exists
        if( !_keyframe ){
            _prev_trans.setIdentity();
            _keyframe_pose.setIdentity();
            _keyframe_stamp = stamp;
            _keyframe = downsample(cloud);
            _registration->setInputTarget(_keyframe);
            return Eigen::Matrix4f::Identity();
        }
        else{
            auto filtered = downsample(cloud);
            _registration->setInputSource(filtered);

            pcl::PointCloud<PointT>::Ptr aligned(new pcl::PointCloud<PointT>());
            _registration->align(*aligned,_prev_trans);     //初值： _prev_trans
            
            if( !_registration->hasConverged() ){       //not converged
                std::cerr << "Scan matching has not been converged!" << std::endl;
                std::cout << "Ignore this frame: " << stamp << std::endl;
                return _keyframe_pose*_prev_trans;
            }
            else{                                       //converged
                Eigen::Matrix4f trans = _registration->getFinalTransformation();
                Eigen::Matrix4f odom = _keyframe_pose*trans;
                //validation by thresholding
                Eigen::Matrix4f delta = _prev_trans.inverse()*trans;        //delta transformation between current frame and last frame
                double delta_trans = delta.block<3,1>(0,3).norm();          //
                double delta_angle = acos( Eigen::Quaternionf(delta.block<3,3>(0,0)).w() );
                if( delta_trans > _max_acceptable_trans || delta_angle > _max_acceptable_angle ){
                    std::cerr << "Too large transform: dx: " << delta_trans << " da: "<< delta_angle << std::endl;
                    std::cout << "Ignore this frame: " << stamp << std::endl;
                    return _keyframe_pose*_prev_trans;
                }
                //update odom and keyframe
                _prev_trans = trans;
                delta_trans = trans.block<3,1>(0,3).norm();
                delta_angle = acos( Eigen::Quaternionf( trans.block<3,3>(0,0) ).w() );\
                double delta_time = (stamp - _keyframe_stamp).toSec();
                if( delta_trans > _keyframe_delta_trans || delta_angle > _keyframe_delta_angle || delta_time > _keyframe_delta_time ){       //update keyframe
                    _keyframe_stamp = stamp;
                    _keyframe_pose = odom;
                    _prev_trans = Eigen::Matrix4f::Identity();
                    _keyframe = filtered;
                    _registration->setInputTarget(_keyframe);
                }
                return odom;
            }
        }
    }
    pcl::PointCloud<PointT>::ConstPtr downsample(const pcl::PointCloud<PointT>::ConstPtr& src_cloud){
        pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
        _downsample_filter->setInputCloud(src_cloud);
        _downsample_filter->filter(*cloud);
        return cloud;
    }
    void publish_odometry(const ros::Time& stamp,const std::string& base_frame_id,Eigen::Matrix4f pose){
        geometry_msgs::TransformStamped odom_trans = matrix2transform(stamp, pose, _odom_frame_id, base_frame_id);
        // publish the transform
        nav_msgs::Odometry odom;
        odom.header.stamp = stamp;
        odom.header.frame_id = _odom_frame_id;

        odom.pose.pose.position.x = pose(0, 3);
        odom.pose.pose.position.y = pose(1, 3);
        odom.pose.pose.position.z = pose(2, 3);
        odom.pose.pose.orientation = odom_trans.transform.rotation;

        odom.child_frame_id = base_frame_id;
        odom.twist.twist.linear.x = 0.0;
        odom.twist.twist.linear.y = 0.0;
        odom.twist.twist.angular.z = 0.0;

        _pubOdom.publish(odom);
    }
    static geometry_msgs::TransformStamped matrix2transform(const ros::Time& stamp, Eigen::Matrix4f pose, std::string frame_id,std::string child_frame_id){
        Eigen::Quaternionf quat(pose.block<3, 3>(0, 0));
        quat.normalize();
        geometry_msgs::Quaternion odom_quat;
        odom_quat.w = quat.w();
        odom_quat.x = quat.x();
        odom_quat.y = quat.y();
        odom_quat.z = quat.z();
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = stamp;
        odom_trans.header.frame_id = frame_id;
        odom_trans.child_frame_id = child_frame_id;

        odom_trans.transform.translation.x = pose(0, 3);
        odom_trans.transform.translation.y = pose(1, 3);
        odom_trans.transform.translation.z = pose(2, 3);
        odom_trans.transform.rotation = odom_quat;

        return odom_trans;
    }
    boost::shared_ptr<pcl::Registration<PointT,PointT> > select_registration_method(std::string registration_method){
        //ICP, GICP, NDT
        if( registration_method == "ICP" ){
            boost::shared_ptr<pcl::IterativeClosestPoint<PointT, PointT> > icp(new pcl::IterativeClosestPoint<PointT, PointT>());
            return icp;
        }
        else if( registration_method == "GICP" ){
            boost::shared_ptr<pclomp::GeneralizedIterativeClosestPoint<PointT, PointT> > gicp(new pclomp::GeneralizedIterativeClosestPoint<PointT, PointT>());
            return gicp;
        }
        else if( registration_method == "NDT" ){
            boost::shared_ptr<pcl::NormalDistributionsTransform<PointT, PointT> > ndt(new pcl::NormalDistributionsTransform<PointT, PointT>());
            ndt->setTransformationEpsilon(0.01);
            double ndt_resolution;
            ros::param::get("~ndt_resolution",ndt_resolution);
            ndt->setResolution(ndt_resolution);
            return ndt;
        }
        else if( registration_method == "NDTOMP" ){
            boost::shared_ptr<pclomp::NormalDistributionsTransform<PointT, PointT>> ndt(new pclomp::NormalDistributionsTransform<PointT, PointT>());
            ndt->setTransformationEpsilon(0.01);
            ndt->setMaximumIterations(64);
            ndt->setResolution(0.5);
            ndt->setNeighborhoodSearchMethod(pclomp::KDTREE);
            return ndt;
        }
        else{
            return nullptr;
        }
    }
private:
    ros::NodeHandle _nh;
    ros::NodeHandle _private_nh;

    //sub and pub
    ros::Subscriber _subPoints;
    ros::Publisher _pubOdom;
    ros::Publisher _read_until_pub;     //?

    //tf broadcaster
    tf::TransformBroadcaster _odom_broadcaster;         //odometry broadcaster
    tf::TransformBroadcaster _keyframe_broadcaster;     //keyframe broadcaster

    //keyframe parameters
    double _keyframe_delta_trans;
    double _keyframe_delta_angle;
    double _keyframe_delta_time;

    //registration validation by thresholding
    double _max_acceptable_trans;
    double _max_acceptable_angle;

    //odometry and calculation
    std::string _odom_frame_id;
    Eigen::Matrix4f _prev_trans;
    Eigen::Matrix4f _keyframe_pose;
    ros::Time _keyframe_stamp;
    pcl::PointCloud<PointT>::ConstPtr _keyframe;
    
    //registration and filter method
    pcl::Filter<PointT>::Ptr _downsample_filter;
    pcl::Registration<PointT,PointT>::Ptr _registration;
};

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "scanMatchingOdometry");
    ScanMatchingOdometry scanMatchingOdometry;
    
    ros::Rate loop(2);
    while( ros::ok() ){
        ros::spinOnce();
        loop.sleep();
    }
    return 0;
}