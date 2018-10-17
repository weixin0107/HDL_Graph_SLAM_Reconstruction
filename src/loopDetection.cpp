#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <hdl_graph_slam_reconstruction/FloorCoeffs1.h>
#include <hdl_graph_slam_reconstruction/keyframe_updater.hpp>
#include <hdl_graph_slam_reconstruction/keyframe.hpp>
#include <hdl_graph_slam_reconstruction/graph_slam.hpp>
#include <hdl_graph_slam_reconstruction/ros_time_hash.hpp>
#include <hdl_graph_slam_reconstruction/information_matrix_calculator.hpp>

#include <Eigen/Core>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <pcl_ros/point_cloud.h>

#include <g2o/types/slam3d/vertex_se3.h>

#include <iostream>
#include <string>
#include <vector>
#include <mutex>
#include <boost/thread.hpp>
#include <unordered_map>        //for hash table



class LoopDetection{
public:
    typedef pcl::PointXYZI PointT;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    LoopDetection(){
        initial_params();
    }
private:
    void initial_params(){
        //parameters
        ros::param::get("~odom_frame_id",_odom_frame_id);
        ros::param::get("~map_frame_id",_map_frame_id);
        ros::param::get("~max_keyframes_per_update",_max_keyframes_per_update);
        //subscriber
        _subOdom.reset(new message_filters::Subscriber<nav_msgs::Odometry>(_nh,"/odom",32));
        _subPoints.reset(new message_filters::Subscriber<sensor_msgs::PointCloud2>(_nh,"/filtered_points",32));
        _sync.reset(new message_filters::TimeSynchronizer<sensor_msgs::PointCloud2,nav_msgs::Odometry>(*_subPoints,*_subOdom,32));
        _sync->registerCallback(boost::bind(&LoopDetection::cloudMsgsHandler, this, _1, _2));
        _subFloor = _nh.subscribe<hdl_graph_slam_reconstruction::FloorCoeffs1>("/floor_coeffs",32,&LoopDetection::floorCoeffsMsgsHandler,this);
        
        //reset
        _keyframeUpdater.reset(new KeyFrameUpdater());
        _graphslam.reset(new GraphSlam());          //error
        _trans_odom2map.setIdentity();
    }
    void cloudMsgsHandler(const sensor_msgs::PointCloud2::ConstPtr& src_cloud,const nav_msgs::Odometry::ConstPtr& odom) {
        const ros::Time cloudStamp = src_cloud->header.stamp;
        Eigen::Isometry3d trans = odom2isometry(odom);  //odometry to transform matrix

        pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
        pcl::fromROSMsg(*src_cloud,*cloud);
        
        if( !_keyframeUpdater->update(trans) ){
            return;
        }
        
        double accum_dist = _keyframeUpdater->getAccumDist();
        Keyframe::Ptr keyframe( new Keyframe(cloudStamp,trans,accum_dist,cloud) ); 
        
        std::lock_guard<std::mutex> lock(_keyframe_queue_mutex);
        _keyframe_queue.push_back(keyframe);        //update keyframe_queue
        std::cout << "size of keyframe queue: " << _keyframe_queue.size() << std::endl;
    }
    bool flush_keyframe_queue(){    //add all keyframes into the pose graph, return true if at least one keyframe exists
        std::lock_guard<std::mutex> lock(_keyframe_queue_mutex);
        if( _keyframe_queue.empty() ){
            return false;
        }
        _trans_odom2map_mutex.lock();
        Eigen::Isometry3d odom2map(_trans_odom2map.cast<double>());
        _trans_odom2map_mutex.unlock();

        int num_processed = 0;
        for(int i = 0; i < _keyframe_queue.size() && i < _max_keyframes_per_update;i++){
            num_processed = i;
            const auto& keyframe = _keyframe_queue[i];
            _new_keyframe_queue.push_back(keyframe);

            Eigen::Isometry3d odom = odom2map*keyframe->_odom;
            keyframe->_node = _graphslam->add_se3_node(odom);       //add se3 node

            _keyframe_hash[keyframe->_stamp] = keyframe;

            //add edges between different keyframes
            if(i == 0 && _keyframes.empty()){
                continue;
            }
            
            const auto& prev_keyframe = (i==0)?_keyframes.back():_keyframe_queue[i-1];
            Eigen::Isometry3d relative_pose = keyframe->_odom.inverse()*prev_keyframe->_odom;
            Eigen::MatrixXd info = _info_matrix_calculator.calc_information_matrix(keyframe->_cloud,prev_keyframe->_cloud,relative_pose);
            _graphslam->add_se3_edge(keyframe->_node,prev_keyframe->_node,relative_pose,info);
            
        }
        _keyframe_queue.erase(_keyframe_queue.begin(), _keyframe_queue.begin() + num_processed + 1);
    }
    void floorCoeffsMsgsHandler(const hdl_graph_slam_reconstruction::FloorCoeffs1::ConstPtr& floor_coeffs_msgs){
        std::cout << "floor coeffs msgs!" << std::endl;
        if( floor_coeffs_msgs->coeffs.empty() ){
            return;
        }
        std::cout << "floor coeffs msgs!" << std::endl;
        std::lock_guard<std::mutex> lock(_floor_coeffs_queue_mutex);
        _floor_coeffs_queue.push_back(floor_coeffs_msgs);
    }
    static Eigen::Isometry3d odom2isometry(const nav_msgs::Odometry::ConstPtr& odom){
        Eigen::Isometry3d isometry = Eigen::Isometry3d::Identity();
        const auto orientation = odom->pose.pose.orientation;
        const auto position = odom->pose.pose.position;
        Eigen::Quaterniond q;
        q.w() = orientation.w;
        q.x() = orientation.x;
        q.y() = orientation.y;
        q.z() = orientation.z;
        isometry.linear() = q.toRotationMatrix();
        isometry.translation() = Eigen::Vector3d(position.x,position.y,position.z);
        return isometry;
    }
private:
    ros::NodeHandle _nh;

    std::unique_ptr<message_filters::Subscriber<nav_msgs::Odometry> > _subOdom;                                 //for odometry
    std::unique_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2> > _subPoints;                         //for point cloud
    std::unique_ptr<message_filters::TimeSynchronizer<sensor_msgs::PointCloud2,nav_msgs::Odometry> > _sync;     //for synchronization

    ros::Subscriber _subGPS;    //for GPS
    ros::Subscriber _subFloor;  //for detected floor
    ros::Subscriber _subNmea;   //for nmea messages

    std::string _odom_frame_id; //odom frame id
    std::string _map_frame_id;  //map frame id

    std::unique_ptr<KeyFrameUpdater> _keyframeUpdater;

    //keyframe queue
    std::mutex _keyframe_queue_mutex;
    std::deque<Keyframe::Ptr> _keyframe_queue;
    std::deque<Keyframe::Ptr> _new_keyframe_queue;
    std::vector<Keyframe::Ptr> _keyframes;
    std::unordered_map<ros::Time,Keyframe::Ptr,RosTimeHash> _keyframe_hash;
    int _max_keyframes_per_update;

    //floor_coeffs queue
    std::mutex _floor_coeffs_queue_mutex;
    std::deque<hdl_graph_slam_reconstruction::FloorCoeffs1::ConstPtr> _floor_coeffs_queue;

    Eigen::Isometry3d _trans_odom2map;  //transformation between odom and map coordinate
    std::mutex _trans_odom2map_mutex;

    std::unique_ptr<GraphSlam> _graphslam;
    InformationMatrixCalculator _info_matrix_calculator;
};

int main(int argc, char *argv[]){
    ros::init(argc, argv, "loopDetection");
    ROS_INFO("loop detection is initializing!");
    LoopDetection loopDetection;
    ros::Rate loop(10);
    ROS_INFO("loop detection is initializing done!");
    while ( ros::ok() ){
        loop.sleep();
        ros::spinOnce();
    }
    
    return 0;
}