#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>
#include <hdl_graph_slam_reconstruction/FloorCoeffs1.h>
#include <hdl_graph_slam_reconstruction/keyframe_updater.hpp>
#include <hdl_graph_slam_reconstruction/keyframe.hpp>
#include <hdl_graph_slam_reconstruction/graph_slam.hpp>
#include <hdl_graph_slam_reconstruction/ros_time_hash.hpp>
#include <hdl_graph_slam_reconstruction/information_matrix_calculator.hpp>
#include <hdl_graph_slam_reconstruction/Loopdetector.hpp>

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
        ros::param::get("~graph_update_interval",_graph_update_interval);
        //subscriber
        _subOdom.reset(new message_filters::Subscriber<nav_msgs::Odometry>(_nh,"/odom",32));
        _subPoints.reset(new message_filters::Subscriber<sensor_msgs::PointCloud2>(_nh,"/filtered_points",32));
        _sync.reset(new message_filters::TimeSynchronizer<sensor_msgs::PointCloud2,nav_msgs::Odometry>(*_subPoints,*_subOdom,32));
        _sync->registerCallback(boost::bind(&LoopDetection::cloudMsgsHandler, this, _1, _2));
        _subFloor = _nh.subscribe<hdl_graph_slam_reconstruction::FloorCoeffs1>("/floor_coeffs",32,&LoopDetection::floorCoeffsMsgsHandler,this);
        _pubMarker = _nh.advertise<visualization_msgs::MarkerArray>("/markers",16);
        //reset
        _keyframeUpdater.reset(new KeyFrameUpdater());
        _graphslam.reset(new GraphSlam());
        _loop_detector.reset(new LoopDetector());
        _trans_odom2map.setIdentity();
        _floor_plane_node = nullptr;
        //optimization
        _optimizationTimer = _nh.createTimer(ros::Duration(_graph_update_interval),&LoopDetection::optimization_timer_callback,this);
    }
    void cloudMsgsHandler(const sensor_msgs::PointCloud2::ConstPtr& src_cloud,const nav_msgs::Odometry::ConstPtr& odom) {
        //std::cout << "cloud msgs!" << std::endl;
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
        //std::cout << "size of keyframe queue: " << _keyframe_queue.size() << std::endl;
    }
    bool flush_keyframe_queue(){    //add all keyframes into the pose graph, return true if at least one keyframe exists
        std::cout << "flush keyframe queue" << std::endl;
        //std::cout << "size of keyframe queue: " << _keyframe_queue.size() << std::endl;
        std::lock_guard<std::mutex> lock(_keyframe_queue_mutex);
        if( _keyframe_queue.empty() ){
            return false;
        }
        _trans_odom2map_mutex.lock();
        Eigen::Isometry3d odom2map(_trans_odom2map.cast<double>());
        _trans_odom2map_mutex.unlock();

        int num_processed = -1;
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
        //std::cout << "Has processed " << num_processed+1 << " frames this time" << std::endl;
        _keyframe_queue.erase(_keyframe_queue.begin(), _keyframe_queue.begin() + num_processed + 1);
        //std::cout << "size of keyframe queue: " << _keyframe_queue.size() << std::endl;
        return true;
    }
    void floorCoeffsMsgsHandler(const hdl_graph_slam_reconstruction::FloorCoeffs1::ConstPtr& floor_coeffs_msgs){
        //std::cout << "floor coeffs msgs!" << std::endl;
        if( floor_coeffs_msgs->coeffs.empty() ){
            return;
        }
        //std::cout << "floor coeffs msgs!" << std::endl;
        std::lock_guard<std::mutex> lock(_floor_coeffs_queue_mutex);
        _floor_coeffs_queue.push_back(floor_coeffs_msgs);
    }
    bool flush_floor_coeffs_queue(){
        std::cout << "flush floor queue" << std::endl;
    }
    void optimization_timer_callback(const ros::TimerEvent& event){
        std::cout << "optimization call back!" << std::endl;
        if( !flush_keyframe_queue() ){
            return; 
        }
        //loop detection and add loop-edge into graph
        std::cout << "Detect loop..." << std::endl;
        std::vector<Loop::Ptr> loops = _loop_detector->detectLoop(_keyframes,_new_keyframe_queue,*_graphslam);
        std::cout << "Loop detected" << std::endl; 
        for(const auto& loop : loops) {
            Eigen::Isometry3d relpose(loop->_relpose.cast<double>());
            Eigen::MatrixXd information_matrix = _info_matrix_calculator.calc_information_matrix(loop->_keyframe1->_cloud, loop->_keyframe2->_cloud, relpose);
            _graphslam->add_se3_edge(loop->_keyframe1->_node, loop->_keyframe2->_node, relpose, information_matrix);
        }
        std::copy(_new_keyframe_queue.begin(), _new_keyframe_queue.end(), std::back_inserter(_keyframes));
        _new_keyframe_queue.clear();
        std::cout << "size of keyframes: " << _keyframes.size() << std::endl;
        //publish marker
        visualization_msgs::MarkerArray markerArray = create_marker_array(event.current_real);
        _pubMarker.publish(markerArray);
        // optimize the pose graph
        std::cout << "optimize..." << std::endl;
        _graphslam->optimize();
        std::cout << "Optimization is done" << std::endl;
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
    //add markers for nodes and edges
    visualization_msgs::MarkerArray create_marker_array(const ros::Time& stamp) const {
        std::cout << "create marker" << std::endl;
        visualization_msgs::MarkerArray markerArray;
        markerArray.markers.resize(5);
        
        //add keyframe_trajectory array
        visualization_msgs::Marker& trajectory_marker = markerArray.markers[0];
        trajectory_marker.header.frame_id= "map";
        trajectory_marker.header.stamp = stamp;
        trajectory_marker.ns = "nodes";
        trajectory_marker.id = 0;
        trajectory_marker.type = visualization_msgs::Marker::SPHERE_LIST;

        trajectory_marker.pose.orientation.w = 1.0;
        trajectory_marker.scale.x = trajectory_marker.scale.y = trajectory_marker.scale.z = 0.5;

        trajectory_marker.points.resize(_keyframes.size());
        trajectory_marker.colors.resize(_keyframes.size());
        for(int i = 0;i < _keyframes.size();i++){
            Eigen::Vector3d pos = _keyframes[i]->_node->estimate().translation();
            trajectory_marker.points[i].x = pos.x();
            trajectory_marker.points[i].y = pos.y();
            trajectory_marker.points[i].z = pos.z();

            double p = static_cast<double>(i) / _keyframes.size();
            trajectory_marker.colors[i].r = 1.0 - p;
            trajectory_marker.colors[i].g = p;
            trajectory_marker.colors[i].b = 0.0;
            trajectory_marker.colors[i].a = 1.0;
        }
        return markerArray;
    }     
private:
    ros::NodeHandle _nh;

    std::unique_ptr<message_filters::Subscriber<nav_msgs::Odometry> > _subOdom;                                 //for odometry
    std::unique_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2> > _subPoints;                         //for point cloud
    std::unique_ptr<message_filters::TimeSynchronizer<sensor_msgs::PointCloud2,nav_msgs::Odometry> > _sync;     //for synchronization

    ros::Subscriber _subGPS;    //for GPS
    ros::Subscriber _subFloor;  //for detected floor
    ros::Subscriber _subNmea;   //for nmea messages

    ros::Publisher _pubMarker;  //for marker messages

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
    g2o::VertexPlane* _floor_plane_node;

    Eigen::Isometry3d _trans_odom2map;  //transformation between odom and map coordinate
    std::mutex _trans_odom2map_mutex;

    std::unique_ptr<GraphSlam> _graphslam;
    InformationMatrixCalculator _info_matrix_calculator;

    //loop detector
    std::unique_ptr<LoopDetector> _loop_detector;
    //optimizaion
    double _graph_update_interval;
    ros::Timer _optimizationTimer;
};

int main(int argc, char *argv[]){
    ros::init(argc, argv, "loopDetection");
    ROS_INFO("loop detection is initializing!");
    LoopDetection loopDetection;
    ros::Rate loop(10);
    ROS_INFO("loop detection is initializing done!");
    std::vector<int> a;
    a.push_back(1), a.push_back(2), a.push_back(3);
    a.erase(a.begin(),a.begin()+1);
    while ( ros::ok() ){
        loop.sleep();
        ros::spinOnce();
    }
    
    return 0;
}