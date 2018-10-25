#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <hdl_graph_slam_reconstruction/FloorCoeffs1.h>

#include <iostream>

#include <pcl_ros/point_cloud.h>
#include <pcl/filters/impl/plane_clipper3D.hpp>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>

#include <Eigen/Core>

class FloorDetection{
public:
    typedef pcl::PointXYZI PointT;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    FloorDetection(){
        initial_params();
    }
private:
    void initial_params(){
        _subPoints = _nh.subscribe("/filtered_points",32,&FloorDetection::cloudMsgsHandler,this);
        _pubFloorPoints = _nh.advertise<sensor_msgs::PointCloud2>("/floor_points",32);
        _pubFloor = _nh.advertise<hdl_graph_slam_reconstruction::FloorCoeffs1>("/floor_coeffs",32);
        ros::param::get("~tiltDeg",_tiltDeg);
        ros::param::get("~sensorHeight",_sensorHeight);
        ros::param::get("~heightRange",_heightRange);
        ros::param::get("~floorNormalThresh",_floorNormalThresh);
    }
    void cloudMsgsHandler(const sensor_msgs::PointCloud2ConstPtr& src_cloud){
        pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
        pcl::fromROSMsg(*src_cloud,*cloud);
        if( cloud->empty() ){
            return;
        }
        boost::optional<Eigen::Vector4f> normal = detectFloor(cloud);
        //publish detected floor coeffs
        hdl_graph_slam_reconstruction::FloorCoeffs1 floorCoeffs;
        floorCoeffs.header = src_cloud->header;
        if( normal ){
            floorCoeffs.coeffs.resize(4);
            for(int i=0; i<4; i++) {
                floorCoeffs.coeffs[i] = (*normal)[i];
            }
        }
        _pubFloor.publish(floorCoeffs);
    }
    boost::optional<Eigen::Vector4f> detectFloor(const pcl::PointCloud<PointT>::Ptr& cloud) const {
        pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>());

        //height filtering
        filtered = planeFilter(cloud,Eigen::Vector4f(0.0f,0.0f,1.0f,_sensorHeight+_heightRange),false);
        filtered = planeFilter(filtered,Eigen::Vector4f(0.0f,0.0f,1.0f,_sensorHeight-_heightRange),true);
        //normal filtering
        filtered = normalFilter(filtered);
        
        //floor detection with RANSAC
        if( filtered->size() < _floorPointsThresh ){
            return boost::none;
        }

        pcl::SampleConsensusModelPlane<PointT>::Ptr model_p(new pcl::SampleConsensusModelPlane<PointT>(filtered));      //plane model
        pcl::RandomSampleConsensus<PointT> ransac(model_p);     //ransac with plane model
        ransac.setDistanceThreshold(0.1);
        ransac.computeModel();

        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        ransac.getInliers(inliers->indices);

        //too few inlier points
        if( inliers->indices.size() < _floorPointsThresh ){
            return boost::none;
        }
        //std::cout << "num of inliers: " << inliers->indices.size() << std::endl;
        //extract inlier points
        pcl::PointCloud<PointT>::Ptr inlier_cloud(new pcl::PointCloud<PointT>);
        pcl::ExtractIndices<PointT> extract;
        extract.setInputCloud(filtered);
        extract.setIndices(inliers);
        extract.filter(*inlier_cloud);
        inlier_cloud->header = cloud->header;
        //publish floor detected result
        inlier_cloud->header = cloud->header;
        _pubFloorPoints.publish(inlier_cloud);
        
        //floor coeffs
        Eigen::VectorXf coeffs;
        ransac.getModelCoefficients(coeffs);
        //make the normal upward
        if(coeffs.head<3>().dot(Eigen::Vector3f::UnitZ()) < 0.0f) {
            coeffs *= -1.0f;
        }
        return Eigen::Vector4f(coeffs);
    }
    pcl::PointCloud<PointT>::Ptr planeFilter(const pcl::PointCloud<PointT>::Ptr& cloud,const Eigen::Vector4f& vec,bool flag) const {
        pcl::PlaneClipper3D<PointT> planeClipper(vec);     //define plane clipper
        
        pcl::PointIndices::Ptr indices(new pcl::PointIndices);

        planeClipper.clipPointCloud3D(*cloud, indices->indices);

        pcl::PointCloud<PointT>::Ptr dst_cloud(new pcl::PointCloud<PointT>);

        pcl::ExtractIndices<PointT> extract;
        extract.setInputCloud(cloud);
        extract.setIndices(indices);
        extract.setNegative(flag);
        extract.filter(*dst_cloud);

        return dst_cloud;
    }
    pcl::PointCloud<PointT>::Ptr normalFilter(const pcl::PointCloud<PointT>::Ptr& cloud) const {
        pcl::NormalEstimation<PointT, pcl::Normal> ne;      //pcl::normal estimation
        ne.setInputCloud(cloud);
        
        pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);     //kd tree search
        ne.setSearchMethod(tree);

        pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
        ne.setKSearch(10);
        ne.setViewPoint(0.0f, 0.0f, _sensorHeight);
        ne.compute(*normals);

        pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>);
        filtered->reserve(cloud->size());

        for( int i = 0; i < cloud->size();i++ ){
            float dot = normals->at(i).getNormalVector3fMap().normalized().dot(Eigen::Vector3f::UnitZ());
            if (std::abs(dot) > std::cos(_floorNormalThresh * M_PI / 180.0)) {
                filtered->push_back(cloud->at(i));
            }
        }
        filtered->width = filtered->size();
        filtered->height = 1;
        return filtered;
    }
private:
    ros::NodeHandle _nh;
    
    //ros topics
    ros::Subscriber _subPoints;
    ros::Publisher _pubFloor;
    ros::Publisher _pubFloorPoints;

    //initial parameters
    double _tiltDeg;
    double _sensorHeight;
    double _heightRange;

    int _floorPointsThresh;
    double _floorNormalThresh;
};

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "floorDetection");
    
    FloorDetection floorDetection;
    ros::Rate loop(10);

    while ( ros::ok() ){
        ros::spinOnce();
        loop.sleep();
    }
    
    return 0;
}