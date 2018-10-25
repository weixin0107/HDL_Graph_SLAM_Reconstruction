#include <iostream>
#include <string>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>




class Prefiltering{
public:
    typedef pcl::PointXYZI PointT;
    Prefiltering(){
        std::cout << "Initializing prefiltering node!" << std::endl;
        initialize_params();
        
        _subPoints = _nh.subscribe("/velodyne_points",64,&Prefiltering::cloudMsgsHandler,this);
        _pubPoints = _nh.advertise<sensor_msgs::PointCloud2>("/filtered_points",32);
    }
    ~Prefiltering(){
    }
private:
    void initialize_params(){
        double downsample_resolution = 0.1;
        boost::shared_ptr<pcl::VoxelGrid<PointT>> voxelgrid(new pcl::VoxelGrid<PointT>());
        voxelgrid->setLeafSize(downsample_resolution, downsample_resolution, downsample_resolution);
        downsample_filter = voxelgrid;

        int mean_k = 20;
        double stddev_mul_thresh = 1.0;
        pcl::StatisticalOutlierRemoval<PointT>::Ptr sor(new pcl::StatisticalOutlierRemoval<PointT>());
        sor->setMeanK(mean_k);
        sor->setStddevMulThresh(stddev_mul_thresh);
        outlier_removal_filter = sor;
    }
    void cloudMsgsHandler(const pcl::PointCloud<PointT>::ConstPtr& src_cloud) const{
        if(src_cloud->empty()) {
            return;
        }
        //ROS_INFO("new source cloud is coming!");
        pcl::PointCloud<PointT>::ConstPtr filtered;
        filtered = downsample(src_cloud); 
        filtered = outlier_removal(filtered);

        _pubPoints.publish(filtered);
    }

    pcl::PointCloud<PointT>::ConstPtr downsample(const pcl::PointCloud<PointT>::ConstPtr& src_cloud) const{
        if(!downsample_filter) {
            return src_cloud;
        }
        pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>());
        downsample_filter->setInputCloud(src_cloud);
        downsample_filter->filter(*filtered);
        filtered->header = src_cloud->header;

        return filtered;
    }

    pcl::PointCloud<PointT>::ConstPtr outlier_removal(const pcl::PointCloud<PointT>::ConstPtr& src_cloud) const{
        if(!outlier_removal_filter) {
            return src_cloud;
        }
        ros::Time start = ros::Time::now();
        pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>());
        outlier_removal_filter->setInputCloud(src_cloud);
        outlier_removal_filter->filter(*filtered);
        filtered->header = src_cloud->header;
        ros::Time end = ros::Time::now();
        //std::cout << (end-start) << std::endl;
        return filtered;
    }
private:
    ros::NodeHandle _nh;
    ros::NodeHandle _private_nh;

    ros::Subscriber _subPoints;
    ros::Publisher _pubPoints;

    pcl::Filter<PointT>::Ptr downsample_filter;
    pcl::Filter<PointT>::Ptr outlier_removal_filter;
};

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "prefiltering");
    
    Prefiltering prefilter;
    ros::Rate loop(10);
    while( ros::ok() ){
        loop.sleep();
        ros::spinOnce();
    }
    return 0;
}