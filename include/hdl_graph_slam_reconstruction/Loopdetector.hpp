#ifndef LOOPDETECTOR_HPP
#define LOOPDETECTOR_HPP

#include <hdl_graph_slam_reconstruction/keyframe.hpp>
#include <pcl_ros/point_cloud.h>
#include <Eigen/Dense>
#include <hdl_graph_slam_reconstruction/graph_slam.hpp>
#include <pcl/registration/registration.h>
#include <hdl_graph_slam_reconstruction/gicp_omp.h>
#include <hdl_graph_slam_reconstruction/gicp_omp_impl.hpp>



class Loop{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using Ptr = std::shared_ptr<Loop>;

    Loop(const Keyframe::Ptr& key1,const Keyframe::Ptr& key2,const Eigen::Matrix4f& relpose):_keyframe1(key1),_keyframe2(key2),_relpose(relpose){
    }
public:
    Keyframe::Ptr _keyframe1;
    Keyframe::Ptr _keyframe2;
    Eigen::Matrix4f _relpose;
};

class LoopDetector{
public:
    typedef pcl::PointXYZI PointT;
    LoopDetector(){
        distance_thresh = 5.0;
        accum_distance_thresh = 8.0;
        distance_from_last_edge_thresh = 5.0;

        fitness_score_thresh = 0.5;
        last_edge_accum_distance = 0.0;
        boost::shared_ptr<pclomp::GeneralizedIterativeClosestPoint<PointT, PointT> > gicp(new pclomp::GeneralizedIterativeClosestPoint<PointT, PointT>());
        registration = gicp;
    }
    std::vector<Loop::Ptr> detectLoop(const std::vector<Keyframe::Ptr>& keyframes, const std::deque<Keyframe::Ptr>& newkeyframes, GraphSlam& graph_slam ){
        std::vector<Loop::Ptr> loopDetected;
        for(const auto& keyframe : newkeyframes){
            auto candidates = findCandidates(keyframes,keyframe);
            auto loop = matching(candidates, keyframe, graph_slam);
            if(loop){
                loopDetected.push_back(loop);
            }
        }
        return loopDetected;
    }
private:
    std::vector<Keyframe::Ptr> findCandidates(const std::vector<Keyframe::Ptr>& keyframes, const Keyframe::Ptr& newkeyframe) const {
        //too close to the last edge
        if( newkeyframe->_accum_dist - last_edge_accum_distance < distance_from_last_edge_thresh ) {
            return std::vector<Keyframe::Ptr>();
        }
        std::vector<Keyframe::Ptr> candidates;
        candidates.reserve(32);
        for( const auto& k : keyframes ){
            //traveled distance between different keyframes is too small
            if( newkeyframe->_accum_dist - k->_accum_dist < accum_distance_thresh ){
                continue;
            }
            const auto& pos1 = k->_node->estimate().translation();
            const auto& pos2 = newkeyframe->_node->estimate().translation();
            // estimated distance between keyframes is too small
            double dist = (pos1.head<2>() - pos2.head<2>()).norm();
            if(dist > distance_thresh) {
                continue;
            }
            candidates.push_back(k);
        }
        return candidates;
    }
    Loop::Ptr matching(const std::vector<Keyframe::Ptr>& candidate_keyframes,const Keyframe::Ptr& keyframe, GraphSlam& graph_slam){
        //empty
        if( candidate_keyframes.empty() ){
            return nullptr;
        }
        registration->setInputTarget(keyframe->_cloud);
        Keyframe::Ptr best_matched;
        double best_score = std::numeric_limits<double>::max();
        Eigen::Matrix4f relpose;

        std::cout << std::endl;
        std::cout << "--- loop detection ---" << std::endl;
        std::cout << "num_candidates: " << candidate_keyframes.size() << std::endl;
        std::cout << "matching" << std::flush;
        auto t1 = ros::Time::now();

        pcl::PointCloud<PointT>::Ptr aligned(new pcl::PointCloud<PointT>());
        for(const auto& candidate : candidate_keyframes) {
            registration->setInputSource(candidate->_cloud);
            Eigen::Matrix4f guess = (keyframe->_node->estimate().inverse() * candidate->_node->estimate()).matrix().cast<float>();
            guess(2, 3) = 0.0;
            registration->align(*aligned, guess);
            std::cout << "." << std::flush;

            double score = registration->getFitnessScore();
            if(!registration->hasConverged() || score > best_score) {
                continue;
            }

            best_score = score;
            best_matched = candidate;
            relpose = registration->getFinalTransformation();
        }
        auto t2 = ros::Time::now();
        std::cout << " done" << std::endl;
        std::cout << "best_score: " << boost::format("%.3f") % best_score << "    time: " << boost::format("%.3f") % (t2 - t1).toSec() << "[sec]" << std::endl;
        if(best_score > fitness_score_thresh) {
            std::cout << "loop not found..." << std::endl;
            return nullptr;
        }

        std::cout << "loop found!!" << std::endl;
        std::cout << "relpose: " << relpose.block<3, 1>(0, 3) << " - " << Eigen::Quaternionf(relpose.block<3, 3>(0, 0)).coeffs().transpose() << std::endl;
        last_edge_accum_distance = keyframe->_accum_dist;
        return std::make_shared<Loop>(keyframe, best_matched, relpose);
    }
private:
    double distance_thresh;                 // estimated distance between keyframes consisting a loop must be less than this distance
    double accum_distance_thresh;           // traveled distance between ...
    double distance_from_last_edge_thresh;  // a new loop edge must far from the last one at least this distance

    double fitness_score_thresh;            // threshold for scan matching

    double last_edge_accum_distance;

    pcl::Registration<PointT, PointT>::Ptr registration;
};

#endif