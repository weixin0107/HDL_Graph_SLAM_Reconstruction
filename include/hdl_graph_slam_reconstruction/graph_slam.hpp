#ifndef GRAPH_SLAM_HPP
#define GRAPH_SLAM_HPP

#include <memory>
#include <string>
#include <ros/time.h>
#include <Eigen/Dense>


#include <g2o/core/sparse_optimizer.h>
#include <g2o/stuff/macros.h>
#include <g2o/core/factory.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/linear_solver.h>
#include <g2o/core/optimization_algorithm_factory.h>
#include <g2o/solvers/pcg/linear_solver_pcg.h>
#include <g2o/types/slam3d/types_slam3d.h>
#include <g2o/types/slam3d/edge_se3_pointxyz.h>
#include <g2o/types/slam3d_addons/types_slam3d_addons.h>
#include <g2o/edge_se3_plane.hpp>
#include <g2o/edge_se3_priorxy.hpp>
#include <g2o/edge_se3_priorxyz.hpp>

namespace g2o {
  class VertexSE3;      //se3 ndoe
  class VertexPlane;    //plane node
  class VertexPointXYZ; //point_xyz node
  class EdgeSE3;        //edge between se3 node
  class EdgeSE3Plane;   //edge between se3 and plane node
  class EdgeSE3PointXYZ;//edge between se3 and point_xyz node
  class EdgeSE3PriorXY; //edge between se3 and prior_xy node
  class EdgeSE3PriorXYZ;//edge between se3 and prior_xyz node
}
namespace g2o {
  G2O_REGISTER_TYPE(EDGE_SE3_PLANE, EdgeSE3Plane)
  G2O_REGISTER_TYPE(EDGE_SE3_PRIORXY, EdgeSE3PriorXY)
  G2O_REGISTER_TYPE(EDGE_SE3_PRIORXYZ, EdgeSE3PriorXYZ)
}
G2O_USE_OPTIMIZATION_LIBRARY(csparse)
class GraphSlam{
public:
   // EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    GraphSlam(){
        _graph.reset(new g2o::SparseOptimizer());
        std::cout << "Constructing g2o solver..." << std::endl;
        std::string solver_name("lm_var");
        g2o::OptimizationAlgorithmFactory* solver_factory = g2o::OptimizationAlgorithmFactory::instance();
        g2o::OptimizationAlgorithmProperty solver_property;
        g2o::OptimizationAlgorithm* solver = solver_factory->construct(solver_name, solver_property);
        _graph->setAlgorithm(solver);
        if(!_graph->solver()){
            std::cerr << "error : failed to allocate solver!!" << std::endl;
            solver_factory->listSolvers(std::cerr);
            std::cerr << "-------------" << std::endl;
            std::cin.ignore(1);
            return;
        }
        std::cout << "Done" << std::endl;
        ROS_INFO("blabala");
        //_floor_plane_node = add_plane_node(Eigen::Vector4d(0.0,0.0,1.0,0.0));
        ROS_INFO("blabala");
        //_floor_plane_node->setFixed(true);
        ROS_INFO("Graph slam has been initialized");
    }
    g2o::VertexSE3* add_se3_node(const Eigen::Isometry3d& pose) {           //add se3 node
        g2o::VertexSE3* vertex(new g2o::VertexSE3());
        vertex->setId(_graph->vertices().size());
        vertex->setEstimate(pose);
        _graph->addVertex(vertex);

        return vertex;
    }

    g2o::VertexPlane* add_plane_node(const Eigen::Vector4d& plane_coeffs) { //add plane node
        g2o::VertexPlane* vertex(new g2o::VertexPlane());
        vertex->setId(_graph->vertices().size());
        vertex->setEstimate(plane_coeffs);
        _graph->addVertex(vertex);

        return vertex;
    }
    g2o::VertexPointXYZ* add_point_xyz_node(const Eigen::Vector3d& xyz) {
        g2o::VertexPointXYZ* vertex(new g2o::VertexPointXYZ());
        vertex->setId(_graph->vertices().size());
        vertex->setEstimate(xyz);
        _graph->addVertex(vertex);

        return vertex;
    }
    g2o::EdgeSE3* add_se3_edge(g2o::VertexSE3* v1, g2o::VertexSE3* v2, const Eigen::Isometry3d& relative_pose, const Eigen::MatrixXd& information_matrix) {
        g2o::EdgeSE3* edge(new g2o::EdgeSE3());
        edge->setMeasurement(relative_pose);
        edge->setInformation(information_matrix);
        edge->vertices()[0] = v1;
        edge->vertices()[1] = v2;
        _graph->addEdge(edge);

        return edge;
    }
    g2o::EdgeSE3Plane* add_se3_plane_edge(g2o::VertexSE3* v_se3, g2o::VertexPlane* v_plane, const Eigen::Vector4d& plane_coeffs, const Eigen::MatrixXd& information_matrix) {
        g2o::EdgeSE3Plane* edge(new g2o::EdgeSE3Plane());
        edge->setMeasurement(plane_coeffs);
        edge->setInformation(information_matrix);
        edge->vertices()[0] = v_se3;
        edge->vertices()[1] = v_plane;
        _graph->addEdge(edge);

        return edge;
    }
    g2o::EdgeSE3PointXYZ* add_se3_point_xyz_edge(g2o::VertexSE3* v_se3, g2o::VertexPointXYZ* v_xyz, const Eigen::Vector3d& xyz, const Eigen::MatrixXd& information_matrix) {
        g2o::EdgeSE3PointXYZ* edge(new g2o::EdgeSE3PointXYZ());
        edge->setMeasurement(xyz);
        edge->setInformation(information_matrix);
        edge->vertices()[0] = v_se3;
        edge->vertices()[1] = v_xyz;
        _graph->addEdge(edge);

        return edge;
    }
    g2o::EdgeSE3PriorXY* add_se3_prior_xy_edge(g2o::VertexSE3* v_se3, const Eigen::Vector2d& xy, const Eigen::MatrixXd& information_matrix) {
        g2o::EdgeSE3PriorXY* edge(new g2o::EdgeSE3PriorXY());
        edge->setMeasurement(xy);
        edge->setInformation(information_matrix);
        edge->vertices()[0] = v_se3;
        _graph->addEdge(edge);

        return edge;
    }
    g2o::EdgeSE3PriorXYZ* add_se3_prior_xyz_edge(g2o::VertexSE3* v_se3, const Eigen::Vector3d& xyz, const Eigen::MatrixXd& information_matrix) {
        g2o::EdgeSE3PriorXYZ* edge(new g2o::EdgeSE3PriorXYZ());
        edge->setMeasurement(xyz);
        edge->setInformation(information_matrix);
        edge->vertices()[0] = v_se3;
        _graph->addEdge(edge);

        return edge;
    }
    void optimize() {
        if(_graph->edges().size() < 10){
            return;
        }
        std::cout << std::endl;
        std::cout << "--- pose graph optimization ---" << std::endl;
        std::cout << "nodes: " << _graph->vertices().size() << "   edges: " << _graph->edges().size() << std::endl;
        std::cout << "optimizing... " << std::flush;

        double chi2 = _graph->chi2();
        _graph->initializeOptimization();
        _graph->setVerbose(false);
        auto t1 = ros::Time::now();
        int iterations = _graph->optimize(1024);
        auto t2 = ros::Time::now();

        std::cout << "done" << std::endl;
        std::cout << "iterations: " << iterations << std::endl;
        std::cout << "chi2: (before)" << chi2 << " -> (after)" << _graph->chi2() << std::endl;
        std::cout << "time: " << boost::format("%.3f") % (t2 - t1).toSec() << "[sec]" << std::endl;
    }
    ~GraphSlam(){
        _graph.reset();
    }
public:
    std::unique_ptr<g2o::SparseOptimizer> _graph;  // graph                  
    g2o::VertexPlane* _floor_plane_node;           // ground floor plane node
};


#endif