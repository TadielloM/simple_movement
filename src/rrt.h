#ifndef RRT_H
#define RRT_H

#include <octomap/OcTree.h>
#include <ros/ros.h>
#include <boost/geometry.hpp>
#include <boost/geometry/index/rtree.hpp>

//Definition of types used with the boost library
//Examples avilable here: https://www.boost.org/doc/libs/1_73_0/libs/geometry/doc/html/geometry/spatial_indexes/rtree_quickstart.html
namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;
typedef bg::model::point<float, 3, bg::cs::cartesian> point3d;
typedef bg::model::box<point3d> box;
typedef std::pair<point3d, std::shared_ptr<RRTNode>> value;
typedef bgi::rtree<value, bgi::rstar<16>> value_rtree;     // Used to store the rrt rtree
typedef bgi::rtree<point3d, bgi::rstar<16>> point_rtree; //Used to store octomap rtree


class RRT : public std::enable_shared_from_this<RRT>
{   
    private:
        float max_sampling_radius = 2; //maximum sampling radius for new point in the RRT
        double extension_range = 2.5;
    public:
        Eigen::Vector4d state_;
        std::shared_ptr<RRT> parent_;
        std::vector<std::shared_ptr<RRT>> children_;

    RRTNode() : parent_(NULL)
    {
    }

    double distance(std::shared_ptr<RRTNode> other)
    {
        Eigen::Vector3d p3(this->state_[0], this->state_[1], this->state_[2]);
        Eigen::Vector3d q3(other->state_[0], other->state_[1], other->state_[2]);
        return (p3 - q3).norm();
    }
    
    //Find the trajectory from the root of the RRT to the position to reach
    void findTrajectory(std::shared_ptr<octomap::OcTree> otree, const Eigen::Vector4d& current_state, const Eigen::Vector4d& state_to_reach);

    //Sample new point for the RRT
    Eigen::Vector4d STLAEPlanner::sampleNewPoint();

    std::shared_ptr<RRTNode> STLAEPlanner::chooseParent(std::shared_ptr<RRTNode> node, double extension_range);

};

#endif