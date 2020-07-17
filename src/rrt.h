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
        std::vector<double> boundary_min (3,-100);
        std::vector<double> boundary_max (3,100);
        float max_sampling_radius = 2; //maximum sampling radius for new point in the RRT
        float extension_range = 2.5;   //Maximum extension of a branch of the rrt
        float collision_radius = 0.5; //The radius in which search collision should be a little be higher than the dimension of the robot
    public:
        Eigen::Vector4d state;
        std::shared_ptr<RRT> parent_;
        std::vector<std::shared_ptr<RRT>> children_;

    RRTNode() : parent_(NULL)
    {
    }

    double distance(std::shared_ptr<RRTNode> other)
    {
        Eigen::Vector3d p3(this->state[0], this->state[1], this->state[2]);
        Eigen::Vector3d q3(other->state[0], other->state[1], other->state[2]);
        return (p3 - q3).norm();
    }
    
    //Find the trajectory from the root of the RRT to the position to reach
    void findTrajectory(std::shared_ptr<octomap::OcTree> otree, std::shared_ptr<point_rtree> octomap_rtree, value_rtree* rrt_rtree, const Eigen::Vector4d& current_state, const Eigen::Vector4d& state_to_reach);

    //Sample new point for the RRT
    Eigen::Vector4d STLAEPlanner::sampleNewPoint();

    std::shared_ptr<RRTNode> STLAEPlanner::chooseParent(std::shared_ptr<RRTNode> node, double extension_range);
    Eigen::Vector4d RRT::restrictDistance(Eigen::Vector4d nearest, Eigen::Vector4d new_pos);
    point_rtree RRT::getRtree(std::shared_ptr<octomap::OcTree> ot, octomap::point3d min, octomap::point3d max);
    bool RRT::isInsideBoundaries(Eigen::Vector4d point);
    bool RRT::collisionLine(std::shared_ptr<point_rtree> octomap_rtree, Eigen::Vector4d p1, Eigen::Vector4d p2,double r);
    float STLAEPlanner::CylTest_CapsFirst(const octomap::point3d& pt1, const octomap::point3d& pt2, float lsq, float rsq, const octomap::point3d& pt);

};

#endif