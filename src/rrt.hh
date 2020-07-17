#ifndef RRT_HH
#define RRT_HH

#include <octomap/OcTree.h>
#include <ros/ros.h>
#include <boost/geometry.hpp>
#include <boost/geometry/index/rtree.hpp>
#include <nav_msgs/Path.h>
#include <eigen3/Eigen/Dense>


//Definition of types used with the boost library
//Examples avilable here: https://www.boost.org/doc/libs/1_73_0/libs/geometry/doc/html/geometry/spatial_indexes/rtree_quickstart.html
namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;
class RRT;
typedef bg::model::point<float, 3, bg::cs::cartesian> point3d;
typedef bg::model::box<point3d> box;
typedef std::pair<point3d, std::shared_ptr<RRT>> value;
typedef bgi::rtree<value, bgi::rstar<16>> value_rtree;     // Used to store the rrt rtree
typedef bgi::rtree<point3d, bgi::rstar<16>> point_rtree; //Used to store octomap rtree


class RRT : public std::enable_shared_from_this<RRT>
{   
    private:
        std::vector<float> boundary_min{std::vector<float>(3,-100)};
        std::vector<float> boundary_max{std::vector<float>(3,100)};
        float max_sampling_radius = 2; //maximum sampling radius for new point in the RRT
        float extension_range = 2.5;   //Maximum extension of a branch of the rrt
        float collision_radius = 0.5; //The radius in which search collision should be a little be higher than the dimension of the robot
    public:
        Eigen::Vector4d state;
        std::shared_ptr<RRT> parent;
        std::vector<std::shared_ptr<RRT>> children;

    RRT() : parent(NULL)
    {
    }

    //Find the trajectory from the root of the RRT to the position to reach
    nav_msgs::Path findTrajectory(std::shared_ptr<octomap::OcTree> otree, std::shared_ptr<point_rtree> octomap_rtree, value_rtree* rrt_rtree, const Eigen::Vector4d& current_state, const Eigen::Vector4d& state_to_reach);

    //Sample new point for the RRT
    Eigen::Vector4d sampleNewPoint();

    //Choose Parent of the New point
    std::shared_ptr<RRT> chooseParent(const value_rtree& rrt_rtree, std::shared_ptr<point_rtree> octomap_rtree,std::shared_ptr<RRT> node);
    
    Eigen::Vector4d restrictDistance(Eigen::Vector4d nearest, Eigen::Vector4d new_pos);
    
    //Get The Rtree from the octomap
    point_rtree getRtree(std::shared_ptr<octomap::OcTree> ot, octomap::point3d min, octomap::point3d max);
    
    //Check is inside boundaries of the bounding box
    bool isInsideBoundaries(Eigen::Vector4d point);
    
    //Check the collision in a certain segment
    bool collisionLine(std::shared_ptr<point_rtree> octomap_rtree, Eigen::Vector4d p1, Eigen::Vector4d p2,double r);
    float CylTest_CapsFirst(const octomap::point3d& pt1, const octomap::point3d& pt2, float lsq, float rsq, const octomap::point3d& pt);

};



#endif