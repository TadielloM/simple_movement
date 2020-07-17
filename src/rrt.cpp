include "rrt.h"

bool RRT::findTrajectory(std::shared_ptr<octomap::OcTree> otree, const Eigen::Vector4d& current_state,const Eigen::Vector4d& state_to_reach){
    std::shared_ptr<RRTNode> new_node = std::make_shared<RRTNode>();
    std::shared_ptr<RRTNode> nearest;

    do{
            Eigen::Vector4d offset = sampleNewPoint();
            new_node->state = current_state + offset;
            nearest = chooseParent(new_node);
    }while(!isInsideBoundaries(new_node ))
}
    

Eigen::Vector4d RRT::sampleNewPoint()
{
  // Samples one point uniformly over a sphere with a radius of max_sampling_radius
  Eigen::Vector4d point(0.0, 0.0, 0.0, 0.0);
  do
  {
    for (int i = 0; i < 3; i++)
      point[i] = max_sampling_radius * 2.0 * (((double)rand()) / ((double)RAND_MAX) - 0.5);
  } while (point.squaredNorm() > max_sampling_radius_squared_);

  return point;
}

std::shared_ptr<RRTNode> STLAEPlanner::chooseParent(const value_rtree& rrt, std::shared_ptr<point_rtree> stl_rtree,
                                                    std::shared_ptr<RRTNode> node, double l)
{
  // Find nearest neighbour
  std::vector<value> nearest;
  point3d bbx_min(node->state_[0] - (2 * l), node->state_[1] - (2 * l), node->state_[2] - (2 * l));
  point3d bbx_max(node->state_[0] + (2 * l), node->state_[1] + (2 * l), node->state_[2] + (2 * l));
  box query_box(bbx_min, bbx_max);
  rtree.query(boost::geometry::index::intersects(query_box), std::back_inserter(nearest));

  if (nearest.empty())
  {
    rtree.query(boost::geometry::index::nearest(point(node->state_[0], node->state_[1], node->state_[2]), 15),
              std::back_inserter(nearest));
  }

  // TODO: Check if correct
  std::shared_ptr<RRTNode> best_node;
  double best_cost;

  for (value item : nearest)
  {
    std::shared_ptr<RRTNode> current_node = item.second;

    double current_cost = current_node->cost(
        stl_rtree, ltl_lambda_, ltl_min_distance_, ltl_max_distance_, ltl_min_distance_active_,
        ltl_max_distance_active_, ltl_max_search_distance_, params_.bounding_radius, ltl_step_size_, ltl_routers_,
        ltl_routers_active_, ltl_min_altitude_, ltl_max_altitude_, ltl_min_altitude_active_, ltl_max_altitude_active_);

    if (!best_node || current_cost < best_cost)
    {
      best_node = current_node;
      best_cost = current_cost;
    }
  }

  return best_node;
}

point_rtree STLAEPlanner::getRtree(std::shared_ptr<octomap::OcTree> ot, octomap::point3d min, octomap::point3d max)
{
  point_rtree octomap_rtree;
  for (octomap::OcTree::leaf_bbx_iterator it = ot->begin_leafs_bbx(min, max), it_end = ot->end_leafs_bbx();
       it != it_end; ++it)
  {
    if (it->getLogOdds() > 0)
    {
      octomap_rtree.insert(point(it.getX(), it.getY(), it.getZ()));
    }
  }

  return octomap_rtree;
}