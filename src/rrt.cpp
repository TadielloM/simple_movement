#include "rrt.hh"

nav_msgs::Path RRT::findTrajectory(std::shared_ptr<octomap::OcTree> otree, std::shared_ptr<point_rtree> octomap_rtree, value_rtree* rrt_rtree,const Eigen::Vector4d& current_state,const Eigen::Vector4d& state_to_reach){

    //Defining the path message and set the first element to actual position
    nav_msgs::Path path;
    geometry_msgs::PoseStamped start;
        start.pose.position.x = current_state[0];
        start.pose.position.y = current_state[1];
        start.pose.position.z = current_state[2];
        Eigen::Quaternion<double> q;
        Eigen::Vector3d init(1.0, 0.0, 0.0);
        // Zero out rotation along
        // x and y axis so only
        // yaw is kept
        Eigen::Vector3d dir(0,0,0);
        q.setFromTwoVectors(init, dir);

        start.pose.orientation.x = q.x();
        start.pose.orientation.y = q.y();
        start.pose.orientation.z = q.z();
        start.pose.orientation.w = q.w();

    path.poses.push_back(start);

    Eigen::Vector3d point_to_reach(state_to_reach[0], state_to_reach[1], state_to_reach[2]);
    Eigen::Vector3d current_point(current_state[0], current_state[1], current_state[2]);
    // std::cout<<"Point to reach: "<<point_to_reach<<std::endl;
    // std::cout<<"Current point: "<<current_state<<std::endl;

    //Check if Octree is empty, don't move
    if(otree == NULL){
      std::cout<<"Octree empty\n";
      return path;
    }
      
    
    //If point is unreacheable return, don't move
    if (!otree->search(octomap::point3d(state_to_reach[0], state_to_reach[1], state_to_reach[2]))){
        // std::cout<<"Point unreacheable\n";
        return path;
    }
        

    //Check if it is possible to reach the point in a single segment
    std::shared_ptr<RRT> goal = std::make_shared<RRT>();
    goal->state = state_to_reach;
    // std::cout <<"The distance between the two point is "<<(point_to_reach - current_point).norm()<<" meters\n";
    // if((point_to_reach - current_point).norm() < extension_range){
      // std::cout<<"It's possibile to reach the point in one step\n";
      if (!collisionLine(octomap_rtree, current_state, state_to_reach, collision_radius)){
        std::shared_ptr<RRT> new_node = std::make_shared<RRT>();
        new_node->state = state;
        std::shared_ptr<RRT> nearest = chooseParent(*rrt_rtree,octomap_rtree,new_node);
        goal->parent = nearest;
        nearest->children.push_back(goal);

        rrt_rtree->insert(std::make_pair(point3d(state_to_reach[0], state_to_reach[1], state_to_reach[2]),goal));
        std::shared_ptr<RRT> n = goal;

        geometry_msgs::PoseStamped p;
        p.pose.position.x = goal->state[0];
        p.pose.position.y = goal->state[1];
        p.pose.position.z = goal->state[2];
        Eigen::Quaternion<double> q;
        Eigen::Vector3d init(1.0, 0.0, 0.0);
        // Zero out rotation along
        // x and y axis so only
        // yaw is kept

        Eigen::Vector3d dir(goal->state[0] - goal->parent->state[0], goal->state[1] - goal->parent->state[1], 0);
        q.setFromTwoVectors(init, dir);

        p.pose.orientation.x = q.x();
        p.pose.orientation.y = q.y();
        p.pose.orientation.z = q.z();
        p.pose.orientation.w = q.w();

        path.poses.push_back(p);

        // std::cout<<"Path: "<<path<<std::endl;
        return path;
      }
    // }

    //Build the RRT to reach the goal point
    // bool goal_reached = false;
    // while(!goal_reached){
    //     std::cout<<"State to reah: "<<state_to_reach<<"\nCurrent state: "<<current_state<<std::endl;
    //     std::cout<<"Need more then one step to reach the goal\n";
    //     std::shared_ptr<RRT> new_node = std::make_shared<RRT>();
    //     std::shared_ptr<RRT> nearest;
    //     octomap::OcTreeNode* ot_result;
    //     std::cout<<"a\n";
    //     do{
    //             std::cout<<"1\n";

    //             Eigen::Vector4d offset = sampleNewPoint();
    //                             std::cout<<"2\n";

    //             new_node->state = current_state + offset;
    //             nearest = chooseParent(*rrt_rtree,octomap_rtree,new_node);
    //                             std::cout<<"3\n";

    //             new_node->state = restrictDistance(nearest->state, new_node->state);
    //                             std::cout<<"4\n";

    //             ot_result = otree->search(octomap::point3d(new_node->state[0], new_node->state[1], new_node->state[2]));
    //                             std::cout<<"5\n";

    //             if (ot_result == NULL){
    //                 std::cout<<"6\n";

    //                 continue;
    //             }            
    //                             std::cout<<"7\n";

    //     }while(!isInsideBoundaries(new_node->state) or !ot_result or collisionLine(octomap_rtree, nearest->state, new_node->state, collision_radius));
    //     new_node->parent = nearest;
    //     nearest->children.push_back(new_node);
    //     rrt_rtree->insert(std::make_pair(point3d(new_node->state[0], new_node->state[1], new_node->state[2]), new_node));

    //     Eigen::Vector3d point_new(new_node->state[0], new_node->state[1], new_node->state[2]);
        
    //     std::cout <<"The distance between the two point is "<<(point_to_reach - point_new).norm()<<" meters\n";
    //     if((point_to_reach - point_new).norm() < extension_range){
    //         if (!collisionLine(octomap_rtree, current_state, state_to_reach, collision_radius)){
    //             goal->parent = new_node;
    //             new_node->children.push_back(goal);
                
    //             rrt_rtree->insert(std::make_pair(point3d(state_to_reach[0], state_to_reach[1], state_to_reach[2]),goal));
    //             std::shared_ptr<RRT> n = goal;
    //             for (int id = 0; n->parent; ++id)
    //             {
    //                 geometry_msgs::PoseStamped p;
    //                 p.pose.position.x = n->state[0];
    //                 p.pose.position.y = n->state[1];
    //                 p.pose.position.z = n->state[2];
    //                 Eigen::Quaternion<double> q;
    //                 Eigen::Vector3d init(1.0, 0.0, 0.0);
    //                 // Zero out rotation along
    //                 // x and y axis so only
    //                 // yaw is kept
    //                 Eigen::Vector3d dir(n->state[0] - n->parent->state[0], n->state[1] - n->parent->state[1], 0);
    //                 q.setFromTwoVectors(init, dir);

    //                 p.pose.orientation.x = q.x();
    //                 p.pose.orientation.y = q.y();
    //                 p.pose.orientation.z = q.z();
    //                 p.pose.orientation.w = q.w();

    //                 path.poses.push_back(p);
    //             }

    //             goal_reached=true;
    //         }
    //     }
    // }

    return path;
}
    

Eigen::Vector4d RRT::sampleNewPoint()
{
  // Samples one point uniformly over a sphere with a radius of max_sampling_radius
  Eigen::Vector4d point(0.0, 0.0, 0.0, 0.0);
  do
  {
    for (int i = 0; i < 3; i++)
      point[i] = max_sampling_radius * 2.0 * (((double)rand()) / ((double)RAND_MAX) - 0.5);
  } while (point.squaredNorm() > (max_sampling_radius*max_sampling_radius));

  return point;
}

std::shared_ptr<RRT> RRT::chooseParent(const value_rtree& rrt_rtree, std::shared_ptr<point_rtree> octomap_rtree,
                                                    std::shared_ptr<RRT> node)
{
  // Find nearest neighbour
  std::vector<value> nearest;
  rrt_rtree.query(bgi::nearest(point3d(node->state[0], node->state[1], node->state[2]), 15),
              std::back_inserter(nearest));
  
  //return the RRT_Node of the first element of the vector nearest 
  return nearest.front().second;
}

Eigen::Vector4d RRT::restrictDistance(Eigen::Vector4d nearest, Eigen::Vector4d new_pos)
{
  // Check for collision
  Eigen::Vector3d origin(nearest[0], nearest[1], nearest[2]);
  Eigen::Vector3d direction(new_pos[0] - origin[0], new_pos[1] - origin[1], new_pos[2] - origin[2]);
  if (direction.norm() > extension_range)
    direction = extension_range * direction.normalized();

  new_pos[0] = origin[0] + direction[0];
  new_pos[1] = origin[1] + direction[1];
  new_pos[2] = origin[2] + direction[2];

  return new_pos;
}

point_rtree RRT::getRtree(std::shared_ptr<octomap::OcTree> ot, octomap::point3d min, octomap::point3d max)
{
  point_rtree octomap_rtree;
  for (octomap::OcTree::leaf_bbx_iterator it = ot->begin_leafs_bbx(min, max), it_end = ot->end_leafs_bbx();
       it != it_end; ++it)
  {
    if (it->getLogOdds() > 0)
    {
      octomap_rtree.insert(point3d(it.getX(), it.getY(), it.getZ()));
    }
  }

  return octomap_rtree;
}

bool RRT::isInsideBoundaries(Eigen::Vector4d point)
{
  return point[0] > boundary_min[0] and point[0] < boundary_max[0] and
         point[1] > boundary_min[1] and point[1] < boundary_max[1] and
         point[2] > boundary_min[2] and point[2] < boundary_max[2];
}

bool RRT::collisionLine(std::shared_ptr<point_rtree> octomap_rtree, Eigen::Vector4d p1, Eigen::Vector4d p2,float r)
{
  octomap::point3d start(p1[0], p1[1], p1[2]);
  octomap::point3d end(p2[0], p2[1], p2[2]);

  point3d bbx_min(std::min(p1[0], p2[0]) - r, std::min(p1[1], p2[1]) - r, std::min(p1[2], p2[2]) - r);
  point3d bbx_max(std::max(p1[0], p2[0]) + r, std::max(p1[1], p2[1]) + r, std::max(p1[2], p2[2]) + r);

  box query_box(bbx_min, bbx_max);
  std::vector<point3d> hits;
  octomap_rtree->query(bgi::intersects(query_box), std::back_inserter(hits));

  double lsq = (end - start).norm_sq();
  double rsq = r * r;

  for (size_t i = 0; i < hits.size(); ++i)
  {
    octomap::point3d pt(hits[i].get<0>(), hits[i].get<1>(), hits[i].get<2>());

    if (CylTest_CapsFirst(start, end, lsq, rsq, pt) > 0 or (end - pt).norm() < r)
    {
      return true;
    }
  }

  return false;
}

//-----------------------------------------------------------------------------
// Name: CylTest_CapsFirst
// Orig: Greg James - gjames@NVIDIA.com
// Lisc: Free code - no warranty & no money back.  Use it all you want
// Desc:
//    This function tests if the 3D point 'pt' lies within an arbitrarily
// oriented cylinder.  The cylinder is defined by an axis from 'pt1' to 'pt2',
// the axis having a length squared of 'lsq' (pre-compute for each cylinder
// to avoid repeated work!), and radius squared of 'rsq'.
//    The function tests against the end caps first, which is cheap -> only
// a single dot product to test against the parallel cylinder caps.  If the
// point is within these, more work is done to find the distance of the point
// from the cylinder axis.
//    Fancy Math (TM) makes the whole test possible with only two dot-products
// a subtract, and two multiplies.  For clarity, the 2nd mult is kept as a
// divide.  It might be faster to change this to a mult by also passing in
// 1/lengthsq and using that instead.
//    Elminiate the first 3 subtracts by specifying the cylinder as a base
// point on one end cap and a vector to the other end cap (pass in {dx,dy,dz}
// instead of 'pt2' ).
//
//    The dot product is constant along a plane perpendicular to a vector.
//    The magnitude of the cross product divided by one vector length is
// constant along a cylinder surface defined by the other vector as axis.
//
// Return:  -1.0 if point is outside the cylinder
// Return:  distance squared from cylinder axis if point is inside.
//
//-----------------------------------------------------------------------------
float RRT::CylTest_CapsFirst(const octomap::point3d& pt1, const octomap::point3d& pt2, float lsq, float rsq, const octomap::point3d& pt)
{
  float dx, dy, dz;     // vector d  from line segment point 1 to point 2
  float pdx, pdy, pdz;  // vector pd from point 1 to test point
  float dot, dsq;

  dx = pt2.x() - pt1.x();  // translate so pt1 is origin.  Make vector from
  dy = pt2.y() - pt1.y();  // pt1 to pt2.  Need for this is easily eliminated
  dz = pt2.z() - pt1.z();

  pdx = pt.x() - pt1.x();  // vector from pt1 to test point.
  pdy = pt.y() - pt1.y();
  pdz = pt.z() - pt1.z();

  // Dot the d and pd vectors to see if point lies behind the
  // cylinder cap at pt1.x, pt1.y, pt1.z

  dot = pdx * dx + pdy * dy + pdz * dz;

  // If dot is less than zero the point is behind the pt1 cap.
  // If greater than the cylinder axis line segment length squared
  // then the point is outside the other end cap at pt2.

  if (dot < 0.0f || dot > lsq)
    return (-1.0f);
  else
  {
    // Point lies within the parallel caps, so find
    // distance squared from point to line, using the fact that sin^2 + cos^2 = 1
    // the dot = cos() * |d||pd|, and cross*cross = sin^2 * |d|^2 * |pd|^2
    // Carefull: '*' means mult for scalars and dotproduct for vectors
    // In short, where dist is pt distance to cyl axis:
    // dist = sin( pd to d ) * |pd|
    // distsq = dsq = (1 - cos^2( pd to d)) * |pd|^2
    // dsq = ( 1 - (pd * d)^2 / (|pd|^2 * |d|^2) ) * |pd|^2
    // dsq = pd * pd - dot * dot / lengthsq
    //  where lengthsq is d*d or |d|^2 that is passed into this function

    // distance squared to the cylinder axis:

    dsq = (pdx * pdx + pdy * pdy + pdz * pdz) - dot * dot / lsq;

    if (dsq > rsq)
      return (-1.0f);
    else
      return (dsq);  // return distance squared to axis
  }
}
