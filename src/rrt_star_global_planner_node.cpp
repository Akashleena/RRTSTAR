/* © Copyright Newspace Research and Technologies. All rights reserved.
 *  Any permission to use it shall be granted in writing. Requests shall be addressed to CAT, NRT
 *
 * Author: Akashleena Sarkar <akashleena.s@newspace.co.in>
 * 
 *==================================================================================================
*/

#include <rrt_star_global_planner_node.hpp>

#include <nav_msgs/OccupancyGrid.h>
#include <iostream>
#include <random>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_msgs/Path.h> //to use nav_msgs/path
using namespace std;
std::random_device rd;
static std::default_random_engine generator(rd());

namespace RRTstar_planner
{

  // RRTstarPlannerROS::RRTstarPlannerROS()
  //         : costmap_(nullptr), initialized_(false) {}

  RRTstarPlannerROS::RRTstarPlannerROS(const std::string &name)
  {
    //subscribe to voxel grid and pass to costmap
    cout << "Inside subscriber function";
    RRTstarPlannerROS::globalcostmap_sub = nh.subscribe("/move_base/global_costmap/costmap", 100, &RRTstarPlannerROS::gbcostmapcb, this);
    //initialize the planner
    cout << "Subscribed to global costmap";
    // costmap_2d::Costmap2DROS *costmap_ros;
    // initialize(name, costmap_ros);
  }

  void RRTstarPlannerROS::gbcostmapcb(const nav_msgs::OccupancyGrid &msg)
  {

    // costmap_2d::Costmap2DROS *costmap_ros = new costmap_2d::Costmap2D(msg.info.width, msg.info.height, msg.info.resolution, msg.info.origin.position.x, msg.info.origin.position.y);
    cout << "\n message info ";
    // cout << "\n width" << msg.info.width;
    // cout << "\n height" << msg.info.height;
    // cout << "\n resolution" << msg.info.resolution;
    // cout << "\n origin" << msg.info.origin.position.x << " " << msg.info.origin.position.y << endl;
    // costmap_ = costmap_ros->getCostmap();
    // std::cout << "global costmap callback !! " << std::endl;
  }

  void RRTstarPlannerROS::initialize(std::string name, costmap_2d::Costmap2DROS *costmap_ros)
  {

    if (!initialized_)
    {
      // Initialize map
      costmap_ros_ = costmap_ros;           //initialize the costmap_ros_ attribute to the parameter
      costmap_ = costmap_ros->getCostmap(); //get the costmap_ from costmap_ros_

      ros::NodeHandle private_nh("~/" + name);

      originX = costmap_->getOriginX();

      cout << "Origin x" << originX << endl; //0.016-0.027
      originY = costmap_->getOriginY();
      cout << "Origin y" << originY << endl; //0.00319-0.131
      width = costmap_->getSizeInCellsX();   //10,000
      height = costmap_->getSizeInCellsY();  //10,000
      cout << "width" << width << endl;
      cout << "height" << height << endl;
      resolution = costmap_->getResolution(); //0.05m/pixel
      cout << "Resolution of costmap" << resolution << endl;

      RADIUS = 0.3;
      GOAL_RADIUS = 0.5;
      epsilon_min = 0.05;
      epsilon_max = 0.1;

      ROS_INFO("RRT* planner initialized successfully");
      initialized_ = true;
      //makePlan(start, goal, plan);
    }
    else
      ROS_WARN("This planner has already been initialized... doing nothing");
  }

  bool RRTstarPlannerROS::makePlan(const geometry_msgs::PoseStamped &start,
                                   const geometry_msgs::PoseStamped &goal,
                                   std::vector<geometry_msgs::PoseStamped> &plan)
  {
    //clear the plan, just in case
    plan.clear();

    std::vector<std::pair<float, float>> path;
    ros::NodeHandle n;
    std::string global_frame = frame_id_;

    std::vector<Node> nodes;

    MAX_NUM_NODES = 30000;

    Node start_node;
    start_node.x = start.pose.position.x;
    start_node.y = start.pose.position.y;
    start_node.node_id = 0;
    start_node.parent_id = -1; // None parent node
    start_node.cost = 0.0;
    cout << "Start node:"
         << "x" << start_node.x << endl
         << "y:" << start_node.y << endl;
    nodes.push_back(start_node);

    std::pair<float, float> p_rand;
    std::pair<float, float> p_new;

    Node node_nearest;
    while (nodes.size() < MAX_NUM_NODES)
    {
      bool found_next = false;
      while (found_next == false)
      {
        p_rand = sampleFree();                                                      // random point in the free space
        node_nearest = getNearest(nodes, p_rand);                                   // The nearest node of the random point
        p_new = steer(node_nearest.x, node_nearest.y, p_rand.first, p_rand.second); // new point and node candidate.
        if (obstacleFree(node_nearest, p_new.first, p_new.second))
        {
          Node newnode;
          newnode.x = p_new.first;
          newnode.y = p_new.second;
          newnode.node_id = nodes.size(); // index of the last element after the push_bask below
          newnode.parent_id = node_nearest.node_id;
          newnode.cost = 0.0;

          // Optimize
          newnode = chooseParent(node_nearest, newnode, nodes); // Select the best parent
          nodes.push_back(newnode);
          nodes = rewire(nodes, newnode);
          found_next = true;
        }
      }
      // Check if the distance between the goal and the new node is less than
      // the GOAL_RADIUS
      if (pointCircleCollision(p_new.first, p_new.second, goal.pose.position.x, goal.pose.position.y, GOAL_RADIUS) && nodes.size() > 10000)
      {
        ROS_INFO("RRT* Global Planner: Path found!!!!");
        std::pair<float, float> point;

        // New goal inside of the goal tolerance
        Node new_goal_node = nodes[nodes.size() - 1];
        Node current_node = new_goal_node;

        current_node = new_goal_node;
        // Final Path
        while (current_node.parent_id != -1)
        {
          point.first = current_node.x;
          point.second = current_node.y;
          path.insert(path.begin(), point);

          current_node = nodes[current_node.parent_id];
        }
        //std::cout << "Path size: " << path.size() << std::endl;

        //if the global planner find a path
        if (path.size() > 0)
        {
          plan.push_back(start);
          //ros::Time plan_time = ros::Time::now();
          ros::Time plan_time = ros::Time::now();
          // convert the points to poses
          for (int i = 0; i < path.size(); i++)
          {
            //std::cout << path[i].first << " " << path[i].second << std::endl;
            geometry_msgs::PoseStamped pose;
            pose.header.stamp = plan_time;
            pose.header.frame_id = "/map";
            pose.pose.position.x = path[i].first;
            pose.pose.position.y = path[i].second;
            pose.pose.position.z = 0.0;
            pose.pose.orientation.x = 0.0;
            pose.pose.orientation.y = 0.0;
            pose.pose.orientation.z = 0.0;
            pose.pose.orientation.w = 1.0;
            plan.push_back(pose);
            navmsgpath.header.stamp = plan_time;
            navmsgpath.header.frame_id = "/map";
            navmsgpath.poses.push_back(pose);
          }
          cout << "Publishing to rrt star global plan topic ";
          ros::Publisher global_pub = nh.advertise<nav_msgs::Path>("/rrt_star_global_plan", 1000);
          global_pub.publish(navmsgpath);
          return true;
        }
        else
        {
          ROS_WARN("The planner failed to find a path, choose other goal position");
          return false;
        }
      }
    }
    ROS_WARN("The planner failed to find a path, choose other goal position");
    return false;
  }

  bool RRTstarPlannerROS::pointCircleCollision(float x1, float y1, float x2, float y2, float radius)
  {
    float dist = distance(x1, y1, x2, y2);
    if (dist < radius)
      return true;
    else
      return false;
  }

  float RRTstarPlannerROS::distance(float px1, float py1, float px2, float py2)
  {
    float dist = sqrt((px1 - px2) * (px1 - px2) + (py1 - py2) * (py1 - py2));
    return dist;
  }

  std::pair<float, float> RRTstarPlannerROS::sampleFree()
  {
    std::pair<float, float> random_point;
    for (int i = 0; i < 50000; i++)
    {
      // generate random x and y coords within map bounds

      std::random_device rd;
      std::mt19937 gen(rd());
      //float map_width = costmap_->getSizeInMetersX();
      //float map_height = costmap_->getSizeInMetersY();

      // Using the clearpath Husky World I know that the dimensions are
      float map_width = 100;
      float map_height = 100;
      std::uniform_real_distribution<> x(-map_width, map_width);
      std::uniform_real_distribution<> y(-map_height, map_height);

      random_point.first = x(gen);
      random_point.second = y(gen);

      if (!collision(random_point.first, random_point.second))
        return random_point;
      //TODO check collision
      //TODO check collision
      //TODO check collision
      //TODO check collision
      //TODO check collision
      //TODO check collision
    }
    return random_point;
  }

  void RRTstarPlannerROS::mapToWorld(int mx, int my, float &wx, float &wy)
  {
    wx = costmap_->getOriginX() + mx * costmap_->getResolution();
    wy = costmap_->getOriginY() + my * costmap_->getResolution();
  }

  void RRTstarPlannerROS::worldToMap(float wx, float wy, int &mx, int &my)
  {
    float origin_x = costmap_->getOriginX(), origin_y = costmap_->getOriginY();
    float resolution = costmap_->getResolution();

    mx = (wx - origin_x) / resolution;
    my = (wy - origin_y) / resolution;
  }

  //check if point collides with the obstacle
  bool RRTstarPlannerROS::collision(float wx, float wy)
  {
    int mx, my;
    worldToMap(wx, wy, mx, my);

    if ((mx < 0) || (my < 0) || (mx >= costmap_->getSizeInCellsX()) || (my >= costmap_->getSizeInCellsY())) //going outside map
      return true;

    // grid[row][column] = vector[row*WIDTH + column]
    //if (costmap_[my*width + mx] > 0)
    //  return true;

    unsigned int cost = static_cast<int>(costmap_->getCost(mx, my));
    if (cost > 0) //freespace cost==0
      return true;

    return false;
  }

  Node RRTstarPlannerROS::getNearest(std::vector<Node> nodes, std::pair<float, float> p_rand)
  {
    Node node = nodes[0];
    for (int i = 1; i < nodes.size(); i++)
    {
      if (distance(nodes[i].x, nodes[i].y, p_rand.first, p_rand.second) < distance(node.x, node.y, p_rand.first, p_rand.second))
        node = nodes[i];
    }

    return node;
  }

  Node RRTstarPlannerROS::chooseParent(Node nn, Node newnode, std::vector<Node> nodes)
  {

    for (int i = 0; i < nodes.size(); i++)
    {
      if (distance(nodes[i].x, nodes[i].y, newnode.x, newnode.y) < RADIUS &&
          nodes[i].cost + distance(nodes[i].x, nodes[i].y, newnode.x, newnode.y) < nn.cost + distance(nn.x, nn.y, newnode.x, newnode.y) &&
          obstacleFree(nodes[i], nn.x, nn.y))
      {
        nn = nodes[i];
      }
    }
    newnode.cost = nn.cost + distance(nn.x, nn.y, newnode.x, newnode.y);
    newnode.parent_id = nn.node_id;

    return newnode;
  }

  std::vector<Node> RRTstarPlannerROS::rewire(std::vector<Node> nodes, Node newnode)
  {
    Node node;
    for (int i = 0; i < nodes.size(); i++)
    {
      node = nodes[i];
      if (node != nodes[newnode.parent_id] && distance(node.x, node.y, newnode.x, newnode.y) < RADIUS &&
          newnode.cost + distance(node.x, node.y, newnode.x, newnode.y) < node.cost && obstacleFree(node, newnode.x, newnode.y))
      {
        node.parent_id = newnode.node_id;
        node.cost = newnode.cost + distance(node.x, node.y, newnode.x, newnode.y);
      }
    }
    return nodes;
  }

  std::pair<float, float> RRTstarPlannerROS::steer(float x1, float y1, float x2, float y2)
  {
    std::pair<float, float> p_new;
    float dist = distance(x1, y1, x2, y2);
    if (dist < epsilon_max && dist > epsilon_min)
    {
      p_new.first = x1;
      p_new.second = y1;
      return p_new;
    }
    else
    {
      float theta = atan2(y2 - y1, x2 - x1);
      p_new.first = x1 + epsilon_max * cos(theta);
      p_new.second = y1 + epsilon_max * sin(theta);
      return p_new;
    }
  }

  bool RRTstarPlannerROS::obstacleFree(Node node_nearest, float px, float py)
  {
    int n = 1;
    float theta;

    std::pair<float, float> p_n;
    p_n.first = 0.0;
    p_n.second = 0.0;

    float dist = distance(node_nearest.x, node_nearest.y, px, py);
    if (dist < resolution)
    {
      if (collision(px, py))
        return false;
      else
        return true;
    }
    else
    {
      int value = int(floor(dist / resolution));
      float theta;
      for (int i = 0; i < value; i++)
      {
        theta = atan2(node_nearest.y - py, node_nearest.x - px);
        p_n.first = node_nearest.x + n * resolution * cos(theta); //get co-ordinates of each pixel value b/w nearestnode and new node
        p_n.second = node_nearest.y + n * resolution * sin(theta);
        if (collision(p_n.first, p_n.second))
          return false;

        n++;
      }
      return true;
    }
  }

}; // RRTstar_planner namespace

int main(int argc, char **argv)
{

  ros::init(argc, argv, "rrt_star_global_planner"); //node name

  ROS_INFO_STREAM("inside main \n");
  //ros::Time::init();

  //RRTstar_planner::RRTstarPlannerROS rrtstar;
  //cout<<"Default constructor called";
  // RRTstar_planner::RRTstarPlannerROS rrtstar1("RRTstarPlannerROS");
  //RRTstar_planner::RRTstarPlannerROS *rrtstar = new RRTstar_planner::RRTstarPlannerROS("RRTstarPlannerROS");
  RRTstar_planner::RRTstarPlannerROS *rrtstar = new RRTstar_planner::RRTstarPlannerROS("RRTstarPlannerROS");
  ROS_INFO_STREAM("constructor called \n");
  std::vector<geometry_msgs::PoseStamped> plan;
  geometry_msgs::PoseStamped start;
  start.pose.position.x = 0.027;
  start.pose.position.y = 0.131;
  geometry_msgs::PoseStamped goal;
  goal.pose.position.x = 100;
  goal.pose.position.y = 100; //or subscribe to move_base_simple/goal
  // rrtstar->makePlan(start, goal, plan);
  //rrtros.RRTstarPlannerROS(rrtros.start, rrtros);
  //rrtros.makePlan();

  ros::spin();

  return 0;
}