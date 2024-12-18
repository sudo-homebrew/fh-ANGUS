#include <explore/costmap_tools.h>
#include <explore/frontier_search.h>

#include <geometry_msgs/msg/point.hpp>
#include <mutex>
#include <cmath>
#include <array>
#include <iostream>
#include <chrono>

#include "nav2_costmap_2d/cost_values.hpp"

// Below mecro functions are for Custom Heuristic
#define ALPHA 1.6
#define BETA 4.9
#define GAMMA 3.5
#define DELTA 5.0

// Below mecro functions are for GDAE Heuristic
#define L1 1.0
#define L2 5.0
#define K 1.0

namespace frontier_exploration
{
using nav2_costmap_2d::FREE_SPACE;
using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::NO_INFORMATION;


FrontierSearch::FrontierSearch(nav2_costmap_2d::Costmap2D* costmap,
                               double potential_scale, double gain_scale,
                               double min_frontier_size)
  : costmap_(costmap)
  , potential_scale_(potential_scale)
  , gain_scale_(gain_scale)
  , min_frontier_size_(min_frontier_size)
{
}

std::vector<Frontier>
FrontierSearch::searchFrom(geometry_msgs::msg::Point position)
{
  std::vector<Frontier> frontier_list;

  // Sanity check that robot is inside costmap bounds before searching
  unsigned int mx, my;
  if (!costmap_->worldToMap(position.x, position.y, mx, my)) {
    RCLCPP_ERROR(rclcpp::get_logger("FrontierSearch"), "Robot out of costmap "
                                                       "bounds, cannot search "
                                                       "for frontiers");
    return frontier_list;
  }

  // make sure map is consistent and locked for duration of search
  std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lock(
      *(costmap_->getMutex()));

  map_ = costmap_->getCharMap();
  size_x_ = costmap_->getSizeInCellsX();
  size_y_ = costmap_->getSizeInCellsY();

  
  // initialize flag arrays to keep track of visited and frontier cells
  std::vector<bool> frontier_flag(size_x_ * size_y_, false);
  std::vector<bool> visited_flag(size_x_ * size_y_, false);

  // initialize breadth first search
  std::queue<unsigned int> bfs;

  // find closest clear cell to start search
  unsigned int clear, pos = costmap_->getIndex(mx, my);
  if (nearestCell(clear, pos, FREE_SPACE, *costmap_)) {
    bfs.push(clear);
  } else {
    bfs.push(pos);
    RCLCPP_WARN(rclcpp::get_logger("FrontierSearch"), "Could not find nearby "
                                                      "clear cell to start "
                                                      "search");
  }
  visited_flag[bfs.front()] = true;

  while (!bfs.empty()) {
    unsigned int idx = bfs.front();
    bfs.pop();

    // iterate over 4-connected neighbourhood
    for (unsigned nbr : nhood4(idx, *costmap_)) {
      // add to queue all free, unvisited cells, use descending search in case
      // initialized on non-free cell
      if (map_[nbr] <= map_[idx] && !visited_flag[nbr]) {
        visited_flag[nbr] = true;
        bfs.push(nbr);
        // check if cell is new frontier cell (unvisited, NO_INFORMATION, free
        // neighbour)
      } else if (isNewFrontierCell(nbr, frontier_flag)) {
        frontier_flag[nbr] = true;
        Frontier new_frontier = buildNewFrontier(nbr, pos, frontier_flag);
        if (new_frontier.size * costmap_->getResolution() >=
            min_frontier_size_) {
          frontier_list.push_back(new_frontier);
        }
      }
    }
  }

  // set costs of frontiers
  for (auto& frontier : frontier_list) {
    frontier.cost = frontierCost(frontier);
  }
  std::sort(
      frontier_list.begin(), frontier_list.end(),
      [](const Frontier& f1, const Frontier& f2) { return f1.cost < f2.cost; });

  return frontier_list;
}

Frontier FrontierSearch::buildNewFrontier(unsigned int initial_cell,
                                          unsigned int reference,
                                          std::vector<bool>& frontier_flag)
{
  // initialize frontier structure
  Frontier output;
  output.centroid.x = 0;
  output.centroid.y = 0;
  output.size = 1;
  output.min_distance = std::numeric_limits<double>::infinity();

  // record initial contact point for frontier
  unsigned int ix, iy;
  costmap_->indexToCells(initial_cell, ix, iy);
  costmap_->mapToWorld(ix, iy, output.initial.x, output.initial.y);

  // push initial gridcell onto queue
  std::queue<unsigned int> bfs;
  std::vector<unsigned int> frontiers;
  bfs.push(initial_cell);

  // cache reference position in world coords
  unsigned int rx, ry;
  double reference_x, reference_y;
  costmap_->indexToCells(reference, rx, ry);
  costmap_->mapToWorld(rx, ry, reference_x, reference_y);

  while (!bfs.empty()) {
    unsigned int idx = bfs.front();
    bfs.pop();

    // try adding cells in 8-connected neighborhood to frontier
    for (unsigned int nbr : nhood8(idx, *costmap_)) {
      // check if neighbour is a potential frontier cell
      if (isNewFrontierCell(nbr, frontier_flag)) {
        // mark cell as frontier
        frontier_flag[nbr] = true;
        frontiers.push_back(nbr);
        unsigned int mx, my;
        double wx, wy;
        costmap_->indexToCells(nbr, mx, my);
        costmap_->mapToWorld(mx, my, wx, wy);

        geometry_msgs::msg::Point point;
        point.x = wx;
        point.y = wy;
        output.points.push_back(point);

        // update frontier size
        output.size++;

        // update centroid of frontier
        output.centroid.x += wx;
        output.centroid.y += wy;

        // determine frontier's distance from robot, going by closest gridcell
        // to robot
        double distance = sqrt(pow((double(reference_x) - double(wx)), 2.0) +
                               pow((double(reference_y) - double(wy)), 2.0));
        if (distance < output.min_distance) {
          output.min_distance = distance;
          output.middle.x = wx;
          output.middle.y = wy;
        }

        // add to queue for breadth first search
        bfs.push(nbr);
      }
    }
  }

  // average out frontier centroid
  output.centroid.x /= output.size;
  output.centroid.y /= output.size;

  output.radius = fardest_point_from_centroid(frontiers, output.centroid.x, output.centroid.y);

  return output;
}

bool FrontierSearch::isNewFrontierCell(unsigned int idx,
                                       const std::vector<bool>& frontier_flag)
{
  // check that cell is unknown and not already marked as frontier
  if (map_[idx] != NO_INFORMATION || frontier_flag[idx]) {
    return false;
  }

  // frontier cells should have at least one cell in 4-connected neighbourhood
  // that is free
  for (unsigned int nbr : nhood4(idx, *costmap_)) {
    if (map_[nbr] == FREE_SPACE) {
      return true;
    }
  }

  return false;
}

double FrontierSearch::fardest_point_from_centroid(std::vector<unsigned int> line, double centroid_x, double centroid_y){
  double dist = 0;
  for(unsigned int p : line){
    unsigned int rx, ry;
    double reference_x, reference_y;
    costmap_->indexToCells(p, rx, ry);
    costmap_->mapToWorld(rx, ry, reference_x, reference_y);
    double dist_tmp = sqrt(pow((reference_x - centroid_x), 2) + pow((reference_y - centroid_y), 2));

    if(dist_tmp > dist){
      dist = dist_tmp;
    }
  }
  return dist;
}

double FrontierSearch::sigmoid(double x){
    return 1 / (1 + exp(-x));
}

double FrontierSearch::distance_score(double d){
    return tanh(exp(d/BETA) * sigmoid(exp(d/BETA) * (1 - 1 / sinh(d/ALPHA)))) * GAMMA;
}

double FrontierSearch::distance_score_GDAE(double d){
  return tanh(exp(std::pow(d / (L2 - L1), 2)) / exp(pow(L2 / (L2 - L1), 2))) * L2;
}

std::vector<double> FrontierSearch::createPoint(double x, double y) {
    std::vector<double> point = {x, y};
    return point;
}

std::vector<std::vector<double>> FrontierSearch::inner_circle_points(double mx, double my, double radius){
  std::vector<std::vector<double>> points;

  for(int row=0; row <= radius; row++){
    for(int col=0; col <= radius; col++){
      double x_coordinahte = radius - row;
      double y_coordinahte = radius - col;
      double p = std::sqrt(std::pow(x_coordinahte, 2) + std::pow(y_coordinahte, 2));

      if(p <= std::sqrt(std::pow(radius, 2) + 1)){
        points.push_back(createPoint(x_coordinahte + mx, y_coordinahte + my));
      }
    }
  }
  
  return points;
}

std::vector<std::vector<double>> FrontierSearch::square_points(double mx, double my){
  std::vector<std::vector<double>> points;

  for(int row=int(-K / 2); row <= int(K / 2); row++){
    for(int col=int(-K / 2); col <= int(K / 2); col++){
      points.push_back(createPoint(row + mx, col + my));
    }
  }

  return points;
}

double FrontierSearch::cell_information(std::vector<double> p){
  unsigned int idx = costmap_->getIndex((unsigned int) p[0], (unsigned int) p[1]);
  unsigned int map_value = map_[idx];

  if(map_value == 255)
    return 0;
  return map_value + 1;
}

double FrontierSearch::cell_information_GDAE(std::vector<double> p){
  unsigned int idx = costmap_->getIndex((unsigned int) p[0], (unsigned int) p[1]);
  unsigned int map_value = map_[idx];

  if(map_value == 255){
    return 0;
  }
  else if(map_value == 0){
    return 1;
  }

  return 5;
}

double FrontierSearch::map_information(std::vector<double> p, double radius){
  double m_value = 0.0;
  unsigned int mx, my;
  double wx, wy, map_radius = radius/costmap_->getResolution();

  wx = p[0];
  wy = p[1];
  costmap_->worldToMap(wx, wy, mx, my);

  std::vector<std::vector<double>> points = inner_circle_points(mx, my, map_radius);
  
  // for(long unsigned int i = 0; i < points.size(); i++){
  for(const auto point : points){
    // m_value += (cell_information(point) / 255);
    m_value += (cell_information(point) / 25.5);
  }

  return (m_value / (M_PI * std::pow(map_radius, 2)));
}

double FrontierSearch::map_information_GDAE(std::vector<double> p){
  double m_value = 0.0;
  unsigned int mx, my;
  double wx, wy;

  wx = p[0];
  wy = p[1];
  costmap_->worldToMap(wx, wy, mx, my);

  std::vector<std::vector<double>> points = square_points(mx, my);

  // for(long unsigned int i = 0; i < points.size(); i++){
  for(const auto point : points){
    m_value += (cell_information_GDAE(point) / 255);
  }

  return exp(m_value);
}

double FrontierSearch::frontierCost(const Frontier& frontier)
{
  // Custom Hueristic Fucntion

  double distance_score_value = distance_score(frontier.min_distance);

  double map_information_value = map_information(createPoint(frontier.centroid.x, frontier.centroid.y), frontier.radius);
  // std::cout<< "frontier (" << frontier.centroid.x << ", " << frontier.centroid.y << ") occupancy stocastic score : " << map_information_value << std::endl;

  return (distance_score_value + map_information_value * (1 / cosh(frontier.size / ALPHA) * DELTA));// * costmap_->getResolution();


  // GDAE Function

  // double distance_score_value_GDAE = distance_score_GDAE(frontier.min_distance);

  // double map_information_value_GDAE = map_information_GDAE(createPoint(frontier.centroid.x, frontier.centroid.y));

  // return (distance_score_value_GDAE + map_information_value_GDAE);


  // Original code //

  // return (potential_scale_ * frontier.min_distance *
  //         costmap_->getResolution()) -
  //        (gain_scale_ * frontier.size * costmap_->getResolution());


  // Neardest Fronteir //
  // return frontier.min_distance * costmap_->getResolution();
}
}  // namespace frontier_exploration
