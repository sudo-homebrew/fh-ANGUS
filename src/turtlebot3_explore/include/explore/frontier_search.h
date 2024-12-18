#ifndef FRONTIER_SEARCH_H_
#define FRONTIER_SEARCH_H_

#include "nav2_costmap_2d/costmap_2d_ros.hpp"

namespace frontier_exploration
{
/**
 * @brief Represents a frontier
 *
 */
struct Frontier {
  std::uint32_t size;
  double min_distance;
  double cost;
  double radius;
  geometry_msgs::msg::Point initial;
  geometry_msgs::msg::Point centroid;
  geometry_msgs::msg::Point middle;
  std::vector<geometry_msgs::msg::Point> points;
};

/**
 * @brief Thread-safe implementation of a frontier-search task for an input
 * costmap.
 */
class FrontierSearch
{
public:
  FrontierSearch()
  {
  }

  /**
   * @brief Constructor for search task
   * @param costmap Reference to costmap data to search.
   */
  FrontierSearch(nav2_costmap_2d::Costmap2D* costmap, double potential_scale,
                 double gain_scale, double min_frontier_size);

  /**
   * @brief Runs search implementation, outward from the start position
   * @param position Initial position to search from
   * @return List of frontiers, if any
   */
  std::vector<Frontier> searchFrom(geometry_msgs::msg::Point position);

protected:
  /**
   * @brief Starting from an initial cell, build a frontier from valid adjacent
   * cells
   * @param initial_cell Index of cell to start frontier building
   * @param reference Reference index to calculate position from
   * @param frontier_flag Flag vector indicating which cells are already marked
   * as frontiers
   * @return new frontier
   */
  Frontier buildNewFrontier(unsigned int initial_cell, unsigned int reference,
                            std::vector<bool>& frontier_flag);

  /**
   * @brief isNewFrontierCell Evaluate if candidate cell is a valid candidate
   * for a new frontier.
   * @param idx Index of candidate cell
   * @param frontier_flag Flag vector indicating which cells are already marked
   * as frontiers
   * @return true if the cell is frontier cell
   */
  bool isNewFrontierCell(unsigned int idx,
                         const std::vector<bool>& frontier_flag);

  /**
   * @brief functions needed when computing frontier cost
   * @details this functions are for computing complemented heuristic function
   *
   * @param sevral diffenent depend each functions
   * @return math values
   */
  std::vector<double> createPoint(double x, double y);

  double fardest_point_from_centroid(std::vector<unsigned int> line, double centroid_x, double centroid_y);

  double sigmoid(double x);

  double distance_score(double d);
  double distance_score_GDAE(double d);

  std::vector<std::vector<double>>inner_circle_points(double mx, double my, double radius);
  std::vector<std::vector<double>>square_points(double mx, double my);

  double cell_information(std::vector<double> p);

  double map_information(std::vector<double> p, double radius);

  double cell_information_GDAE(std::vector<double> p);
  double map_information_GDAE(std::vector<double> p);

  /**
   * @brief computes frontier cost
   * @details cost function is defined by potential_scale and gain_scale
   *
   * @param frontier frontier for which compute the cost
   * @return cost of the frontier
   */
  double frontierCost(const Frontier& frontier);

private:
  nav2_costmap_2d::Costmap2D* costmap_;
  unsigned char* map_;
  unsigned int size_x_, size_y_;
  double potential_scale_, gain_scale_;
  double min_frontier_size_;
};
}  // namespace frontier_exploration
#endif
