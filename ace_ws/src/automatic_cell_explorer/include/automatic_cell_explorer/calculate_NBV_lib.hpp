#ifndef CALCULATE_NBV_LIB_HPP
#define CALCULATE_NBV_LIB_HPP

#include <memory>
#include <octomap/octomap.h>

class CalculateNBV
{
public:
  CalculateNBV(std::shared_ptr<octomap::OcTree> planning_scene);

  std::shared_ptr<octomap::point3d> calculate_next_best_view();

  double calculate_occupied_volume() const;

private:

  std::shared_ptr<octomap::OcTree> planning_scene_;

  double compute_node_volume(double resolution) const;

  
};


#endif // CALCULATE_NBV_LIB_HPP