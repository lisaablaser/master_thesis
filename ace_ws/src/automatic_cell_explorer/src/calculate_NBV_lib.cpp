#include "automatic_cell_explorer/calculate_NBV_lib.hpp"
#include <octomap/OcTree.h>
#include <octomap/Pointcloud.h>
#include <octomap/math/Vector3.h>


CalculateNBV::CalculateNBV(std::shared_ptr<octomap::OcTree> planning_scene)
    : planning_scene_(planning_scene)
{
}

std::shared_ptr<octomap::point3d> CalculateNBV::calculate_next_best_view()
{
    // Placeholder logic for calculating the next best view
    // In a real scenario, this would involve more complex logic to evaluate
    // different potential viewpoints and select the one with the highest information gain.
    return std::make_shared<octomap::point3d>(0.0, 0.0, 0.0);
}

double CalculateNBV::calculate_occupied_volume() const
{
    double volume = 0.0;
    for (auto it = planning_scene_->begin_leafs(), end = planning_scene_->end_leafs(); it != end; ++it) {
        if (planning_scene_->isNodeOccupied(*it)) {
            volume += compute_node_volume(planning_scene_->getResolution());
        }
    }
    return volume;
}

double CalculateNBV::compute_node_volume(double resolution) const
{
    // Assuming each occupied node is a cube with side length equal to the resolution
    return resolution * resolution * resolution;
}