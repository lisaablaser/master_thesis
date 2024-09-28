#include "automatic_cell_explorer/exploration_planner/evaluate_nbv.hpp"


void updateRayViews(NbvCandidates& nbvs,  std::shared_ptr<octomap::OcTree> octo_map){
    double max_range = 10.0;

    for(Nbv &nbv: nbvs.nbv_candidates){
        Eigen::Isometry3d sensor_pose = nbv.pose;
        RayView ray_view = calculateRayView(sensor_pose, octo_map, max_range);
        nbv.ray_view = ray_view;
    
    }

}

