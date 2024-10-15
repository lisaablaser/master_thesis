#include "automatic_cell_explorer/exploration_planner/evaluate_nbv.hpp"


void updateRayViews(NbvCandidates& nbvs,  std::shared_ptr<octomap::OcTree> octo_map){
   // add as constant/variable later. 
    double max_range = 0.93;

    // or range worksapce bounds in all directions???
    
    std::cout << "updating ray views " << std::endl;
    for(Nbv &nbv: nbvs.nbv_candidates){
        Eigen::Isometry3d sensor_pose = nbv.pose;
        RayView ray_view = calculateRayView(sensor_pose, octo_map, max_range);
        nbv.ray_view = ray_view;
    
    }

}

