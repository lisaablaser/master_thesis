#include "automatic_cell_explorer/ep_factory.hpp"

std::shared_ptr<ExplorationPlanner> createPlanner(PlannerType type, MoveGrpPtr mvt_interface, std::shared_ptr<octomap::OcTree> octo_map) {
    /*
        Combines a local and a global exploration planning method.
    */

    switch (type) {
        case PlannerType::Local:

            return std::make_shared<ExplorationPlannerV3>(mvt_interface, octo_map);
            
        case PlannerType::Global:

            return std::make_shared<ExplorationPlannerV4>(mvt_interface, octo_map);

        default:
           
            throw std::invalid_argument("Unsupported PlannerType");
    }
}