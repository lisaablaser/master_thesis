#include <memory>
#include <octomap/OcTree.h>


#include "automatic_cell_explorer/constants.hpp"
#include "automatic_cell_explorer/exploration_planner/raycast.hpp"



RayView calculateRayView(
    const Eigen::Isometry3d& sensor_state, 
    std::shared_ptr<octomap::OcTree> octo_map, 
    double max_range) 
{
    RayView ray_view;
    ray_view.pose = sensor_state;

    const double horizontal_fov = FOV_H * M_PI / 180.0; 
    const double vertical_fov = FOV_V * M_PI / 180.0;   

    double half_horizontal_fov = horizontal_fov / 2.0;
    double half_vertical_fov = vertical_fov / 2.0;

    const int horizontal_rays = 100;            
    const int vertical_rays = 50;                       


    Eigen::Vector3d sensor_origin = sensor_state.translation();

    
    ray_view.num_unknowns = 0;

    for (int i = 0; i < horizontal_rays; ++i) {
        for (int j = 0; j < vertical_rays; ++j) {
            
            double horizontal_angle = (i / static_cast<double>(horizontal_rays - 1)) * horizontal_fov - half_horizontal_fov;
            double vertical_angle = (j / static_cast<double>(vertical_rays - 1)) * vertical_fov - half_vertical_fov;

            Eigen::Vector3d ray_direction_camera(
                std::cos(horizontal_angle) * std::cos(vertical_angle), 
                std::sin(horizontal_angle) * std::cos(vertical_angle),  
                -std::sin(vertical_angle)                              
            );

            Eigen::Vector3d ray_direction_world = sensor_state.rotation() * ray_direction_camera;

            RayInfo ray_info = castRay(octo_map, sensor_origin, ray_direction_world);
            if(ray_info.node_state == NodeState::Unknown){
                ray_view.num_unknowns++;
            }
            if(ray_info.node_state == NodeState::Error){
                std::cout << " ----- Error in raycast accoured -------------" << std::endl;
            }

            ray_view.rays.push_back(ray_info);
        }
    }

    return ray_view;
}

RayInfo castRay(std::shared_ptr<octomap::OcTree> & octo_map, const Eigen::Vector3d & sensor_origin, const Eigen::Vector3d & ray_direction_world){

    WorkspaceBounds bounds = WORK_SPACE;
    OrigoOffset origo = ORIGO;
    octomap::point3d end;
    Eigen::Vector3d ray_end;
    NodeState node_state = NodeState::Occupied;
    octomap::point3d hit_point;

    // Initialization phase -------------------------------------------------------

    octomap::OcTreeKey current_key;
    if ( !octo_map->coordToKeyChecked(octomap::point3d(sensor_origin.x(), sensor_origin.y(), sensor_origin.z()), current_key) ) {
      OCTOMAP_WARNING_STR("Coordinates out of bounds during ray casting");

      return RayInfo({sensor_origin, ray_end, NodeState::Error});
    }

    octomap::OcTreeNode* startingNode = octo_map->search(current_key);
    if (startingNode){
      if (octo_map->isNodeOccupied(startingNode)){
        // Occupied node found at origin
        // (need to convert from key, since origin does not need to be a voxel center)
        end = octo_map->keyToCoord(current_key);
        ray_end = Eigen::Vector3d(end.x(), end.y(), end.z());

        return RayInfo({sensor_origin, ray_end, NodeState::Occupied});
      }
    } else {
      end = octo_map->keyToCoord(current_key);
      ray_end = Eigen::Vector3d(end.x(), end.y(), end.z());
      return RayInfo({sensor_origin, ray_end, NodeState::Unknown});
    }

    octomap::point3d direction = octomap::point3d(ray_direction_world.x(), ray_direction_world.y(), ray_direction_world.z()).normalized();
   
    int step[3];
    double tMax[3];
    double tDelta[3];

    for(unsigned int i=0; i < 3; ++i) {
      // compute step direction
      if (direction(i) > 0.0) step[i] =  1;
      else if (direction(i) < 0.0)   step[i] = -1;
      else step[i] = 0;

      // compute tMax, tDelta
      if (step[i] != 0) {
        // corner point of voxel (in direction of ray)
        double voxelBorder = octo_map->keyToCoord(current_key[i]);
        voxelBorder += double(step[i] * octo_map->getResolution() * 0.5);

        tMax[i] = ( voxelBorder - sensor_origin(i) ) / direction(i);
        tDelta[i] = octo_map->getResolution() / fabs( direction(i) );
      }
      else {
        tMax[i] =  std::numeric_limits<double>::max();
        tDelta[i] = std::numeric_limits<double>::max();
      }
    }

    if (step[0] == 0 && step[1] == 0 && step[2] == 0){
    	OCTOMAP_ERROR("Raycasting in direction (0,0,0) is not possible!");
    	return RayInfo({sensor_origin, ray_end, NodeState::Error});
    }

    // Incremental phase  ---------------------------------------------------------

    bool done = false;

    while (!done) {
      unsigned int dim;

      // find minimum tMax:
      if (tMax[0] < tMax[1]){
        if (tMax[0] < tMax[2]) dim = 0;
        else                   dim = 2;
      }
      else {
        if (tMax[1] < tMax[2]) dim = 1;
        else                   dim = 2;
      }

      // check for overflow:
      if ((step[dim] < 0 && current_key[dim] == 0)
    		  || (step[dim] > 0 && current_key[dim] == (1 << octo_map->getTreeDepth()) - 1))
      {
        OCTOMAP_WARNING("Coordinate hit bounds in dim %d, aborting raycast\n", dim);
        // return border point nevertheless:
        end = octo_map->keyToCoord(current_key);
        ray_end = Eigen::Vector3d(end.x(), end.y(), end.z());
        return RayInfo({sensor_origin, ray_end, NodeState::Error});
      }

      // advance in direction "dim"
      current_key[dim] += step[dim];
      tMax[dim] += tDelta[dim];


      // generate world coords from key
      end = octo_map->keyToCoord(current_key);

      // check for maxrange:
      if (end(0) <= bounds.min_x || end(0) >= bounds.max_x ||
        end(1) <= bounds.min_y || end(1) >= bounds.max_y ||
        end(2) <= (origo.z+bounds.min_z) || end(2) >= (origo.z+bounds.max_z)) {
        
        ray_end = Eigen::Vector3d(end.x(), end.y(), end.z());
        return RayInfo({sensor_origin, ray_end, NodeState::Free});
        
            
        }
            

      octomap::OcTreeNode* currentNode = octo_map->search(current_key);
      if (currentNode){
        if (octo_map->isNodeOccupied(currentNode)) {
            ray_end = Eigen::Vector3d(end.x(), end.y(), end.z());
            return RayInfo({sensor_origin, ray_end, NodeState::Free});
          //done = true;
          //break;
        }
        // otherwise: node is free and valid, raycasting continues
      } else { // no node found, this usually means we are in "unknown" areas
        ray_end = Eigen::Vector3d(end.x(), end.y(), end.z());
        return RayInfo({sensor_origin, ray_end, NodeState::Unknown});
      }
    } // end while

    ray_end = Eigen::Vector3d(end.x(), end.y(), end.z());
    return RayInfo({sensor_origin, ray_end, NodeState::Occupied});

  }


