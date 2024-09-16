
// Input: sensor_state, fov_h, fov_v, res, octree
// Output: Dict[end_point, stautus(unknown/free/occupied)]

bool castRay(const point3d& origin, const point3d& directionP, point3d& end,
                                          bool ignoreUnknown, double maxRange) const{


    // Initialization phase -------------------------------------------------------
    OcTreeKey current_key;
    if ( !OcTreeBaseImpl<NODE,AbstractOccupancyOcTree>::coordToKeyChecked(origin, current_key) ) {
      OCTOMAP_WARNING_STR("Coordinates out of bounds during ray casting");
      return false;
    }

    NODE* startingNode = this->search(current_key);
    if (startingNode){
      if (this->isNodeOccupied(startingNode)){
        // Occupied node found at origin
        // (need to convert from key, since origin does not need to be a voxel center)
        end = this->keyToCoord(current_key);
        return true;
      }
    } else if(!ignoreUnknown){
      end = this->keyToCoord(current_key);
      return false;
    }

    point3d direction = directionP.normalized();
    bool max_range_set = (maxRange > 0.0);

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
        double voxelBorder = this->keyToCoord(current_key[i]);
        voxelBorder += double(step[i] * this->resolution * 0.5);

        tMax[i] = ( voxelBorder - origin(i) ) / direction(i);
        tDelta[i] = this->resolution / fabs( direction(i) );
      }
      else {
        tMax[i] =  std::numeric_limits<double>::max();
        tDelta[i] = std::numeric_limits<double>::max();
      }
    }

    if (step[0] == 0 && step[1] == 0 && step[2] == 0){
    	OCTOMAP_ERROR("Raycasting in direction (0,0,0) is not possible!");
    	return false;
    }

    // for speedup:
    double maxrange_sq = maxRange *maxRange;

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
    		  || (step[dim] > 0 && current_key[dim] == 2* this->tree_max_val-1))
      {
        OCTOMAP_WARNING("Coordinate hit bounds in dim %d, aborting raycast\n", dim);
        // return border point nevertheless:
        end = this->keyToCoord(current_key);
        return false;
      }

      // advance in direction "dim"
      current_key[dim] += step[dim];
      tMax[dim] += tDelta[dim];


      // generate world coords from key
      end = this->keyToCoord(current_key);

      // check for maxrange:
      if (max_range_set){
        double dist_from_origin_sq(0.0);
        for (unsigned int j = 0; j < 3; j++) {
          dist_from_origin_sq += ((end(j) - origin(j)) * (end(j) - origin(j)));
        }
        if (dist_from_origin_sq > maxrange_sq)
          return false;

      }

      NODE* currentNode = this->search(current_key);
      if (currentNode){
        if (this->isNodeOccupied(currentNode)) {
          done = true;
          break;
        }
        // otherwise: node is free and valid, raycasting continues
      } else if (!ignoreUnknown){ // no node found, this usually means we are in "unknown" areas
        return false;
      }
    } // end while

    return true;
  }