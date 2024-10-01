#include "automatic_cell_explorer/octomap_processor.hpp"
#include "automatic_cell_explorer/constants.hpp"


void createInitialSafeSpace(octomap::OcTree* received_tree) {
    double resolution = RES_SMALL;
    FreespaceBounds bounds = FREE_SPACE;
    OrigoOffset origo = ORIGO;
    
    for (double i = bounds.min_x; i <= bounds.max_x; i += resolution) {
        for (double j = bounds.min_y; j <= bounds.max_y; j += resolution) {
            for (double k = bounds.min_z + origo.z; k <= bounds.max_z + origo.z; k += resolution) {
                octomap::point3d point(i, j, k);
                received_tree->updateNode(point, false);  
            }
        }
    }

    received_tree->updateInnerOccupancy();
}

void updatePlanningScene(octomap::OcTree* received_tree, OctreePtr unknownVoxelsTree) {
    /*
        For now, faster method to update planning scene. But results in sparse updates.
        TODO: account for voxel size. 
    */
    
    for (auto it = unknownVoxelsTree->begin_leafs(), end = unknownVoxelsTree->end_leafs(); it != end; ++it) {
        octomap::point3d point = it.getCoordinate();  
                    
        received_tree->updateNode(point, true);
    }
    
    received_tree->updateInnerOccupancy();
}

void markUnknownSpaceAsObstacles(octomap::OcTree* received_tree) {
    /*
        very slow...
    */
    double resolution = RES_SMALL;
    WorkspaceBounds bounds = WORK_SPACE;
    OrigoOffset origo = ORIGO;

    for (double i = bounds.min_x; i <= bounds.max_x; i += resolution) {
        for (double j = bounds.min_y; j <= bounds.max_y; j += resolution) {
            for (double k = bounds.min_z + origo.z; k <= bounds.max_z + origo.z; k += resolution){
                octomap::point3d point(i, j, k);
                octomap::OcTreeNode* node = received_tree->search(point);
                
                if (node == nullptr) {
                    
                    received_tree->updateNode(point, true); 
                }
            }
        }
    }

    received_tree->updateInnerOccupancy();
}

OctreePtr extractUnknownOctree(const OctreePtr octree) {

    double resolution = RES_LARGE;
    WorkspaceBounds bounds = WORK_SPACE;
    OrigoOffset origo = ORIGO;

    OctreePtr unknownVoxelsTree = std::make_shared<octomap::OcTree>(resolution);
    

    for (double x = bounds.min_x; x <= bounds.max_x + resolution; x += resolution) {
        for (double y = bounds.min_y; y <= bounds.max_y + resolution; y += resolution) {
            for (double z = bounds.min_z - resolution + origo.z; z <= bounds.max_z + origo.z + resolution; z += resolution){
                octomap::OcTreeKey key = octree->coordToKey(x, y, z);
                octomap::OcTreeNode* node = octree->search(key);

                if (!node) {
                    unknownVoxelsTree->updateNode(octomap::point3d(x, y, z), true);
                }
            }
        }
    }
    unknownVoxelsTree->updateInnerOccupancy();
    
    return unknownVoxelsTree;
}

OctreePtr extractFreeOctree(const OctreePtr octree) {
    double resolution = RES_LARGE;
    WorkspaceBounds bounds = WORK_SPACE;
    OrigoOffset origo = ORIGO;

    OctreePtr freeVoxelsTree = std::make_shared<octomap::OcTree>(resolution);
    

    for (double x = bounds.min_x; x <= bounds.max_x ; x += resolution) {
        for (double y = bounds.min_y; y <= bounds.max_y ; y += resolution) {
            for (double z = bounds.min_z + origo.z; z <= bounds.max_z + origo.z ; z += resolution){
                octomap::OcTreeKey key = octree->coordToKey(x, y, z);
                octomap::OcTreeNode* node = octree->search(key);
                if (node != nullptr && !octree->isNodeOccupied(node)) {
                    freeVoxelsTree->updateNode(octomap::point3d(x, y, z), true);
                }
            }
        }
    }

    freeVoxelsTree->updateInnerOccupancy();

    return freeVoxelsTree;
}

OctreePtr extractFrontierOctree(const OctreePtr octree) {
    double resolution = RES_LARGE;
    OctreePtr frontierVoxelsTree = std::make_shared<octomap::OcTree>(resolution);

    for (octomap::OcTree::leaf_iterator it = octree->begin_leafs(), end = octree->end_leafs(); it != end; ++it) {
        if (octree->isNodeOccupied(*it)) {
            continue; 
        } else {
            // The node is free. Check if any of its neighbors are unknown
            bool isFrontier = false;
            octomap::OcTreeKey key;
            for (key[0] = it.getKey()[0] - 1; key[0] <= it.getKey()[0] + 1 && !isFrontier; ++key[0]) {
                for (key[1] = it.getKey()[1] - 1; key[1] <= it.getKey()[1] + 1 && !isFrontier; ++key[1]) {
                    for (key[2] = it.getKey()[2] - 1; key[2] <= it.getKey()[2] + 1 && !isFrontier; ++key[2]) {
                        if (key != it.getKey()) { // Skip the center node itself
                            octomap::OcTreeNode* node = octree->search(key);
                            if (!node) { 
                                isFrontier = true;
                                break;
                            }
                        }
                    }
                }
            }
            if (isFrontier) {
                frontierVoxelsTree->updateNode(it.getCoordinate(), true);
            }
        }
    }

    frontierVoxelsTree->updateInnerOccupancy();
    frontierVoxelsTree->prune();
    return frontierVoxelsTree;
}

OctreePtr extractFrontierOctreeInBounds(const OctreePtr octree) {
    /// TODO: får ikke med seg alle frontiers på sidene. 

    double resolution = RES_LARGE;
    WorkspaceBounds bounds = WORK_SPACE;
    OrigoOffset origo = ORIGO;
  
    OctreePtr frontierVoxelsTree = std::make_shared<octomap::OcTree>(resolution);


    for (octomap::OcTree::leaf_iterator it = octree->begin_leafs(), end = octree->end_leafs(); it != end; ++it) {
        
        octomap::point3d node_coords = it.getCoordinate();

        if (node_coords.x() < bounds.min_x -resolution || node_coords.x() > bounds.max_x + resolution ||
            node_coords.y() < bounds.min_y -resolution || node_coords.y() > bounds.max_y + resolution||
            node_coords.z() <(bounds.min_z+origo.z) || node_coords.z() > (bounds.max_z+origo.z+ resolution)) {
            continue;
        }

        if (octree->isNodeOccupied(*it)) {
            continue;
        } else {
            bool isFrontier = false;
            octomap::OcTreeKey key;
            for (key[0] = it.getKey()[0] - 1; key[0] <= it.getKey()[0] + 1 && !isFrontier; ++key[0]) {
                for (key[1] = it.getKey()[1] - 1; key[1] <= it.getKey()[1] + 1 && !isFrontier; ++key[1]) {
                    for (key[2] = it.getKey()[2] - 1; key[2] <= it.getKey()[2] + 1 && !isFrontier; ++key[2]) {
                        if (key != it.getKey()) { // Skip the center node itself
                            octomap::OcTreeNode* node = octree->search(key);
                            if (!node) { 
                                isFrontier = true;
                                break;
                            }
                        }
                    }
                }
            }
            // If the node is a frontier, update the frontier octree
            if (isFrontier) {
                frontierVoxelsTree->updateNode(it.getCoordinate(), true);
            }
        }
    }

    frontierVoxelsTree->updateInnerOccupancy();
    frontierVoxelsTree->prune();
    return frontierVoxelsTree;
}

double calculateOccupiedVolume(const OctreePtr octree) {
    WorkspaceBounds bounds = WORK_SPACE;
    
    double volume = 0.0;
    for (octomap::OcTree::leaf_iterator it = octree->begin_leafs(), end=octree->end_leafs(); it != end; ++it) {
        if (octree->isNodeOccupied(*it)) {
            double nodeVolume = std::pow(it.getSize(), 3); 
            volume += nodeVolume;
        }
    }
    double total_volume = (bounds.max_x-bounds.min_x)*(bounds.max_y-bounds.min_y)*(bounds.max_z-bounds.min_z);

    double percentage_explored = 100 - (volume/total_volume)*100;

    return percentage_explored;
}