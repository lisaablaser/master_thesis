#include "automatic_cell_explorer/octomap_processor.hpp"


void createInitialSafeSpace(octomap::OcTree* received_tree, double x, double y, double z, double resolution) {
    double z_offset = 0.72;
    
    for (double i = -x / 2.0; i <= x / 2.0; i += resolution) {
        for (double j = -y / 2.0; j <= y / 2.0; j += resolution) {
            for (double k = z_offset; k <= z+ z_offset; k += resolution) {
                octomap::point3d point(i, j, k);
                received_tree->updateNode(point, false);  
            }
        }
    }

    received_tree->updateInnerOccupancy();
}

void updatePlanningScene(octomap::OcTree* received_tree, OctrePtr unknownVoxelsTree) {
    /*
        For now, faster method to update planning scene. But results in sparse updates.
    */
    
    for (auto it = unknownVoxelsTree->begin_leafs(), end = unknownVoxelsTree->end_leafs(); it != end; ++it) {
        octomap::point3d point = it.getCoordinate();  
        double size = it.getSize();                  

        
        received_tree->updateNode(point, true);
    }

    
    received_tree->updateInnerOccupancy();
}

void markUnknownSpaceAsObstacles(octomap::OcTree* received_tree, double x, double y, double z, double resolution) {
    /*
        very slow...
    */
    double z_offset = 0.72;
    for (double i = -x / 2.0; i <= x / 2.0; i += resolution) {
        for (double j = -y / 2.0; j <= y / 2.0; j += resolution) {
            for (double k = z_offset - z / 2.0; k <= z_offset + z / 2.0; k += resolution) {
    
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

OctrePtr extractUnknownOctree(const octomap::OcTree* octree) {

    double exploration_dims = 1.0;
    double z_offset = 0.72;
    
    double resolution = octree->getResolution();

    OctrePtr unknownVoxelsTree = std::make_shared<octomap::OcTree>(octree->getResolution());
    
    octomap::point3d center(0.0, 0.0, z_offset);
    double searchRadius = exploration_dims;// - resolution;
    octomap::point3d min = center - octomap::point3d(searchRadius, searchRadius, 0.0);
    octomap::point3d max = center + octomap::point3d(searchRadius, searchRadius, 2*searchRadius);

    octomap::OcTreeKey minKey, maxKey;
    if (!octree->coordToKeyChecked(min, minKey) || !octree->coordToKeyChecked(max, maxKey)) {
        
        return unknownVoxelsTree;
    }

    for (double x = min.x(); x <= max.x(); x += resolution) {
        for (double y = min.y(); y <= max.y(); y += resolution) {
            for (double z = min.z(); z <= max.z(); z += resolution) {
                octomap::OcTreeKey key = octree->coordToKey(x, y, z);
                octomap::OcTreeNode* node = octree->search(key);
                if (!node) {
                    unknownVoxelsTree->updateNode(octomap::point3d(x, y, z), true);
                }
            }
        }
    }
    unknownVoxelsTree->updateInnerOccupancy();
    //unknownVoxelsTree->prune();
    
    return unknownVoxelsTree;
}

OctrePtr extractFreeOctree(const octomap::OcTree* octree) {
    double exploration_dims = 1.0;
    double z_offset = 0.72;
    
    double resolution = octree->getResolution();

    OctrePtr freeVoxelsTree = std::make_shared<octomap::OcTree>(resolution);
    
    octomap::point3d center(0.0, 0.0, z_offset);
    double searchRadius = exploration_dims - resolution; 
    octomap::point3d min = center - octomap::point3d(searchRadius, searchRadius, 0.0);
    octomap::point3d max = center + octomap::point3d(searchRadius, searchRadius, searchRadius);
    
    for (double x = min.x(); x <= max.x(); x += resolution) {
        for (double y = min.y(); y <= max.y(); y += resolution) {
            for (double z = min.z(); z <= max.z(); z += resolution) {
                octomap::OcTreeKey key = octree->coordToKey(x, y, z);
                octomap::OcTreeNode* node = octree->search(key);
                if (node != nullptr && !octree->isNodeOccupied(node)) {
                    freeVoxelsTree->updateNode(octomap::point3d(x, y, z), true);
                }
            }
        }
    }

    freeVoxelsTree->updateInnerOccupancy();
    freeVoxelsTree->prune();

    return freeVoxelsTree;
}

OctrePtr extractFrontierOctree(const octomap::OcTree* octree) {
    double resolution = octree->getResolution();
    OctrePtr frontierVoxelsTree = std::make_shared<octomap::OcTree>(resolution);

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
                            if (!node) { // Node does not exist (unknown area)
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