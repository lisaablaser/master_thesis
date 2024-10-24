#include "automatic_cell_explorer/octomap_processor.hpp"
#include "automatic_cell_explorer/constants.hpp"

#include <queue>

void computeCenterAndNormal(Cluster &cluster);
bool isWithinDistance(const octomap::point3d& p1, const octomap::point3d& p2);
std::vector<octomap::point3d> extractOccupiedNodes(std::shared_ptr<octomap::OcTree> octree);
void findFrontiers(Cluster &cluste);

std::vector<Cluster> computeClusters(std::shared_ptr<octomap::OcTree>  octree) {
    double res = RES_LARGE;
    OctreePtr unknown_tree = extractUnknownOctree(octree); 
    unknown_tree->expand();
    std::vector<octomap::point3d> unknown_nodes = extractOccupiedNodes(unknown_tree); 

    std::vector<Cluster> clusters;
    std::vector<bool> visited(unknown_nodes.size(), false);  

    std::cout << "Computing clusters " << std::endl;
    // Perform clustering using a BFS-like approach
    for (size_t i = 0; i < unknown_nodes.size(); ++i) {
        if (visited[i]) continue;  

        Cluster new_cluster;
        std::queue<size_t> to_explore;
        to_explore.push(i);
        visited[i] = true;


        while (!to_explore.empty()) {
            size_t current = to_explore.front();
            to_explore.pop();

            new_cluster.points.push_back(unknown_nodes[current]);


            auto current_point = unknown_nodes[current];
 
            // Check neighbors (in original map) for frontiers
            // Obs size is not taken into account. Maybe interate +- resolution and search 3d point instead of key
            /// BUG: floor frontiers are either not being removed (2*res), or dont exist. Maybe due to differences in res. 
            for (float i = current_point.x() - res; i <= current_point.x() + res; i += res) {
                for (float j = current_point.y() - res; j <= current_point.y() + res; j += res) {
                    for (float k = current_point.z() - res; k <= current_point.z() + 2*res; k += res) { //Temporary bug "fix"
                        if (i == current_point.x() && j == current_point.y() && k == current_point.z()) continue; 
                        octomap::OcTreeKey neighbor_key;;
                        octree->coordToKeyChecked(octomap::point3d(i, j, k), neighbor_key);

                        auto node = octree->search(neighbor_key);
                        
                        if (node != nullptr && !octree->isNodeOccupied(node)) {
                            
                            new_cluster.frontiers.push_back(octree->keyToCoord(neighbor_key)); /// TODO: remove duplicates
                        }
                    }
                }
            }


            // Find neighboring points within the cluster distance
            /// TODO: account for voxel size.  
            for (size_t j = 0; j < unknown_nodes.size(); ++j) {
                if (!visited[j] && isWithinDistance(unknown_nodes[current], unknown_nodes[j])) {
                    to_explore.push(j);
                    visited[j] = true;
                }
            }
        }
        
        std::cout << "Number of frontiers attached to cluster is: " << new_cluster.frontiers.size() << std::endl;


        // Calculate center and normal
        computeCenterAndNormal(new_cluster);

        clusters.push_back(new_cluster);  
    }

    return clusters;
}

void findFrontiers(Cluster &cluste){


}

void computeCenterAndNormal(Cluster &cluster) {
    // Compute center
    Eigen::Vector3d center(0, 0, 0);
    for (const auto& point : cluster.points) {
        center += Eigen::Vector3d(point.x(), point.y(), point.z());
    }
    center /= cluster.points.size();
    cluster.center = octomap::point3d(center.x(), center.y(), center.z());

    if (cluster.points.size() < 3) {
        // If there are less than 3 points, assign a default normal (e.g., pointing up in z-direction)
        cluster.normal = Eigen::Vector3d(0, 0, 1);  // Default normal
        return;
    }

    // Compute normal (PCA)
    Eigen::MatrixXd points_matrix(cluster.points.size(), 3);
    for (size_t i = 0; i < cluster.points.size(); ++i) {
        points_matrix(i, 0) = cluster.points[i].x() - center.x();
        points_matrix(i, 1) = cluster.points[i].y() - center.y();
        points_matrix(i, 2) = cluster.points[i].z() - center.z();
    }
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(points_matrix, Eigen::ComputeThinU | Eigen::ComputeThinV);
    cluster.normal = svd.matrixV().col(2);  // The last column corresponds to the smallest singular value (normal to the plane)
}

std::vector<octomap::point3d> extractOccupiedNodes(std::shared_ptr<octomap::OcTree> octree) {
    std::vector<octomap::point3d> occupied_nodes;

    for (octomap::OcTree::leaf_iterator it = octree->begin_leafs(), end = octree->end_leafs(); it != end; ++it) {
        if (octree->isNodeOccupied(*it)) {
            occupied_nodes.push_back(it.getCoordinate());  // Save the occupied node's coordinates
        }
    }

    return occupied_nodes;
}


bool isWithinDistance(const octomap::point3d& p1, const octomap::point3d& p2) {
    double resolution = RES_LARGE; 
    double cluster_distance = 2*resolution;
    return p1.distance(p2) <= cluster_distance;
}


bool isWithinBounds(octomap::point3d point){

    double resolution = RES_LARGE; 
    WorkspaceBounds bounds = WORK_SPACE;
    OrigoOffset origo = ORIGO;

    if (point.x() <= bounds.min_x || point.x() >= bounds.max_x ||
        point.y() <= bounds.min_y || point.y() >= bounds.max_y||
        point.z() <=(bounds.min_z+ origo.z ) || point.z() >=(bounds.max_z+origo.z)) {
        return false;
    }
    return true;
}

bool isNodeUnknown(octomap::OcTreeNode* node){
    if (!node) {
        return true;
    }
    return false;
}

#include <set>
namespace octomap {
    // Define operator< inside the octomap namespace
    bool operator<(const OcTreeKey& lhs, const OcTreeKey& rhs) {
        if (lhs.k[0] != rhs.k[0])
            return lhs.k[0] < rhs.k[0];
        if (lhs.k[1] != rhs.k[1])
            return lhs.k[1] < rhs.k[1];
        return lhs.k[2] < rhs.k[2];
    }
}

std::vector<Cluster> findUnknownVoxelClusters(std::shared_ptr<octomap::OcTree>  octree) {
    /*
    Alternative appraoch for better efficency. 
    Not working yet. 
    
    */
    double resolution = RES_LARGE; 
    WorkspaceBounds bounds = WORK_SPACE;
    OrigoOffset origo = ORIGO;
    
    std::vector<Cluster> clusters;
    
    std::set<octomap::OcTreeKey> visited; // To keep track of visited nodes
    std::queue<octomap::OcTreeKey> to_visit; // Queue for BFS/DFS traversal

    // Iterate through all nodes in the octree
    for (octomap::OcTree::leaf_iterator it = octree->begin_leafs(resolution), end = octree->end_leafs(); it != end; ++it) {
        octomap::point3d point = it.getCoordinate();
        
        // Skip if already visited or not within bounds
        if (visited.count(it.getKey()) || !isWithinBounds(point)) {
            continue;
        }

        if (isNodeUnknown(octree->search(it.getKey())) ){
            // Start a new cluster
            Cluster cluster;
            to_visit.push(it.getKey());

            // BFS/DFS to explore connected unknown voxels
            while (!to_visit.empty()) {
                octomap::OcTreeKey current_key = to_visit.front();
                to_visit.pop();

                if (visited.count(current_key)) continue; // Skip if already visited
                visited.insert(current_key); // Mark current node as visited

                // Get the current point and node
                octomap::point3d current_point = octree->keyToCoord(current_key);
                cluster.points.push_back(current_point);

                // Check neighbors for connected unknown voxels and potential frontiers
                for (int dx = -1; dx <= 1; dx++) {
                    for (int dy = -1; dy <= 1; dy++) {
                        for (int dz = -1; dz <= 1; dz++) {
                            if (dx == 0 && dy == 0 && dz == 0) continue; // Skip self

                            octomap::OcTreeKey neighbor_key = current_key;
                            neighbor_key.k[0] += dx;
                            neighbor_key.k[1] += dy;
                            neighbor_key.k[2] += dz;

                            // Check if the neighbor is unknown and not visited
                            if (!visited.count(neighbor_key)) {
                                octomap::OcTreeNode* neighbor_node = octree->search(neighbor_key);
                                if (neighbor_node && isNodeUnknown(neighbor_node)) {
                                    to_visit.push(neighbor_key);
                                }
                                // If the neighbor is free, mark the current voxel as frontier
                                else if (neighbor_node && octree->isNodeOccupied(neighbor_node) == false) {
                                    cluster.frontiers.push_back(current_point);
                                }
                            }
                        }
                    }
                }
            }

            // Add the cluster to the list of clusters
            clusters.push_back(cluster);
        }
    }
    
    return clusters;
}





void createInitialSafeSpace(octomap::OcTree* received_tree) {
    double resolution = RES_SMALL; // Has to be smaller than incoming tree to avoid gaps. Dont know why, maybe rounding errors. 
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
        Ffaster method to update planning scene if unknown voxel tree is calculated anyways. 
        TODO: account for voxel size. Or expand unkown voxel input to avoid sparse tree. 
    */
    
    for (auto it = unknownVoxelsTree->begin_leafs(), end = unknownVoxelsTree->end_leafs(); it != end; ++it) {
        octomap::point3d point = it.getCoordinate();  
        //get res?
                    
        received_tree->updateNode(point, true);
    }
    
    //received_tree->updateInnerOccupancy();
}


void markUnknownSpaceAsObstacles(octomap::OcTree* received_tree) {
    /*
        very slow...
    */
    double resolution = RES_LARGE;
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
    /*
        Returns an octree of Worksapce size wiht all nodes at resolution RES_LARGE??
        Allows for effeicent leaf iteration later. 
    */

    double resolution = RES_LARGE;
    WorkspaceBounds bounds = WORK_SPACE;
    OrigoOffset origo = ORIGO;

    OctreePtr unknownVoxelsTree = std::make_shared<octomap::OcTree>(resolution);
    

    for (double x = bounds.min_x; x <= bounds.max_x; x += resolution) {
        for (double y = bounds.min_y; y <= bounds.max_y; y += resolution) {
            for (double z = bounds.min_z  + origo.z; z <= bounds.max_z + origo.z; z += resolution){
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


    double resolution = RES_LARGE; // Or small?
    WorkspaceBounds bounds = WORK_SPACE;
    OrigoOffset origo = ORIGO;
  
    OctreePtr frontierVoxelsTree = std::make_shared<octomap::OcTree>(resolution);


    for (octomap::OcTree::leaf_iterator it = octree->begin_leafs(), end = octree->end_leafs(); it != end; ++it) {
        
        octomap::point3d node_coords = it.getCoordinate();

        if (node_coords.x() < bounds.min_x || node_coords.x() > bounds.max_x ||
            node_coords.y() < bounds.min_y || node_coords.y() > bounds.max_y||
            node_coords.z() <(bounds.min_z+ origo.z ) || node_coords.z() > (bounds.max_z+origo.z)) {
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

    //frontierVoxelsTree->updateInnerOccupancy();
    return frontierVoxelsTree;
}

double calculateOccupiedVolume(const OctreePtr octree) {
    octree->expand();
    double res = octree->getResolution();
    WorkspaceBounds w = WORK_SPACE;
    FreespaceBounds f = FREE_SPACE;
    //double res = RES_LARGE;
    double free_volume = (f.max_x-f.min_x)*(f.max_y-f.min_y)*(f.max_z-f.min_z);
    
    double volume = 0.0;
    for (octomap::OcTree::leaf_iterator it = octree->begin_leafs(), end=octree->end_leafs(); it != end; ++it) {
        if (octree->isNodeOccupied(*it)) {
            double nodeVolume = res*res*res;//std::pow(it.getSize(), 3); 
            volume += nodeVolume;
        }
    }
    double worksapce_volume = (w.max_x-w.min_x)*(w.max_y-w.min_y)*(w.max_z-w.min_z);
    double total_volume =  worksapce_volume - free_volume;

    std::cout << "unknown volume: " << volume << std::endl;
    std::cout << "workspace volume: " << worksapce_volume << std::endl;
    std::cout << "free volume: " << free_volume << std::endl;

    double percentage_explored = 100 - (volume/total_volume)*100;


    return percentage_explored;
}