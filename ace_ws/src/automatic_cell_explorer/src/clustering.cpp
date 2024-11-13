#include "automatic_cell_explorer/clustering.hpp"
#include "automatic_cell_explorer/octomap_processor.hpp"
#include "automatic_cell_explorer/constants.hpp"

#include <queue>




std::vector<Cluster> computeClusters(std::shared_ptr<octomap::OcTree> octree) {
    /*
        Computes clusters of connected unknown space, in a BFS manner, restricted with a max_radius
        Also calculates frontiers, cluster centers and normals. 

    */
    double max_radius = 0.5;
    double res = RES_LARGE;
    /// TODO: avoid having to expand tree, maybe iterate taking size into account
    OctreePtr unknown_tree = extractUnknownOctree(octree);
    unknown_tree->expand();


    std::vector<octomap::point3d> unknown_nodes = extractOccupiedNodes(unknown_tree);

    std::vector<Cluster> clusters;
    std::vector<bool> visited(unknown_nodes.size(), false);

    std::cout << "Computing clusters using Euclidean Clustering" << std::endl;

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
            // In general inconsitencies in frontier extraction is observed.. 
            for (float i = current_point.x() - res; i <= current_point.x() + res; i += res) {
                for (float j = current_point.y() - res; j <= current_point.y() + res; j += res) {
                    for (float k = current_point.z() - res; k <= current_point.z() + 2*res; k += res) { 
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
    

            new_cluster.center = computeClusterCenter(new_cluster);

            for (size_t j = 0; j < unknown_nodes.size(); ++j) {
                if (!visited[j]) {
                    double distance_to_center = (new_cluster.center - unknown_nodes[j]).norm();

                    if (distance_to_center <= max_radius) {
                        to_explore.push(j);
                        visited[j] = true;
                        new_cluster.points.push_back(unknown_nodes[j]);
                    }
                }
            }
        }

        computeTargetNormal(new_cluster);
        computeTarget(new_cluster);

        clusters.push_back(new_cluster);
    }

    return clusters;
}


octomap::point3d computeClusterCenter(const Cluster& cluster) {
    double x = 0.0, y = 0.0, z = 0.0;
    for (const auto& point : cluster.points) {
        x += point.x();
        y += point.y();
        z += point.z();
    }
    size_t num_points = cluster.points.size();
    return octomap::point3d(x / num_points, y / num_points, z / num_points);
}


void computeTargetNormal(Cluster& cluster) {
    /*
        Computes target normals using the frontiers and cluster center. 
        Only works well if cluster is convex. Or if frontiers are not distributed evenly aroun the whole cluster.. 
    */
    Eigen::Vector3d accumulated_direction(0.0, 0.0, 0.0);


    for (const auto& frontier : cluster.frontiers) {
        Eigen::Vector3d direction(
            frontier.x() - cluster.center.x(),
            frontier.y() - cluster.center.y(),
            frontier.z() - cluster.center.z()
        );
        accumulated_direction += direction;
    }

    if (accumulated_direction.norm() > 0) {
        cluster.target_normal = accumulated_direction.normalized();
    } else {
        // If no frontiers are available or they are at the center
        cluster.target_normal = Eigen::Vector3d(0.0, 0.0, -1.0);
    }
}
void computeTarget(Cluster& cluster) {
    /*
        Computes target normals using the frontiers and cluster center. 
        Only works well if cluster is convex. Or if frontiers are not distributed evenly aroun the whole cluster.. 
    */
    double x = 0.0, y = 0.0, z = 0.0;
    for (const auto& point : cluster.frontiers) {
        x += point.x();
        y += point.y();
        z += point.z();
    }
    size_t num_points = cluster.frontiers.size();
    cluster.target =  octomap::point3d(x / num_points, y / num_points, z / num_points);
}


std::vector<octomap::point3d> extractOccupiedNodes(std::shared_ptr<octomap::OcTree> octree) {
    std::vector<octomap::point3d> occupied_nodes;

    for (octomap::OcTree::leaf_iterator it = octree->begin_leafs(), end = octree->end_leafs(); it != end; ++it) {
        if (octree->isNodeOccupied(*it)) {
            occupied_nodes.push_back(it.getCoordinate()); 
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


/// ##### Trash code #####

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
    Alternative appraoch for better efficency exploiting octomap. 
    Not working yet. 
    
    */
    double resolution = RES_LARGE; 
    
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

std::vector<Cluster> computeClusters2(std::shared_ptr<octomap::OcTree>  octree) {
    /*
        Computes large non-convex clusters. All unknown voxels connected are clustered toghether. 
    */
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
                    for (float k = current_point.z() - res; k <= current_point.z() + res; k += res) { //Temporary bug "fix"
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
        new_cluster.center = computeClusterCenter(new_cluster);
        computeTargetNormal(new_cluster);

        clusters.push_back(new_cluster);  
    }

    return clusters;
}
