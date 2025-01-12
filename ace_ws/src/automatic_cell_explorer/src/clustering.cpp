#include "automatic_cell_explorer/clustering.hpp"
#include "automatic_cell_explorer/octomap_processor.hpp"
#include "automatic_cell_explorer/constants.hpp"

#include <queue>




std::vector<Cluster> computeClusters(std::shared_ptr<octomap::OcTree> octree) {
    /*
        Computes clusters of connected unknown space in a BFS manner, restricted with a max_radius.
        Also calculates frontiers attached to each cluster and cluster. 

    */
    double max_radius = 0.5;
    double res = RES_LARGE;
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
            for (float i = current_point.x() - res; i <= current_point.x() + res; i += res) {
                for (float j = current_point.y() - res; j <= current_point.y() + res; j += res) {
                    for (float k = current_point.z() - res; k <= current_point.z() + 2*res; k += res) { 
                        if (i == current_point.x() && j == current_point.y() && k == current_point.z()) continue; 
                        octomap::OcTreeKey neighbor_key;;
                        octree->coordToKeyChecked(octomap::point3d(i, j, k), neighbor_key);

                        auto node = octree->search(neighbor_key);
                        
                        if (node != nullptr && !octree->isNodeOccupied(node)) {
                            
                            new_cluster.frontiers.push_back(octree->keyToCoord(neighbor_key));
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

void removeClustersWhithoutFrontiers(std::vector<Cluster>& clusters){
    clusters.erase(
        std::remove_if(
            clusters.begin(),
            clusters.end(),
            [](const Cluster& cluster) {
                return cluster.frontiers.empty(); 
            }
        ),
        clusters.end()
    );
}

void sortClustersByFrontiers(std::vector<Cluster>& clusters) {
    std::sort(
        clusters.begin(),
        clusters.end(),
        [](const Cluster& a, const Cluster& b) {
            return a.frontiers.size() > b.frontiers.size(); 
        }
    );
}

void sortClustersByUnknowns(std::vector<Cluster>& clusters) {
    std::sort(
        clusters.begin(),
        clusters.end(),
        [](const Cluster& a, const Cluster& b) {
            return a.points.size() > b.points.size(); 
        }
    );
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

