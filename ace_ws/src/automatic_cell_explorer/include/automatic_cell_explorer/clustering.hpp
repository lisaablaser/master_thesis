#ifndef CLUSTERING_HPP
#define CLUSTERING_HPP

#include <octomap/octomap.h>
#include <memory>
#include <Eigen/Dense>

struct Cluster {
    /*
        Points: connected unknonw voxels
        Frontiers: asociated frontiers to the unknonw voxels
        Center: of the unknown coxels
        Noraml: mean direction from the center to the frontiers.  
    
    */ 
    std::vector<octomap::point3d> points;  
    std::vector<octomap::point3d> frontiers;
    octomap::point3d center;
    octomap::point3d target; //frontier center
    Eigen::Vector3d target_normal; // mean frontier normal
};

std::vector<Cluster> computeClusters(std::shared_ptr<octomap::OcTree>  octree);

void computeTarget(Cluster& cluster);
octomap::point3d computeClusterCenter(const Cluster& cluster);
bool isWithinDistance(const octomap::point3d& p1, const octomap::point3d& p2);
std::vector<octomap::point3d> extractOccupiedNodes(std::shared_ptr<octomap::OcTree> octree);
void computeTargetNormal(Cluster& cluster);




#endif // CLUSTERING_HPP