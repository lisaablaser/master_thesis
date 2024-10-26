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
    octomap::point3d target; 
    Eigen::Vector3d target_normal;
};

std::vector<Cluster> computeClusters(std::shared_ptr<octomap::OcTree>  octree);
std::vector<Cluster> findUnknownVoxelClusters(std::shared_ptr<octomap::OcTree>  octree);



#endif // CLUSTERING_HPP