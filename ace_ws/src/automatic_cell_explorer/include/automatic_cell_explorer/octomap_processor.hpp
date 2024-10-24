#ifndef OCTOMAP_PROCESSOR_HPP
#define OCTOMAP_PROCESSOR_HPP

#include <octomap/octomap.h>
#include <memory>
#include <Eigen/Dense>

using OctreePtr = std::shared_ptr<octomap::OcTree>;

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
    Eigen::Vector3d normal;
};


void createInitialSafeSpace(octomap::OcTree* received_tree);
void updatePlanningScene(octomap::OcTree* received_tree, OctreePtr unknownVoxelsTree);
void markUnknownSpaceAsObstacles(octomap::OcTree* received_tree);

OctreePtr extractUnknownOctree(const OctreePtr octree);
OctreePtr extractFreeOctree(const OctreePtr octree);
OctreePtr extractFrontierOctree(const OctreePtr octree);
OctreePtr extractFrontierOctreeInBounds(const OctreePtr octree);

std::vector<Cluster> computeClusters(std::shared_ptr<octomap::OcTree>  octree);
std::vector<Cluster> findUnknownVoxelClusters(std::shared_ptr<octomap::OcTree>  octree);

double calculateOccupiedVolume(const OctreePtr octree);

#endif // OCTOMAP_PROCESSOR_HPP