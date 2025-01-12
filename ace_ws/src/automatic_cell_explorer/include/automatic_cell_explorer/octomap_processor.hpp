#ifndef OCTOMAP_PROCESSOR_HPP
#define OCTOMAP_PROCESSOR_HPP

#include <octomap/octomap.h>
#include <memory>
#include <Eigen/Dense>

using OctreePtr = std::shared_ptr<octomap::OcTree>;



void createInitialSafeSpace(octomap::OcTree* received_tree);
void markUnknownSpaceAsObstacles(octomap::OcTree* received_tree);

OctreePtr extractUnknownOctree(const OctreePtr octree);
OctreePtr extractFreeOctree(const OctreePtr octree);
OctreePtr extractFrontierOctree(const OctreePtr octree);
OctreePtr extractFrontierOctreeInBounds(const OctreePtr octree);

double calculateOccupiedVolume(const OctreePtr octree);

#endif // OCTOMAP_PROCESSOR_HPP