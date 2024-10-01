#ifndef OCTOMAP_PROCESSOR_HPP
#define OCTOMAP_PROCESSOR_HPP

#include <octomap/octomap.h>
#include <memory>

using OctreePtr = std::shared_ptr<octomap::OcTree>;


void createInitialSafeSpace(octomap::OcTree* received_tree);
void updatePlanningScene(octomap::OcTree* received_tree, OctreePtr unknownVoxelsTree);
void markUnknownSpaceAsObstacles(octomap::OcTree* received_tree);

OctreePtr extractUnknownOctree(const octomap::OcTree* octree);
OctreePtr extractFreeOctree(const octomap::OcTree* octree);
OctreePtr extractFrontierOctree(const octomap::OcTree* octree);
OctreePtr extractFrontierOctreeInBounds(const octomap::OcTree* octree);

#endif // OCTOMAP_PROCESSOR_HPP