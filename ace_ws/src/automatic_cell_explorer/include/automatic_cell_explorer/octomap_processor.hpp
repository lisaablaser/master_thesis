#ifndef OCTOMAP_PROCESSOR_HPP
#define OCTOMAP_PROCESSOR_HPP

#include <octomap/octomap.h>
#include <memory>

using OctrePtr = std::shared_ptr<octomap::OcTree>;


void createInitialSafeSpace(octomap::OcTree* received_tree, double x, double y, double z, double resolution);
void markUnknownSpaceAsObstacles(octomap::OcTree* received_tree, double x, double y, double z, double resolution);

OctrePtr extractUnknownOctree(const octomap::OcTree* octree);
OctrePtr extractFreeOctree(const octomap::OcTree* octree);
OctrePtr extractFrontierOctree(const octomap::OcTree* octree);

#endif // OCTOMAP_PROCESSOR_HPP