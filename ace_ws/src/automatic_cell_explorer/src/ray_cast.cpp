
// Input: sensor_state, fov_h, fov_v, res, octree
// Output: Dict[end_point, stautus(unknown/free/occupied)]

#include <octomap/octomap.h>
#include <eigen3/Eigen/Geometry>

using namespace octomap;
enum class Type {Unknown, Free, Occupied};

struct Ray {
    Eigen::Vector3d start;  // Ray start (sensor origin)
    Eigen::Vector3d end;    // Ray end (hit point or max range)
    Type state;             // Whether the ray hits an unknown, free or occupied voxel
};

void castRay(OcTree& octree, const point3d& origin, const point3d& directionP, Ray ray, double maxRange){



}