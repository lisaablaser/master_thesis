#include <cstddef>


// Tight: -0.25, -0.13, 0.045, 0.4, 0.4, 1.28


// UR5: 0.85 m, Sensor: <box size="0.07271 0.27794 0.073"/>: 
struct WorkspaceBounds {
    double min_x = -(0.85 + 0.08);
    double min_y = -(0.85 + 0.08);
    double min_z = -0.5;
    double max_x = (0.85 + 0.08);
    double max_y = (0.85 + 0.08);
    double max_z = (1.3 + 0.08);
};

struct FreespaceBounds {
    double min_x = -0.5;
    double min_y = -0.4; 
    double min_z = 0.045;
    double max_x =  0.75;
    double max_y = 0.4;
    double max_z = 1.2 + 0.08;
};


struct OrigoOffset {
    double x = 0.0;
    double y = 0.0;
    double z = 0.755;
};

const OrigoOffset ORIGO;     
const WorkspaceBounds WORK_SPACE;  
const FreespaceBounds FREE_SPACE;
const double RES_SMALL = 0.01; 
const double RES_LARGE = 0.1; 
const double MAX_RANGE = 2.0;
const double FOV_H     = 64;
const double FOV_V     = 36;
const size_t N_SAMPLES = 10;