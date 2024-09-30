
// Workspace Boundries

double OFFSET_Z = 0.72;

double MIN_X = -2.0; double MAX_X = 2.0;
double MIN_Y = -2.0; double MAX_Y = 2.0;
double MIN_Z = -2.0 + OFFSET_Z; double MAX_Z = 2.0 + OFFSET_Z; 

// Initial Free Space

double IN_MIN_X = -1.0; double IN_MAX_X = 1.0;
double IN_MIN_Y = -1.0; double IN_MAX_Y = 1.0;
double IN_MIN_Z = -1.0 + OFFSET_Z; double IN_MAX_Z = 1.0 + OFFSET_Z; 

int octomap_res = 0.01;