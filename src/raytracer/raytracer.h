#include "../io/scene.h"
#include "utils.h"
#include "cfg.h"


Eigen::Vector3f ray_tracing(scene &scene,
                            ray &ray,
                            int depth,
                            unsigned short *seed);