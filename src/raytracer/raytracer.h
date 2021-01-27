#ifndef __RAYTRACER_H__
#define __RAYTRACER_H__

#include "../io/scene.h"
#include "../utils/ray.h"


Eigen::Vector3f ray_tracing(scene &scene,
                            ray &ray,
                            int depth,
                            const int max_depth,
                            unsigned short *seed);

#endif