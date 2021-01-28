#ifndef __RAYTRACER_H__
#define __RAYTRACER_H__

#include "../io/scene.h"
#include "../utils/ray.h"


Eigen::Vector3f ray_tracing(scene &scene,
                            Ray &ray,
                            int depth);

#endif