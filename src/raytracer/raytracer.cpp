#include <iostream>
#include "Eigen/Dense"
#include "raytracer.h"
#include "cfg.h"

using namespace Eigen;
using namespace std;


Vector3f ray_tracing(scene &scene,
                     ray &ray,
                     int depth,
                     unsigned short *seed)
{
    // visualization ray 
    // float t = (ray.d(0) + 0.86) / 1.7;
    // return (1.0 - t) * Vector3f(1, 1, 1) + t * Vector3f(0.1, 0.1, 0.1);

    
}
