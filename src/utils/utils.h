#ifndef __UTILS_H__
#define __UTILS_H__

#include "cfg.h"
#include <Eigen/Dense>
#include "ray.h"

void save_image(Eigen::Vector3f* image, const std::string &scene_name, renderCfg cfg);

double tent_filter(double x);


double rand_() 
{
    // returns a random real in [0,1)
    return rand() / (RAND_MAX + 1.0);
}

Eigen::Vector3f get_reflect(const Ray &ray, Eigen::Vector3f n)
{
    return ray.d - 2 * n.dot(ray.d) * n;
}

Eigen::Vector3f get_random_diffuse(Eigen::Vector3f o, Eigen::Vector3f n)
{
    float r1 = 2 * pi * rand_(), r2 = rand_();

    Eigen::Vector3f U = (fabs(n(0)) > .1 ? 
                        Eigen::Vector3f(0, 1, 0) : Eigen::Vector3f(1, 0, 0)).cross(n);
    U.normalize(); 
    Eigen::Vector3f V = n.cross(U);
    V.normalize();

    Eigen::Vector3f D = U * cos(r1) * sqrt(r2) + V * sin(r1) * sqrt(r2) + n * sqrt(1 - r2);
    D.normalize();

    return D;

}

#endif