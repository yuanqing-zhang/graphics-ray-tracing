#ifndef __UTILS_H__
#define __UTILS_H__

#include "cfg.h"
#include <Eigen/Dense>
#include "ray.h"

void save_image(Eigen::Vector3f* image, const std::string &scene_name, renderCfg cfg);

double tent_filter(double x);


float clamp(float x){ return x < 0 ? 0 : x > 1 ? 1 : x; }

Eigen::Vector3f get_reflect(Eigen::Vector3f i, Eigen::Vector3f n)
{
    Eigen::Vector3f reflect = -i + 2 * n.dot(i) * n;
    reflect.normalize();
    return reflect;
}

Eigen::Vector3f get_random_diffuse(Eigen::Vector3f n)
{
    float r1 = 2 * M_PI * drand48(), r2 = drand48();

    Eigen::Vector3f U = (fabs(n(0)) > .1 ? Eigen::Vector3f(0, 1, 0) : 
                                Eigen::Vector3f(1, 0, 0)).cross(n);
    U.normalize(); 
    Eigen::Vector3f V = n.cross(U);
    V.normalize();

    Eigen::Vector3f D = U * cos(r1) * sqrt(r2) + 
                        V * sin(r1) * sqrt(r2) + 
                        n * sqrt(1 - r2);
    D.normalize();
    return D;
}

Eigen::Vector3f get_random_specular(Eigen::Vector3f i, Eigen::Vector3f n)
{
    Eigen::Vector3f reflect = get_reflect(i, n);
    float radius = reflect.dot(n);
    Eigen::Vector3f ray_d = reflect + get_random_diffuse(reflect) * radius;
    ray_d.normalize();
    return ray_d;
}

#endif