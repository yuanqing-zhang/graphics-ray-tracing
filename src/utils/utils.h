#ifndef __UTILS_H__
#define __UTILS_H__

#include <Eigen/Dense>
#include <cmath>
#include "cfg.h"
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

Eigen::Vector3f get_cos_hemisphere_sample(Eigen::Vector3f n)
{
    float r1 = 2 * M_PI * drand48(), r2 = drand48();

    Eigen::Vector3f U = (fabs(n(0)) > .1 ? Eigen::Vector3f(0, 1, 0) : 
                                Eigen::Vector3f(1, 0, 0)).cross(n);
    U.normalize(); 
    Eigen::Vector3f V = n.cross(U);
    V.normalize();
    // get cosine-weighted hemisphere lobe sample direction
    Eigen::Vector3f D = U * cos(r1) * sqrt(r2) + 
                        V * sin(r1) * sqrt(r2) + 
                        n * sqrt(1 - r2);
    return D;
}

Eigen::Vector3f get_spec_sample(Eigen::Vector3f in, Eigen::Vector3f n, float Ns)
{
    Eigen::Vector3f reflect = get_reflect(in, n);

    float r1 = 2 * M_PI * drand48(), r2 = 1 - pow(drand48(), 1.0 / (Ns + 1));
    Eigen::Vector3f U = (fabs(reflect(0)) > .1 ? Eigen::Vector3f(0, 1, 0) : 
                                Eigen::Vector3f(1, 0, 0)).cross(reflect);
    U.normalize();
    Eigen::Vector3f V = reflect.cross(U);
    V.normalize();
    Eigen::Vector3f D = U * cos(r1) * sqrt(r2) + 
                        V * sin(r1) * sqrt(r2) + 
                        reflect * sqrt(1 - r2);
    D.normalize();
    return D;

}

bool prob_samp_diffuse(Eigen::Vector3f Kd, Eigen::Vector3f Ks)
{
    float pkd = fmax(1e-6f, Kd.norm());
    float pks = fmax(1e-6f, Ks.norm());
    float p = drand48();
    return p < (pkd / (pkd + pks));
}

Eigen::Vector3f get_rect_sample(Eigen::Vector3f A, 
                                Eigen::Vector3f B, 
                                Eigen::Vector3f C)
{
    Eigen::Vector3f axis1 = A - B, axis2 = C - B;
    float u = drand48(), v = drand48();
    return B + u * axis1 + v * axis2;
}

#endif