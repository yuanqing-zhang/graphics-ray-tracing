#ifndef __UTILS_H__
#define __UTILS_H__

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <cmath>
#include "cfg.h"
#include "ray.h"
#include "AABB.h"

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
    float pkd = fmax(1e-3f, Kd.norm());
    float pks = fmax(1e-3f, Ks.norm());
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


// AABB comp_fs_bbox(std::vector<Eigen::Vector3f> &v_mat, 
//                     std::vector<Eigen::Vector3i> &f_set, std::vector<int> f_id)
// {
//     AABB bbox;
//     for(int axis = 0; axis < 3; axis++)
//     {
//         float min = 10000, max = -10000;
//         for(int v = 0; v < 3; v++)
//         {
//             // sort all faces
//             auto rule = [v_mat, f_set, v, axis](int i, int j)->bool{
//                 return v_mat[f_set[i][v]][axis] < v_mat[f_set[j][v]][axis];};
//             sort(f_id.begin(), f_id.end(), rule);

//             float curr_min = v_mat[f_set[f_id[0]][v]](axis);
//             float curr_max = v_mat[f_set[f_id[f_id.size() - 1]][v]](axis);
//             if(curr_min < min)
//                 min = curr_min;
//             if(curr_max > max)
//                 max = curr_max;
//         }
//         bbox.bbox_min(axis) = min;
//         bbox.bbox_max(axis) = max;
//     }
//     return bbox;
// }
AABB comp_fs_bbox(std::vector<Eigen::Vector3f> &v_mat, 
                    std::vector<Eigen::Vector3i> &f_set, std::vector<int> f_id)
{
    AABB bbox;
    for(int axis = 0; axis < 3; axis++)
    {
        float min = 10000, max = -10000;
        for(int f = 0; f < f_id.size(); f++)
        {
            Eigen::Vector3i curr_f = f_set[f_id[f]];
            for(int v = 0; v < 3; v++)
            {
                if(v_mat[curr_f[v]](axis) < min)
                    min = v_mat[curr_f[v]](axis);
                if(v_mat[curr_f[v]](axis) > max)
                    max = v_mat[curr_f[v]](axis);
            }
        }
        bbox.bbox_min(axis) = min - 1e-6f;
        bbox.bbox_max(axis) = max + 1e-6f;
    }
    return bbox;
}


AABB merge_bbox(AABB &a, AABB &b)
{
    AABB merge;
    for(int axis = 0; axis < 3; axis++)
    {
        merge.bbox_min[axis] = fmin(a.bbox_min[axis], b.bbox_min[axis]);
        merge.bbox_max[axis] = fmax(a.bbox_max[axis], b.bbox_max[axis]);
    }
    return merge;
}



#endif