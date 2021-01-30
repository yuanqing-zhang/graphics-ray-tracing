#ifndef __RAY_H__
#define __RAY_H__

#include <Eigen/Dense>
#include "AABB.h"
#include <iostream>

class Ray
{
public:
    Eigen::Vector3f o; // origin
    Eigen::Vector3f d; // direction

    Ray(Eigen::Vector3f _o, Eigen::Vector3f _d):o(_o), d(_d){};
    Eigen::Vector3f at(float t){return o + t * d; };

    bool is_hit_bbox(AABB &bbox)
    {
        // slab method
        float t_min = 1e-6f;
        float t_max = 100000.0;

        for(int i = 0; i < 3; i++)
        {
            if(fabsf(d(i)) < 1e-6f)
            {
                if (o(i) < bbox.bbox_min(i) || o(i) > bbox.bbox_max(i))
                    return false;
            }
            float t1 = (bbox.bbox_min(i) - o(i)) / d(i);
            float t2 = (bbox.bbox_max(i) - o(i)) / d(i);
            // make t1 <= t2
            if(t1 > t2) { float tmp = t1; t1 = t2; t2 = tmp; }
            
            if(t1 > t_min) t_min = t1;
            if(t2 < t_max) t_max = t2;

            if (t_min > t_max) return false;
        }
        return true;
    };

    bool is_hit_triangle(Eigen::Vector3f A, 
                         Eigen::Vector3f B, 
                         Eigen::Vector3f C,
                         Eigen::Vector3f normal,
                         float &t)
    {
        // find the intersection p
        if (fabsf(normal.dot(d)) < 1e-6f) return false; // paralell

        t = (normal.dot(A) - normal.dot(o)) / (normal.dot(d));
        if(t < 0) return false;
        Eigen::Vector3f p = at(t);

        // check whether p in triangle
        if((B - A).cross(p - A).dot(normal) < 0) return false;
        if((C - B).cross(p - B).dot(normal) < 0) return false;
        if((A - C).cross(p - C).dot(normal) < 0) return false;
        return true;
    };
};

#endif