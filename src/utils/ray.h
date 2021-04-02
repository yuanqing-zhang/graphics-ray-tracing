#ifndef __RAY_H__
#define __RAY_H__

#include <cmath>
#include <Eigen/Dense>
#include <iostream>
#include <opencv2/opencv.hpp>
#include "AABB.h"

class Ray
{
public:
    Eigen::Vector3f o; // origin
    Eigen::Vector3f d; // direction

    Ray(){o = Eigen::Vector3f(0, 0, 0); d = Eigen::Vector3f(0, 0, 0);};
    Ray(Eigen::Vector3f _o, Eigen::Vector3f _d):o(_o), d(_d){d.normalize();};
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
        float nd = normal.dot(d);
        if (fabsf(normal.dot(d)) < 1e-3f) return false; // paralell
        if (nd > 0) return false; // back face

        // find the intersection p
        t = (normal.dot(A) - normal.dot(o)) / nd;
        if(t < 1e-6f) return false;
        Eigen::Vector3f p = at(t);

        // check whether p in triangle
        if((B - A).cross(p - A).dot(normal) < 0) return false;
        if((C - B).cross(p - B).dot(normal) < 0) return false;
        if((A - C).cross(p - C).dot(normal) < 0) return false;
        return true;
    }; 

    Eigen::Vector3f get_envir_color(cv::Mat &envir_map)
    {
        // get intersection point
        Eigen::Vector3f op = -o;
        float rad = 1200;
        float b=op.dot(d);
        float det = sqrt(b * b - op.dot(op) + rad * rad);
        float t = b - det;
        if(t < 0) t = b + det;
        Eigen::Vector3f p = at(t);
        p = p / rad;

        // change to u, v
        float theta = acos(-p(1));
        float phi = atan2(-p(2), p(0)) + M_PI;

        float u = envir_map.rows * (1 - theta / M_PI);
        float v = phi / (2 * M_PI) - 0.8;
        v = envir_map.cols * (1 - (v - floor(v)));

        cv::Vec3f pixel = envir_map.at<cv::Vec3f>(u, v);
        Eigen::Vector3f envir_color(pixel[2], pixel[1], pixel[0]);
        return envir_color;
    }
};

#endif