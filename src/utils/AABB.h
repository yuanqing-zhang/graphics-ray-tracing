#ifndef __AABB_H__
#define __AABB_H__

#include <Eigen/Dense>


// axis-aligned bounding box
class AABB
{
public:
    Eigen::Vector3f bbox_min;
    Eigen::Vector3f bbox_max;

    AABB()
    {
        bbox_min = Eigen::Vector3f(0, 0, 0);
        bbox_max = Eigen::Vector3f(0, 0, 0);
    };

};

#endif