#ifndef __BVH_H__
#define __BVH_H__

#include <Eigen/Dense>
#include <vector>

#include "../utils/AABB.h"

class BVH_node
{
public:
    AABB bbox;
    std::vector<int> face_id;
    BVH_node *l_child, *r_child;

    BVH_node(){};
    BVH_node(std::vector<Eigen::Vector3f> &v_mat,
                std::vector<Eigen::Vector3i> &f_set,
                std::vector<int> f_id,
                AABB &_bbox);
};


class obj_BVH_node
{
public:
    AABB bbox;
    obj_BVH_node *l_child, *r_child;
    int obj_id;

    obj_BVH_node(){};
    obj_BVH_node(std::vector<AABB> &all_bbox, 
                 std::vector<int> all_obj_id,
                 AABB &_bbox, int dimension);
};



#endif


