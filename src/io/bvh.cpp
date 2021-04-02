#include <iostream>
#include <algorithm>
#include "bvh.h"
#include "../utils/utils.h"

using namespace std;
using namespace Eigen;


BVH_node::BVH_node(vector<Vector3f> &v_mat, vector<Vector3i> &f_set, 
                    vector<int> f_id, AABB &_bbox, int dimension)
{
    // end recursion if only a few faces
    int f_size = f_id.size();
    // cout << f_size << endl;
    assert(f_size > 0);
    if(f_size <= 3)
    {
        _bbox = comp_fs_bbox(v_mat, f_set, f_id);
        bbox = _bbox; face_id = f_id;
        l_child = nullptr; r_child = nullptr;
        return;
    }
    
    // split bounding box
    vector<int> l_f_id_set, r_f_id_set;

    // sort all faces
    // auto rule = [v_mat, f_set, dimension](int i, int j)->bool{
    //     return v_mat[f_set[i][0]][dimension] < v_mat[f_set[j][0]][dimension];};
    // sort(f_id.begin(), f_id.end(), rule);
    float avg_d;
    for(int i = 0; i < f_size; i++)
    {
        float curr = v_mat[f_set[f_id[i]][0]][dimension];
        avg_d += curr;
    }
    avg_d = avg_d / f_size;
    
    bool p = true;
    for(int i = 0; i < f_size; i++)
    {
        float curr = v_mat[f_set[f_id[i]][0]][dimension];
        if(curr - avg_d > 5e-3f) l_f_id_set.push_back(f_id[i]);
        else if (avg_d - curr > 5e-3f) r_f_id_set.push_back(f_id[i]);
        else
        {
            if(l_f_id_set.size() < r_f_id_set.size())
                l_f_id_set.push_back(f_id[i]);
            else 
                r_f_id_set.push_back(f_id[i]);
        }
    }

    AABB l_bbox, r_bbox;
    l_child = new BVH_node(v_mat, f_set, l_f_id_set, l_bbox, (dimension + 1) % 3);
    r_child = new BVH_node(v_mat, f_set, r_f_id_set, r_bbox, (dimension + 1) % 3);
    
    //recompute bbox
    _bbox = merge_bbox(l_bbox, r_bbox);
    bbox = _bbox;

}


obj_BVH_node::obj_BVH_node(vector<AABB> &all_bbox, vector<int> o_id,
                            AABB &_bbox, int dimension)
{
    // end recursion if only one obj
    int obj_size = o_id.size();
    assert(obj_size > 0);
    //recompute bbox
    if(obj_size == 1)
    {
        obj_id = o_id[0]; 
        _bbox = all_bbox[obj_id];
        bbox = _bbox;
        l_child = nullptr; r_child = nullptr;
        return;
    }
    
    // split bounding box
    vector<int> l_o_id_set, r_o_id_set;

    // sort all bbox
    auto rule = [all_bbox, dimension](int i, int j)->bool{
        return all_bbox[i].bbox_min[dimension] < all_bbox[j].bbox_min[dimension];};
    sort(o_id.begin(), o_id.end(), rule);
    
    for(int i = 0; i < obj_size; i++)
    {
        if(i < obj_size / 2)
            l_o_id_set.push_back(o_id[i]);
        else
            r_o_id_set.push_back(o_id[i]);
    }

    AABB l_bbox, r_bbox;
    l_child = new obj_BVH_node(all_bbox, l_o_id_set, l_bbox, (dimension + 1) % 3);
    r_child = new obj_BVH_node(all_bbox, r_o_id_set, r_bbox, (dimension + 1) % 3);

    //recompute bbox
    _bbox = merge_bbox(l_bbox, r_bbox);
    bbox = _bbox;
    obj_id = -1;

}