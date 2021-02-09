#include <iostream>
#include <algorithm>
#include "bvh.h"

using namespace std;
using namespace Eigen;


AABB comp_fs_bbox(vector<Vector3f> &v_mat, 
                    vector<Vector3i> &f_set, vector<int> f_id)
{
    AABB bbox;
    for(int axis = 0; axis < 3; axis++)
    {
        float min = 10000, max = -10000;
        for(int f = 0; f < f_id.size(); f++)
        {
            Vector3i curr_f = f_set[f_id[f]];
            for(int v = 0; v < 3; v++)
            {
                if(v_mat[curr_f[v]](axis) < min)
                    min = v_mat[curr_f[v]](axis);
                if(v_mat[curr_f[v]](axis) > max)
                    max = v_mat[curr_f[v]](axis);
            }
        }
        bbox.bbox_min(axis) = min;
        bbox.bbox_max(axis) = max;
    }
    return bbox;
}


BVH_node::BVH_node(vector<Vector3f> &v_mat, vector<Vector3i> &f_set, 
                    vector<int> f_id, AABB _bbox, int dimension)
{
    // end recursion if only a few faces
    int f_size = f_id.size();
    assert(f_size > 0);
    if(f_size <= 6)
    {
        bbox = _bbox; face_id = f_id;
        l_child = nullptr; r_child = nullptr;
        return;
    }
    
    // split bounding box
    vector<int> l_f_id_set, r_f_id_set;
    AABB l_bbox, r_bbox;

    // sort all faces
    auto rule = [v_mat, f_set, dimension](int i, int j)->bool{
        return v_mat[f_set[i][0]][dimension] < v_mat[f_set[j][0]][dimension];};
    sort(f_id.begin(), f_id.end(), rule);
    

    for(int i = 0; i < f_size; i++)
    {
        if(i < f_size / 2)
            l_f_id_set.push_back(f_id[i]);
        else
            r_f_id_set.push_back(f_id[i]);
    }
    //recompute bbox
    l_bbox = comp_fs_bbox(v_mat, f_set, l_f_id_set);
    r_bbox = comp_fs_bbox(v_mat, f_set, r_f_id_set);

    // cout << "l " << l_f_id_set.size() << " " << l_bbox.bbox_min[dimension] << endl;
    // cout << "r " << r_f_id_set.size() << " " << r_bbox.bbox_min[dimension] << endl;

    bbox = _bbox;
    l_child = new BVH_node(v_mat, f_set, l_f_id_set, l_bbox, (dimension + 1) % 3);
    r_child = new BVH_node(v_mat, f_set, r_f_id_set, r_bbox, (dimension + 1) % 3);

}