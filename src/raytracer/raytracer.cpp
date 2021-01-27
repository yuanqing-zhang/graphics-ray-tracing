#include <iostream>
#include <Eigen/Dense>
#include "raytracer.h"
#include "../utils/utils.h"

using namespace Eigen;
using namespace std;


inline vector<int> get_all_hit_box(scene &scene, ray &ray)
{
    vector<int> hit_ids;
    for(int i = 0; i < scene.all_objs.size(); i++)
    {
        if(ray.is_hit_bbox(scene.all_objs[i].bbox))
            hit_ids.push_back(i);
    }
    return hit_ids;
}


inline int get_hit_face(scene &scene, ray &ray, int obj_id, float &z)
{
    // intersection with all faces and return the nearest one
    Vector3f p = Vector3f(0, 0, 0); // intersect point p
    int face_id = -1;

    for(int f = 0; f < scene.all_objs[obj_id].fv_set.size(); f++)
    {
        Vector3f A, B, C, normal;
        scene.get_face_v(obj_id, f, A, B, C);
        scene.get_face_n(obj_id, f, normal);

        if(ray.is_hit_triangle(A, B, C, normal, p) && p(2) > z)
        {
            face_id = f; z = p(2);
        }
    }
    return face_id;
}


bool hit_scene(scene &scene, ray &ray, Vector3f &color)
{
    // ray intersection with all AABBs
    vector<int> hit_ids = get_all_hit_box(scene, ray);
    // not hit any bounding box
    if(hit_ids.size() < 1) return false;

    //暂时去掉球
    bool is_hit = false;
    float max_z = -10000;
    for(int i = 0; i < hit_ids.size(); i++)
    {
        if(hit_ids[i] > 6) continue;

        float z = -10000;
        int face_id = get_hit_face(scene, ray, hit_ids[i], z);
        if(face_id >= 0 && z > max_z)
        {
            is_hit = true;
            max_z = z;
            Vector3f normal;
            scene.get_face_n(hit_ids[i], face_id, normal);
            color = 0.5 * (normal + Vector3f(1, 1, 1));
        }
    }

    return is_hit;
}


Vector3f ray_tracing(scene &scene, ray &ray,
                     int depth, const int max_depth, unsigned short *seed)
{
    float t = (ray.d(0) + 0.85) / 1.7;
    Vector3f bg_color = (1.0 - t) * Vector3f(1.0, 1.0, 1.0) + t * Vector3f(0.5, 0.7, 1.0);
    if(depth >= max_depth) return bg_color;

    Vector3f trace_color = Vector3f(0, 0, 0);
    if(hit_scene(scene, ray, trace_color))
    {
        return trace_color;
    }

    return bg_color;
}
