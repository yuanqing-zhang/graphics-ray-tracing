#include <iostream>
#include <Eigen/Dense>
#include "raytracer.h"
#include "../utils/utils.h"

using namespace Eigen;
using namespace std;


inline vector<int> get_all_hit_box(scene &scene, Ray &ray)
{
    vector<int> hit_ids;
    for(int i = 0; i < scene.all_objs.size(); i++)
    {
        if(ray.is_hit_bbox(scene.all_objs[i].bbox))
            hit_ids.push_back(i);
    }
    return hit_ids;
}


inline int get_hit_face(scene &scene, Ray &ray, int obj_id, float &t)
{
    // intersection with all faces and return the nearest one
    float curr_t;
    int face_id = -1; // -1 means not hit any face

    for(int f = 0; f < scene.all_objs[obj_id].fv_set.size(); f++)
    {
        Vector3f A, B, C, normal;
        scene.get_face_v(obj_id, f, A, B, C);
        scene.get_face_n(obj_id, f, normal);

        if(ray.is_hit_triangle(A, B, C, normal, curr_t) && curr_t < t)
        {
            face_id = f; t = curr_t;
        }
    }
    return face_id;
}


inline bool hit_scene(scene &scene, Ray &ray, 
                Vector3f &hit_p, Vector3f &hit_n, material &hit_mat)
{
    // ray intersection with all AABBs
    vector<int> hit_ids = get_all_hit_box(scene, ray);
    // not hit any bounding box
    if(hit_ids.size() < 1) return false;

    bool is_hit = false;
    float hit_t = 100000.0;
    int hit_obj, hit_face;
    for(int i = 0; i < hit_ids.size(); i++)
    {
        //暂时去掉球
        if(hit_ids[i] > 6) continue;

        float t = 100000.0;
        int face_id = get_hit_face(scene, ray, hit_ids[i], t);
        if(face_id >= 0 && t < hit_t)
        {
            hit_t = t; is_hit = true; hit_obj = i; hit_face = face_id;
        }
    }
    if(is_hit)
    {
        hit_p = ray.at(hit_t);
        scene.get_face_n(hit_ids[hit_obj], hit_face, hit_n);
        scene.get_obj_mat(hit_ids[hit_obj], hit_mat);
    }
    return is_hit;
}


Vector3f ray_tracing(scene &scene, Ray &ray, int depth)
{

    Vector3f hit_p, hit_n;
    material hit_mat;
    // skybox color
    // float t = (ray.d(0) + 0.85) / 1.7;
    // Vector3f bg_color = (1.0 - t) * Vector3f(1.0, 1.0, 1.0) 
    //                                + t * Vector3f(0.5, 0.7, 1.0);
    if(!hit_scene(scene, ray, hit_p, hit_n, hit_mat)) return Vector3f(0, 0, 0);

    if(depth <= 0 && drand48() < 0.3) return hit_mat.Le;

    Vector3f diffuse = Vector3f(0, 0, 0);
    Vector3f specular = Vector3f(0, 0, 0);
    float rand_r = -1;
    if(hit_mat.Kd.norm() > 1e-6 && hit_mat.Ks.norm() > 1e-6)
    {
        rand_r = drand48();
    }
    if(hit_mat.Kd.norm() > 1e-6 && rand_r < 0.5)
    {
        Ray diff_ray(hit_p, get_random_diffuse(hit_n));
        Vector3f next_d = ray_tracing(scene, diff_ray, depth - 1);
        diffuse = hit_mat.Kd.cwiseProduct(next_d);
    }
    if(hit_mat.Ks.norm() > 1e-6 && (rand_r > 0.5 || rand_r == -1))
    {
        Ray spec_ray(hit_p, get_reflect(ray, hit_n));
        Vector3f next_s = ray_tracing(scene, spec_ray, depth - 1);
        specular = hit_mat.Ks.cwiseProduct(next_s);
    }
    return diffuse + specular + hit_mat.Le;

}
