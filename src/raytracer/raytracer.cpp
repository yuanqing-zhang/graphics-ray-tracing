#include <iostream>
#include <Eigen/Dense>
#include "raytracer.h"
#include "../utils/utils.h"

using namespace Eigen;
using namespace std;


inline vector<int> get_all_hit_box(Scene &scene, Ray &ray)
{
    vector<int> hit_ids;
    for(int i = 0; i < scene.all_objs.size(); i++)
    {
        if(ray.is_hit_bbox(scene.all_objs[i].bbox))
            hit_ids.push_back(i);
    }
    return hit_ids;
}


int get_hit_face(Scene &scene, Ray &ray, int obj_id,  
                    vector<int> cand_face_id, float &t)
{
    // intersection with all faces and return the nearest one
    float curr_t = 1e-6f;
    int face_id = -1; // -1 means not hit any face

    for(int f = 0; f < cand_face_id.size(); f++)
    {
        Vector3f A, B, C, normal;
        scene.get_face_v(obj_id, cand_face_id[f], A, B, C);
        scene.get_face_n(obj_id, cand_face_id[f], normal);

        if(ray.is_hit_triangle(A, B, C, normal, curr_t) && curr_t < t)
        {
            face_id = cand_face_id[f]; t = curr_t;
        }
    }
    return face_id;
}

vector<int> get_candidate_face(Ray &ray, BVH_node* curr_node)
{
    vector<int> res;
    if(!ray.is_hit_bbox(curr_node->bbox))
        return res;

    // leaf node
    if(curr_node->face_id.size() > 0)
        return curr_node->face_id;
    
    // non-leaf node, traverse child
    vector<int> l_res = get_candidate_face(ray, curr_node->l_child);
    vector<int> r_res = get_candidate_face(ray, curr_node->r_child);
    for(int i = 0; i < l_res.size(); i++)
        res.push_back(l_res[i]);
    for(int i = 0; i < r_res.size(); i++)
        res.push_back(r_res[i]);
    
    return res;
}


bool hit_scene(Scene &scene, Ray &ray, 
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
        // if(hit_ids[i] > 6) continue;

        float t = 100000.0;
        vector<int> cand_face = get_candidate_face(ray, 
                                        scene.all_objs[hit_ids[i]].BVH);
        int face_id = get_hit_face(scene, ray, hit_ids[i], cand_face, t);
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

vector<Ray> get_acess_light(Scene &scene, Vector3f hit_p)
{
    vector<Ray> light_rays;
    Ray r_0;
    for(int i = 0; i < scene.all_lights.size(); i++)
    {
        Vector3f A, B, C;
        scene.get_face_v(scene.all_lights[i].obj_id, 0, A, B, C);
        Vector3f o = get_rect_sample(A, B, C);
        Ray r(o, hit_p - o);
        // find light ray intersection
        Vector3f r_p, r_n;
        material r_mat;
        if(hit_scene(scene, r, r_p, r_n, r_mat) && (r_p - hit_p).norm() < 0.001f)
            light_rays.push_back(r);
        else light_rays.push_back(r_0);
    }
    return light_rays;
}



Vector3f ray_tracing(Scene &scene, Ray &ray, int depth)
{

    Vector3f hit_p, hit_n;
    material hit_mat;
    // skybox color
    // float t = (ray.d(0) + 0.85) / 1.7;
    // Vector3f bg_color = (1.0 - t) * Vector3f(1.0, 1.0, 1.0) 
    //                                + t * Vector3f(0.5, 0.7, 1.0);
    if(!hit_scene(scene, ray, hit_p, hit_n, hit_mat)) return Vector3f(0, 0, 0);
    // return if depth < 0 or hit light
    if(depth <= 0 || hit_mat.Le.norm() > 1e-6) return hit_mat.Le;

    // direct illumination
    vector<Ray> acc_ray = get_acess_light(scene, hit_p);
    Vector3f direct_light = Vector3f(0, 0, 0);
    for(int i = 0; i < acc_ray.size(); i++)
    {
        if(acc_ray[i].d.norm() > 1e-6f)
        {
            float dist = pow((acc_ray[i].o - hit_p).norm(), 2);
            float A = scene.all_lights[i].w * scene.all_lights[i].h;
            Vector3f n; scene.get_face_n(scene.all_lights[i].obj_id, 0, n);
            Vector3f le = scene.all_lights[i].Le * (acc_ray[i].d.dot(n))/ (2 * M_PI * dist) * A;
            Vector3f c_diff = (-acc_ray[i].d).dot(hit_n) * 
                              (hit_mat.Kd.cwiseProduct(le));
            Vector3f c_spec = (hit_mat.Ks.cwiseProduct(le)) *
                    (pow((-ray.d).dot(get_reflect(-acc_ray[i].d, hit_n)), hit_mat.Ns));

            direct_light += c_diff + c_spec;
        }
    }

    Vector3f diffuse = Vector3f(0, 0, 0), specular = Vector3f(0, 0, 0);
    bool is_diffuse = prob_samp_diffuse(hit_mat.Kd, hit_mat.Ks);
    // diffuse
    if(is_diffuse && hit_mat.Kd.norm() > 1e-6)
    {
        Vector3f sample_d = get_cos_hemisphere_sample(hit_n);
        Ray diff_ray(hit_p, sample_d);
        Vector3f next_c = ray_tracing(scene, diff_ray, depth - 1);
        diffuse = (sample_d.dot(hit_n)) * (hit_mat.Kd.cwiseProduct(next_c));
    }
    // specular
    else if(hit_mat.Ks.norm() > 1e-6)
    {
        Vector3f sample_d = get_spec_sample(-ray.d, hit_n, hit_mat.Ns);
        Ray diff_ray(hit_p, sample_d);
        Vector3f next_c = ray_tracing(scene, diff_ray, depth - 1);
        specular = (sample_d.dot(hit_n)) * (hit_mat.Ks.cwiseProduct(next_c)) ;
    }
    return direct_light + diffuse + specular + hit_mat.Le;

}
