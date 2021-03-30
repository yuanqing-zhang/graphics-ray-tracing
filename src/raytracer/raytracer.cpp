#include <iostream>
#include <Eigen/Dense>

#include "raytracer.h"
#include "../utils/utils.h"

using namespace Eigen;
using namespace std;


vector<int> get_all_hit_box(Ray &ray, obj_BVH_node* curr_node)
{
    vector<int> res;
    if(!ray.is_hit_bbox(curr_node->bbox))
        return res;

    // leaf node
    if(curr_node->obj_id >= 0)
    {
        res.push_back(curr_node->obj_id);
        return res;
    }
    
    // non-leaf node, traverse child
    vector<int> l_res = get_all_hit_box(ray, curr_node->l_child);
    vector<int> r_res = get_all_hit_box(ray, curr_node->r_child);
    res.insert(res.end(), l_res.begin(), l_res.end());
    res.insert(res.end(), r_res.begin(), r_res.end());
    
    return res;
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
    res.insert(res.end(), l_res.begin(), l_res.end());
    res.insert(res.end(), r_res.begin(), r_res.end());
    
    return res;
}


bool hit_scene(Scene &scene, Ray &ray, 
                Vector3f &hit_p, Vector3f &hit_n, material &hit_mat, Vector3f &tex_color)
{
    // ray intersection with all AABBs
    vector<int> hit_ids = get_all_hit_box(ray, scene.obj_BVH);
    // not hit any bounding box
    if(hit_ids.size() < 1) return false;

    bool is_hit = false;
    float hit_t = 100000.0;
    int hit_obj, hit_face;
    for(int i = 0; i < hit_ids.size(); i++)
    {
        // if(hit_ids[i] != 37 && hit_ids[i] != 0 && hit_ids[i] != 39) continue;

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
        scene.get_obj_mat(hit_ids[hit_obj], hit_mat);
        scene.get_n_tex(hit_ids[hit_obj], hit_face, hit_p, hit_n, hit_mat, tex_color);
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
        Vector3f r_p, r_n, tex;
        material r_mat;
        if(hit_scene(scene, r, r_p, r_n, r_mat, tex) && (r_p - hit_p).norm() < 0.001f)
            light_rays.push_back(r);
        else light_rays.push_back(r_0);
    }
    return light_rays;
}


Vector3f ray_tracing(Scene &scene, Ray &ray, int depth)
{

    Vector3f hit_p, hit_n;
    material hit_mat;
    Vector3f tex_color = Vector3f(1, 1, 1);
    // skybox color
    if(!hit_scene(scene, ray, hit_p, hit_n, hit_mat, tex_color))
    {
        if(scene.envir_map.data == nullptr)
            return Vector3f(0, 0, 0);
        return ray.get_envir_color(scene.envir_map);
    }       
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
                                ((hit_mat.Kd).cwiseProduct(tex_color).cwiseProduct(le));
            Vector3f c_spec = (hit_mat.Ks.cwiseProduct(le)) *
                                (pow((-ray.d).dot(get_reflect(-acc_ray[i].d, hit_n)), hit_mat.Ns));

            direct_light += c_diff + c_spec;
        }
    }

    // return direct_light;
    Vector3f diffuse = Vector3f(0, 0, 0), specular = Vector3f(0, 0, 0);
    bool is_diffuse = prob_samp_diffuse(hit_mat.Kd, hit_mat.Ks);
    // diffuse
    if(is_diffuse && hit_mat.Kd.norm() > 1e-6)
    {
        Vector3f sample_d = get_cos_hemisphere_sample(hit_n);
        Ray diff_ray(hit_p, sample_d);
        Vector3f next_c = ray_tracing(scene, diff_ray, depth - 1);
        diffuse = (sample_d.dot(hit_n)) * 
                    ((hit_mat.Kd).cwiseProduct(tex_color).cwiseProduct(next_c));
    }
    // specular
    else if(hit_mat.Ks.norm() > 1e-6)
    {
        Vector3f sample_d = get_spec_sample(-ray.d, hit_n, hit_mat.Ns);
        Ray diff_ray(hit_p, sample_d);
        Vector3f next_c = ray_tracing(scene, diff_ray, depth - 1);
        specular += (hit_mat.Ks).cwiseProduct(next_c);
    }

    // mix color and texture
    return direct_light + diffuse + specular + hit_mat.Le;

}
