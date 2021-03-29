#ifndef __SCENE_H__
#define __SCENE_H__

#include <iostream>
#include <map>
#include <cmath>
#include <vector>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>

#include "../utils/AABB.h"
#include "bvh.h"



class material
{
public:
    
    Eigen::Vector3f Kd; // diffuse
    Eigen::Vector3f Ka; // ambient
    Eigen::Vector3f Ks; // specular
    Eigen::Vector3f Le; // emittance

    float Ns; // specular exponent, 0 ~ 1000
    float Ni; // ptical density, 0.001 ~ 10

    cv::Mat texture;

	material() 
    {
        Kd = Eigen::Vector3f(0, 0, 0); Ka = Eigen::Vector3f(0, 0, 0);
        Ks = Eigen::Vector3f(0, 0, 0); Le = Eigen::Vector3f(0, 0, 0);
        Ns = 0; Ni = 1;
	}
};


class obj
{
// obj format:
//List of geometric vertices
// v x, y, z [,w]
// List of texture coordinates
// vt u, v [,w]
// List of vertex normals
// vn x, y, z
// Polygonal face element(normal/texture is optimal)
// f v1[/vt1/vn1] v2[/vt2/vn2] v3[/vt3/vn3]
public:

    std::vector<Eigen::Vector3i> f_set;  // index of vertices
    std::vector<Eigen::Vector3i> fn_set;  // index of normals
    std::vector<Eigen::Vector3i> ft_set;  // index of texture coordinates

    std::vector<Eigen::Vector3f> f_normal; // face nromal (average of vertex normal)

    std::string mat_name;
    AABB bbox;
    BVH_node* BVH;

    obj(){BVH = nullptr;};
};

// area light(quad)
class light
{
public:
    int obj_id;
    float w, h;
    Eigen::Vector3f Le;
    light(){w = 0; h = 0; obj_id = -1;};
};


class Scene
{
public:

    std::vector<Eigen::Vector3f> v_mat;   // all vertices
    std::vector<Eigen::Vector3f> vn_mat;  // all vertex normals
    std::vector<Eigen::Vector2f> vt_mat;  // all texture coordinates

    std::vector<obj> all_objs;
    std::map<std::string, material> all_materials;
    std::vector<light> all_lights;
    cv::Mat envir_map;

    obj_BVH_node* obj_BVH;

    Scene(){obj_BVH = nullptr;};
    void load_scene(const std::string &scene_name);
    void load_mtl(const std::string &scene_dir,const std::string &mtl_name);
    void build_BVH();
    
    
    // utils
    void get_face_v(int obj_id, 
                    int face_id,
                    Eigen::Vector3f &A, 
                    Eigen::Vector3f &B, 
                    Eigen::Vector3f &C)
    {
        A << v_mat[all_objs[obj_id].f_set[face_id](0)];
        B << v_mat[all_objs[obj_id].f_set[face_id](1)];
        C << v_mat[all_objs[obj_id].f_set[face_id](2)];
    };
    void get_face_vn(int obj_id, 
                     int face_id,
                     Eigen::Vector3f &A, 
                     Eigen::Vector3f &B, 
                     Eigen::Vector3f &C)
    {
        A << vn_mat[all_objs[obj_id].fn_set[face_id](0)];
        B << vn_mat[all_objs[obj_id].fn_set[face_id](1)];
        C << vn_mat[all_objs[obj_id].fn_set[face_id](2)];
    };

    void get_face_vt(int obj_id, 
                     int face_id,
                     Eigen::Vector2f &A, 
                     Eigen::Vector2f &B, 
                     Eigen::Vector2f &C)
    {
        A << vt_mat[all_objs[obj_id].ft_set[face_id](0)];
        B << vt_mat[all_objs[obj_id].ft_set[face_id](1)];
        C << vt_mat[all_objs[obj_id].ft_set[face_id](2)];
    };
    
    void get_face_n(int obj_id, 
                    int face_id,
                    Eigen::Vector3f &normal)
    {
        normal << all_objs[obj_id].f_normal[face_id];
    };

    void get_obj_mat(int obj_id, material &mat)
    {
        mat = all_materials[all_objs[obj_id].mat_name];
    };

    void get_n_tex(int obj_id, int face_id,
                      Eigen::Vector3f &hit_p,
                      Eigen::Vector3f &normal,
                      material &hit_mat,
                      Eigen::Vector3f &tex_color)
    {
        Eigen::Vector3f A, B, C;
        get_face_v(obj_id, face_id, A, B, C);
        Eigen::Vector3f An, Bn, Cn;
        get_face_vn(obj_id, face_id, An, Bn, Cn);
        Eigen::Vector2f At, Bt, Ct;
        get_face_vt(obj_id, face_id, At, Bt, Ct);

        Eigen::Vector3f AB = B - A;
        Eigen::Vector3f AC = C - A;
        Eigen::Vector3f AP = hit_p - A;

        float S_abc = (AB.cross(AC)).norm();
        float S_apc = (AP.cross(AC)).norm();
        float S_apb = (AB.cross(AP)).norm(); 

        float w1 = S_apc / S_abc;
        float w2 = S_apb / S_abc;
        normal = (1 - w1 - w2) * An + w1 * Bn + w2 * Cn;

        Eigen::Vector2f uv = (1 - w1 - w2) * At + w1 * Bt + w2 * Ct;
        tex_color = get_tex(hit_mat.texture, uv);
    };

    float re_uv(float x){ x = x - floor(x); return x >= 1 ? 0.98 : x; }

    Eigen::Vector3f get_tex(cv::Mat texture, Eigen::Vector2f uv)
    {
        if(texture.rows > 0)
        {
            int u = texture.rows * re_uv(uv[1]);
            int v = texture.cols * re_uv(uv[0]);

            cv::Vec3f pixel = texture.at<cv::Vec3f>(u, v);
            return Eigen::Vector3f(pixel[2], pixel[1], pixel[0]);
        }
        else
            return Eigen::Vector3f(0, 0, 0);
    };

};

#endif