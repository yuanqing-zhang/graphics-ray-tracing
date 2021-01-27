#ifndef __SCENE_H__
#define __SCENE_H__

#include <iostream>
#include <map>
#include <vector>
#include <Eigen/Dense>

#include "../utils/AABB.h"


class material
{
public:
    
    Eigen::Vector3f Kd; // diffuse
    Eigen::Vector3f Ka; // ambient
    Eigen::Vector3f Ks; // specular
    Eigen::Vector3f Le; // emittance

    float Ns; // specular exponent, 0 ~ 1000
    float Ni; // ptical density, 0.001 ~ 10

    std::string map_Kd; // texture

	material() 
    {
        Kd = Eigen::Vector3f(0, 0, 0); Ka = Eigen::Vector3f(0, 0, 0);
        Ks = Eigen::Vector3f(0, 0, 0); Le = Eigen::Vector3f(0, 0, 0);
        Ns = 0; Ni = 1;
        map_Kd = "None";
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

    std::vector<Eigen::Vector3i> fv_set;  // index of vertices
    std::vector<Eigen::Vector3i> fn_set;  // index of normals
    std::vector<Eigen::Vector3i> ft_set;  // index of texture coordinates

    std::vector<Eigen::Vector3f> f_normal; // face nromal (average of vertex normal)

    std::string mat_name;
    AABB bbox;

    obj(){};
};


class scene
{
public:

    std::vector<Eigen::Vector3f> v_mat;   // all vertices
    std::vector<Eigen::Vector3f> vn_mat;  // all vertex normals
    std::vector<Eigen::Vector2f> vt_mat;  // all texture coordinates

    std::vector<obj> all_objs;
    std::map<std::string, material> all_materials;

    scene(){};
    void load_scene(const std::string &scene_name);
    void load_mtl(const std::string &scene_name);
    
    
    // utils
    void get_face_v(int obj_id, 
                    int face_id,
                    Eigen::Vector3f &A, 
                    Eigen::Vector3f &B, 
                    Eigen::Vector3f &C)
    {
        A << v_mat[all_objs[obj_id].fv_set[face_id](0)];
        B << v_mat[all_objs[obj_id].fv_set[face_id](1)];
        C << v_mat[all_objs[obj_id].fv_set[face_id](2)];
    };
    
    void get_face_n(int obj_id, 
                    int face_id,
                    Eigen::Vector3f &normal)
    {
        normal << all_objs[obj_id].f_normal[face_id];
    };


};

#endif