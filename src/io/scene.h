#pragma once

#include <iostream>
#include <map>
#include <vector>
#include <Eigen/Dense>

using namespace Eigen;
using namespace std;


class material
{
public:
    
    Vector3f Kd; // diffuse
    Vector3f Ka; // ambient
    Vector3f Ks; // specular
    Vector3f Le; // emission

    float Ns; // specular exponent, 0 ~ 1000
    float Ni; // ptical density, 0.001 ~ 10

    string map_Kd; // texture

	material() 
    {
        Kd = Vector3f(0, 0, 0); Ka = Vector3f(0, 0, 0);
        Ks = Vector3f(0, 0, 0); Le = Vector3f(0, 0, 0);
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
	Vector3f center;

	vector<Vector3f> v_mat;  // vertices
	vector<Vector3f> vn_mat;  // vertex normals
    vector<Vector2f> vt_mat;  // texture coordinates
	vector<Vector3i> fv_set;  // index of vertices
	vector<Vector3i> fn_set; // index of normals
    vector<Vector3i> ft_set; // index of texture coordinates

    string mat_name;


};


class scene
{
public:
    vector<obj> all_objs;
    map<string, material> all_materials;

    scene(){};
    void load_scene(const string &scene_name);
    void load_mtl(const string &scene_name);

};