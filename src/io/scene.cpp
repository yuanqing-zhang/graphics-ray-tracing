#include "scene.h"
#include <fstream>
#include <sstream>
#include <string>

using namespace Eigen;
using namespace std;


bool is_line_valid(const string &line) {
	return (line.empty() || line[0] == 13 || line[0] == '#');
}


void Scene::load_mtl(const string &mtl_name)
{
    fstream f;
    string line, type, mat_name;
    float t1, t2, t3;

    f.open(mtl_name, ios::in);
	if(!f.is_open())
	{
		cout << "[ERROR] fail to open mtl file in " << mtl_name << endl;
		return;
	}

    // read line by line
    while (getline(f, line))
    {
        if(is_line_valid(line)) continue;
        istringstream instream(line);
		instream >> type;
        if(type == "newmtl")
        {
            instream >> mat_name;
            material mat;
            all_materials[mat_name] = mat;
        }
        else if(type == "Ns"){instream >> all_materials[mat_name].Ns;}
        else if(type == "Ni"){instream >> all_materials[mat_name].Ni;}
        else if(type == "map_Kd"){instream >> all_materials[mat_name].map_Kd;}
        else if(type == "Kd")
        {
            instream >> t1 >> t2 >> t3;
            all_materials[mat_name].Kd = Vector3f(t1, t2, t3);
        }
        else if(type == "Ka")
        {
            instream >> t1 >> t2 >> t3;
            all_materials[mat_name].Ka = Vector3f(t1, t2, t3);
        }
        else if(type == "Ks")
        {
            instream >> t1 >> t2 >> t3;
            all_materials[mat_name].Ks = Vector3f(t1, t2, t3);
        }
        else if(type == "Le")
        {
            instream >> t1 >> t2 >> t3;
            all_materials[mat_name].Le = Vector3f(t1, t2, t3);
        }
        else
        {
            cout << "[ERROR] unknown materials type " << type << 
                    ", please check whether to update class material." << endl;
            assert(0);
        }
    }  


}


void Scene::load_scene(const string &scene_name)
{
    fstream f;
    string line, type, mtl_name;
    bool is_new_obj = true;
    int obj_index = -1;
    float v1, v2, v3;
    int i1, i2, i3;
    Vector3i fv, fn, ft;
    string mat_name;

    // open obj file and check 
    string scene_dir = "../example-scenes/" + scene_name + "/";
    string obj_name = scene_dir + scene_name + ".obj";

    f.open(obj_name, ios::in);
	if(!f.is_open())
	{
		cout << "[ERROR]Fail to open obj file in " << obj_name << endl;
		return;
	}

    // read line by line
    while (getline(f, line))
    {
        if(is_line_valid(line)) continue;
		istringstream instream(line);
		instream >> type;
        if(type == "mtllib") 
        {
            instream >> mtl_name;
            load_mtl(scene_dir + mtl_name);
        }
        else if(type == "v" || type == "V") 
        {
            instream >> v1 >> v2 >> v3;
            v_mat.push_back(Vector3f(v1, v2, v3));
            // create if read light
        }
        else if(type == "vn" || type == "VN") 
        {
            instream >> v1 >> v2 >> v3;
            vn_mat.push_back(Vector3f(v1, v2, v3));

        }
        else if(type == "vt" || type == "VT") 
        {
            instream >> v1 >> v2;
            vt_mat.push_back(Vector2f(v1, v2));
        }
        else if(type == "g" || type == "G") 
        {
            continue;
        }
        else if(type == "usemtl" || type == "USEMTL") 
        {
            // create new object while reading first f
            if(is_new_obj == true)
            {
                all_objs.push_back(obj());
                is_new_obj = false;
                obj_index += 1;
            }
            instream >> mat_name;
            all_objs[obj_index].mat_name = mat_name;
            if(mat_name.find("light")!= string::npos)
            {
                light l;
                l.Le = all_materials[mat_name].Le;
                l.obj_index = obj_index;
                all_lights.push_back(l);
            }
        }
        else if(type == "f" || type == "F")
        {
            is_new_obj = true;
            
            fv = Vector3i(0, 0, 0); fn = Vector3i(0, 0, 0); ft = Vector3i(0, 0, 0);
            for(int i = 0; i < 3; i++)
			{
				string f_group;
				instream >> f_group;

				// get vertex index
				string v_str = f_group.substr(0, f_group.find('/'));
				unsigned long v_id;
				sscanf(v_str.c_str(), "%lu", &v_id);
                fv(i) = v_id - 1; // obj file index start from 1

				// get vertex normal index if exist
				string vn_str = f_group.substr(f_group.find('/') + 1, f_group.rfind('/'));
				if (vn_str.size() == 0) continue;
				unsigned long vn_id;
				sscanf(vn_str.c_str(), "%lu", &vn_id);
				fn(i) = vn_id - 1;

                // get texture coordinate if exist
                string vt_str = f_group.substr(f_group.rfind('/') + 1, f_group.length());
                if(vt_str.size() == 0) continue;
                unsigned long vt_id;
                sscanf(vt_str.c_str(), "%lu", &vt_id);
                ft(i) = vt_id - 1;
			}
            all_objs[obj_index].fv_set.push_back(fv);
            all_objs[obj_index].fn_set.push_back(fn);
            all_objs[obj_index].ft_set.push_back(ft);
            
            Vector3f normal = (vn_mat[fn(0)] + vn_mat[fn(1)] + vn_mat[fn(2)]) / 3;
            normal.normalize();
            all_objs[obj_index].f_normal.push_back(normal);

        }        
        else
        {
            cout << "[ERROR]Unknown obj type " << type << 
                    ", please check whether to update class obj." << endl;
            assert(0);
        }

        type.clear();
    }
    f.close();

    // calulate AABB for all objects
    for(int i = 0; i < all_objs.size(); i++)
    {
        for(int axis = 0; axis < 3; axis++)
        {
            float min = 10000, max = -10000;
            for(int f = 0; f < all_objs[i].fv_set.size(); f++)
            {
                Vector3i curr_f = all_objs[i].fv_set[f];
                for(int v = 0; v < 3; v++)
                {
                    if(v_mat[curr_f[v]](axis) < min)
                        min = v_mat[curr_f[v]](axis);
                    if(v_mat[curr_f[v]](axis) > max)
                        max = v_mat[curr_f[v]](axis);
                }
            }
            all_objs[i].bbox.bbox_min(axis) = min;
            all_objs[i].bbox.bbox_max(axis) = max;
        }
    }
    
    cout << "[LOG] Total vertice: " << v_mat.size() 
            << ", total objects:  " << all_objs.size() << endl;
}

