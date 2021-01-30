#include "utils.h"

#include <Eigen/Dense>
#include <iostream>
#include <fstream>

using namespace Eigen;
using namespace std;


inline int to_int(float x){ return int(pow(clamp(x), 1 / 2.2) * 255 + .5); }


void save_image(Vector3f* image, const string &scene_name, renderCfg cfg)
{
    fstream f;
    string save_path = "../render/" + scene_name + ".ppm";

    f.open(save_path, ios::out | ios::trunc);
    f << "P3\n" <<  cfg.width << " " << cfg.height << "\n" << 255 << endl;

    for (int i = 0; i < cfg.width * cfg.height; i++)
        f << to_int(image[i](0)) << " "
          << to_int(image[i](1)) << " "
          << to_int(image[i](2)) << endl;

    f.close();
    cout << "[LOG] Save render result in " << save_path << endl;
};


double tent_filter(double x)
{
    return x < 1 ? sqrt(x) - 1 : 1 - sqrt(2 - x);
};