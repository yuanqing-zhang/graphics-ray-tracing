#include "utils.h"
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>

#include <Eigen/Dense>
#include <iostream>
#include <fstream>

using namespace Eigen;
using namespace std;
using namespace cv;


inline int to_int(float x){ return int(pow(clamp(x), 1 / 2.2) * 255 + .5); }


float tone_mapping(float color)
{
    const float A = 2.51, B = 0.03, C = 2.43, D = 0.59, E = 0.14;
    return (color * (A * color + B)) / (color * (C * color + D) + E);
}


void save_image(Vector3f* image, const string &scene_name, renderCfg cfg)
{
    fstream f;
    string save_path = "../render/" + scene_name + ".ppm";
    
    string folder = "../render";  
    if (access(folder.c_str(), 0) != 0)
    {
        mkdir(folder.c_str(), S_IRWXU);
    }
    // system(command.c_str());

    f.open(save_path, ios::out | ios::trunc);
    f << "P3\n" <<  cfg.width << " " << cfg.height << "\n" << 255 << endl;

    for (int i = 0; i < cfg.width * cfg.height; i++)
        f << to_int(tone_mapping(image[i](0))) << " "
          << to_int(tone_mapping(image[i](1))) << " "
          << to_int(tone_mapping(image[i](2))) << endl;

    f.close();
    cout << "[LOG] Save render result in " << save_path << endl;
};


double tent_filter(double x)
{
    return x < 1 ? sqrt(x) - 1 : 1 - sqrt(2 - x);
};
