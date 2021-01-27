#ifndef __UTILS_H__
#define __UTILS_H__

#include <iostream>
#include <fstream>
#include "cfg.h"


inline float clamp(float x){ return x < 0 ? 0 : x > 1 ? 1 : x; }
inline int to_int(float x){ return int(pow(clamp(x), 1 / 2.2) * 255 + .5); }


void save_image(Eigen::Vector3f* image, const std::string &scene_name, renderCfg cfg)
{
    std::fstream f;
    std::string save_path = "../render/" + scene_name + ".ppm";

    f.open(save_path, std::ios::out | std::ios::trunc);
    f << "P3\n" <<  cfg.width << " " << cfg.height << "\n" << 255 << std::endl;

    for (int i = 0; i < cfg.width * cfg.height; i++)
        f << to_int(image[i](0)) << " "
          << to_int(image[i](1)) << " "
          << to_int(image[i](2)) << std::endl;

    f.close();
    std::cout << "[LOG] Save render result in " << save_path << std::endl;
};


double tent_filter(double x)
{
    return x < 1 ? sqrt(x) - 1 : 1 - sqrt(2 - x);
};

#endif