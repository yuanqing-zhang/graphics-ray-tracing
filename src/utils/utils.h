#ifndef __UTILS_H__
#define __UTILS_H__

#include "cfg.h"


void save_image(Eigen::Vector3f* image, const std::string &scene_name, renderCfg cfg);

double tent_filter(double x);

#endif