#ifndef __CFG_H__
#define __CFG_H__

#include <Eigen/Dense>


class ray
{
public:
    Eigen::Vector3f o;
    Eigen::Vector3f d;
    ray(Eigen::Vector3f _o, Eigen::Vector3f _d):o(_o), d(_d){};
};


class renderCfg
{
public:
    Eigen::Vector3f position;
    Eigen::Vector3f direction;

    int width, height;
    int samples;
    int depth;
    int subpixel;

    Eigen::Vector3f cx;
    Eigen::Vector3f cy;


    renderCfg();
    renderCfg(Eigen::Vector3f pos, Eigen::Vector3f look_at, Eigen::Vector3f up, 
                int f, int w, int h, int s, int d, int subp):
                position(pos), width(w), height(h), samples(s), depth(d), subpixel(subp)
    {
        // direction = look_at - pos;
        direction = look_at;
        direction.normalize();

        float z = 1.0;
        float real_w = z * tan(f);
        cx = direction.cross(up) * real_w;
        cy = cx.cross(direction) * real_w;
    };

};

renderCfg box_cfg = renderCfg(
    Eigen::Vector3f(0, 0, 2.5),
    Eigen::Vector3f(0, 0, 0),
    Eigen::Vector3f(0, 1, 0),
    60,
    128,
    128,
    2,
    6,
    2
);

renderCfg car_cfg1 = renderCfg(
    Eigen::Vector3f( 8.220, -0.610, -9.800),
    Eigen::Vector3f( 7.514, -0.702, -9.097),
    Eigen::Vector3f(-0.065,  0.996,  0.065),
    45,
    128,
    128,
    2,
    6,
    2
);

renderCfg car_cfg2 = renderCfg(
    Eigen::Vector3f( 5.720,  0.120,  9.550),
    Eigen::Vector3f( 5.085, -0.131,  8.819),
    Eigen::Vector3f(-0.165,  0.968, -0.189),
    45,
    128,
    128,
    2,
    6,
    2
);

renderCfg room_cfg = renderCfg(
    Eigen::Vector3f(0.000, 12.720, 31.850),
    Eigen::Vector3f(0.000, 12.546, 30.865),
    Eigen::Vector3f(0.000,  0.985, -0.174),
    60,
    128,
    128,
    2,
    6,
    2
);


#endif