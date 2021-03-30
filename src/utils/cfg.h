#ifndef __CFG_H__
#define __CFG_H__

#include <iostream>
#include <Eigen/Dense>

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
        direction = look_at;
        direction.normalize();
        up.normalize();

        float z = 1.0;
        float real_w = z * tan(f * 1.0 / 180 * M_PI);
        cx = direction.cross(up);
        cy = cx.cross(direction);

        cx.normalize(); cy.normalize();
        cx = cx * real_w;
        cy = cy * real_w;
    };

};

renderCfg box_cfg = renderCfg(
    Eigen::Vector3f(0, 0, 1.75),
    Eigen::Vector3f(0, 0, -1),
    Eigen::Vector3f(0, 1, 0),
    60,
    200,
    200,
    5,
    5,
    2
);

renderCfg car_cfg = renderCfg(
    Eigen::Vector3f(12.220, 0.0, 13.800),
    Eigen::Vector3f(-1.0, 0.0, -1.0),
    Eigen::Vector3f(-0.065,  0.996,  0.065),
    45,
    500,
    500,
    200,
    5,
    2
);

renderCfg room_cfg = renderCfg(
    Eigen::Vector3f(0.000, 20.0, 30.0),
    Eigen::Vector3f(0.000, -1.00, -2.00),
    Eigen::Vector3f(0.000,  0.985, -0.174),
    60,
    600,
    600,
    200,
    5,
    2
);


#endif