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
        cy = cy * real_w * h / w;
    };

};

renderCfg box_cfg = renderCfg(
    Eigen::Vector3f(0, 0, 1.75),
    Eigen::Vector3f(0, 0, -1),
    Eigen::Vector3f(0, 1, 0),
    60,
    300,
    300,
    5,
    5,
    2
);

// renderCfg car_cfg = renderCfg(
//     Eigen::Vector3f(5.72, 0.12, 9.55),
//     Eigen::Vector3f(-5.085, -0.131, -8.819),
//     Eigen::Vector3f( -0.165, 0.968, -0.189),
//     45,
//     200,
//     100,
//     5,
//     5,
//     2
// );

renderCfg car_cfg = renderCfg(
    Eigen::Vector3f(8.22, -0.61, -13.80),
    Eigen::Vector3f(-7.14, -0.702, 9.097),
    Eigen::Vector3f(-0.065, 0.996, 0.065),
    45,
    600,
    400,
    200,
    3,
    2
);

renderCfg room_cfg = renderCfg(
    Eigen::Vector3f( 0.000, 15.720, 26.850),
    Eigen::Vector3f(0.000, -12.546, -41.865),
    Eigen::Vector3f(0.000,  0.985, -0.174),
    60,
    600,
    400,
    200,
    // 150,
    // 100,
    // 1,
    4,
    2
);


#endif