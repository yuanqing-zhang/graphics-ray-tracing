#include <iostream>
#include <fstream>
#include <ctime>
#include <omp.h> //openMP

#include "io/scene.h"
#include "utils/cfg.h"
#include "utils/utils.h"

#include "raytracer/raytracer.h"

#include "Eigen/Dense"

using namespace Eigen;
using namespace std;


Vector3f* ray_tracing(Scene &scene, renderCfg cfg)
{
    Vector3f* image = new Vector3f[cfg.width * cfg.height]; 

    for (int h = 0; h < cfg.height; h++)
    {
        fprintf(stderr, "\r[LOG] Rendering (%d spp) %5.2f%%", 
                cfg.samples * 4, 100. * h / (cfg.height - 1));
        #pragma omp parallel for
        for(int w = 0; w < cfg.width; w++)
        {
            for(int sh = 0; sh < cfg.subpixel; sh++)     // subpixel rows
                for(int sw = 0; sw < cfg.subpixel; sw++) // subpixel cols
                {
                    Vector3f trace_color(0, 0, 0);
                    for(int s = 0; s < cfg.samples; s++)
                    {
                        // rand sample in [-1, 1]
                        double dx= tent_filter(2 * drand48()); 
                        double dy = tent_filter(2 * drand48());

                        Vector3f vec_dx = cfg.cx * (((sw + 0.5 + dx) / cfg.subpixel + w) / cfg.width - 0.5);
                        Vector3f vec_dy = cfg.cy * (((sh + 0.5 + dy) / cfg.subpixel + h) / cfg.height - 0.5);
                        Vector3f d = cfg.direction + vec_dx + vec_dy;

                        Ray ray(cfg.position, d);
                        trace_color += ray_tracing(scene, 
                                                   ray, 
                                                   cfg.depth) * (1.0 / cfg.samples);
                    }
                    // average colors in subpixels
                    for(int t = 0; t < 3; t++)
                        trace_color(t) = clamp(trace_color(t));
                    image[(cfg.height - h - 1) * cfg.width + w] += trace_color / cfg.subpixel / cfg.subpixel;
                }
        }
    }
    cout << endl;
    return image;
}


int main(int argc, char* argv[])
{

    srand48((long int)time(0));
    if(argc < 2)
    {
        std::cout << "Usage: ray-tracing <scene_name>" << std::endl;
        std:: cout << "scene_name can be (1)cornellbox, (2)car, (3)diningroom." << std::endl;
        return 1;
    }


    std::string scene_name = argv[1];

    Scene scene;
    scene.load_scene(scene_name);
    scene.build_BVH();

    Vector3f* image;
    if(scene_name == "cornellbox")
    {
        clock_t start = clock();
        image = ray_tracing(scene, box_cfg);
        cout << "[LOG] Rendering time: " << (double)(clock() - start) / CLOCKS_PER_SEC << endl;
        save_image(image, scene_name, box_cfg);
    }
    if(scene_name == "cornellbox2")
    {
        clock_t start = clock();
        image = ray_tracing(scene, box_cfg2);
        cout << "[LOG] Rendering time: " << (double)(clock() - start) / CLOCKS_PER_SEC << endl;
        save_image(image, scene_name, box_cfg2);
    }
    if(scene_name == "car")
    {
        clock_t start = clock();
        image = ray_tracing(scene, car_cfg1);
        cout << "[LOG] Rendering time: " << (double)(clock() - start) / CLOCKS_PER_SEC << endl;
        save_image(image, scene_name, car_cfg1);
    }
    if(scene_name == "diningroom")
    {
        clock_t start = clock();
        image = ray_tracing(scene, room_cfg);
        cout << "[LOG] Rendering time: " << (double)(clock() - start) / CLOCKS_PER_SEC << endl;
        save_image(image, scene_name, room_cfg);
    }


    return 0;

}