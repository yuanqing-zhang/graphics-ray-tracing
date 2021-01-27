#include <iostream>
#include <fstream>

#include "io/scene.h"
#include "raytracer/cfg.h"
#include "raytracer/utils.h"
#include "raytracer/raytracer.h"

#include "Eigen/Dense"

using namespace Eigen;
using namespace std;


Vector3f* ray_tracing(scene &scene, renderCfg cfg)
{
    Vector3f* image = new Vector3f[cfg.width * cfg.height]; 

    for (int h = 0; h < cfg.height; h++)
    {
        fprintf(stderr, "\r[LOG] Rendering (%d spp) %5.2f%%", 
                cfg.samples * 4, 100. * h / (cfg.height - 1));
        
        for(int w = 0; w < cfg.width; w++)
        {
            for(int sh = 0; sh < cfg.subpixel; sh++)     // subpixel rows
                for(int sw = 0; sw < cfg.subpixel; sw++) // subpixel cols
                {
                    Vector3f trace_color(0, 0, 0);
                    for(int s = 0; s < cfg.samples; s++)
                    {
                        // rand sample in [-1, 1]
                        unsigned short seed[3]={0, 0, (unsigned short)(h * h * h)};
                        double dx= tent_filter(2 * erand48(seed)); 
                        double dy = tent_filter(2 * erand48(seed));

                        Vector3f vec_dx = cfg.cx * (((sw + 0.5 + dx) / cfg.subpixel + w) / cfg.width - 0.5);
                        Vector3f vec_dy = cfg.cy * (((sh + 0.5 + dy) / cfg.subpixel + h) / cfg.height - 0.5);
                        Vector3f d = cfg.direction + vec_dx + vec_dy;

                        ray ray(cfg.position, d);
                        trace_color += ray_tracing(scene, 
                                                   ray, 
                                                   0, 
                                                   seed) * (1.0 / cfg.samples);
                    }
                    // average colors in subpixels
                    image[(cfg.height - h - 1) * cfg.width + w] += trace_color / cfg.subpixel / cfg.subpixel;
                }
        }
    }
    cout << endl;
    return image;
}


int main(int argc, char* argv[])
{
    if(argc != 2)
    {
        std::cout << "Usage: ray-tracing <scene_name>" << std::endl;
        std:: cout << "scene_name can be (1)cornellbox, (2)car, (3)diningroom." << std::endl;
        return 1;
    }

    std::string scene_name = argv[1];
    
    scene scene;
    scene.load_scene(scene_name);
    cout << "[LOG] Finish load sence." << endl;

    Vector3f* image;
    if(scene_name == "cornellbox")
    {
        image = ray_tracing(scene, box_cfg);
        save_image(image, scene_name, box_cfg);
    }

    return 0;

}