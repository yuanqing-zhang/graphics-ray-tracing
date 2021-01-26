#include <iostream>
#include "io/scene.h"


int main(int argc, char* argv[])
{
    if(argc != 2)
    {
        std::cout << "Usage: ray-tracing <scene_name>" << std::endl;
        std:: cout << "scene_name can be (1)cornellbox, (2)car, (3)diningroom." << std::endl;
        return 1;
    }

    std::string scene_name = argv[1];
    
    scene sscene;
    sscene.load_scene(scene_name);
    cout << "[LOG]Finish load sence." << endl;

    return 0;

}