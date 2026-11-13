#include "Renderer.hpp"
#include "Scene.hpp"
#include "Triangle.hpp"
#include "Vector.hpp"
#include "global.hpp"
#include "BVH.hpp"
#include <chrono>
#include <iomanip>
#include <iostream>
#include <string_view>

// In the main function of the program, we create the scene (create objects and
// lights) as well as set the options for the render (image width and height,
// maximum recursion depth, field-of-view, etc.). We then call the render
// function().
int main(int argc, char** argv)
{
    const std::string_view choice = argc == 2 ? argv[1] : "compare";
    if (argc > 2 || (choice != "compare" && choice != "naive" && choice != "sah")) {
        std::cerr << "Usage: RayTracing [compare|naive|sah]\n";
        return 1;
    }

    std::cout << std::fixed << std::setprecision(3);
    for (const auto splitMethod : {BVHAccel::SplitMethod::NAIVE,
                                   BVHAccel::SplitMethod::SAH}) {
        if ((choice == "naive" && splitMethod != BVHAccel::SplitMethod::NAIVE) ||
            (choice == "sah" && splitMethod != BVHAccel::SplitMethod::SAH)) {
            continue;
        }

        const char* label = splitMethod == BVHAccel::SplitMethod::NAIVE
                                ? "NAIVE BVH" : "SAH BVH";
        std::cout << "\n=== " << label << " ===\n";

        Scene scene(512, 512);
        MeshTriangle bunny("Assignment6/Code/models/bunny/bunny.obj", splitMethod);
        scene.Add(&bunny);
        scene.Add(std::make_unique<Light>(Vector3f(-20, 70, 20), 1));
        scene.Add(std::make_unique<Light>(Vector3f(20, 70, 20), 1));
        scene.buildBVH();

        Renderer renderer;
        auto renderStart = std::chrono::steady_clock::now();
        renderer.Render(scene);
        double renderSeconds = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - renderStart).count();

        std::cout << "\n" << label << " triangle BVH build: "
                  << bunny.bvhBuildMilliseconds << " ms\n";
        std::cout << label << " render + PNG save: "
                  << renderSeconds << " seconds\n";
    }

    return 0;
}
