#include "Renderer.hpp"
#include "Scene.hpp"
#include "Triangle.hpp"
#include "Sphere.hpp"
#include "Vector.hpp"
#include "global.hpp"
#include <algorithm>
#include <chrono>
#include <exception>
#include <iostream>
#include <string>
#include <thread>

// In the main function of the program, we create the scene (create objects and
// lights) as well as set the options for the render (image width and height,
// maximum recursion depth, field-of-view, etc.). We then call the render
// function().
int main(int argc, char** argv)
{
    int spp = 256;
    const char* sppArg = nullptr;
    if (argc == 2) {
        sppArg = argv[1];
    } else if (argc == 3 && std::string(argv[1]) == "--spp") {
        sppArg = argv[2];
    } else if (argc != 1) {
        std::cerr << "Usage: " << argv[0] << " [spp] or "
                  << argv[0] << " --spp <spp>\n";
        return 1;
    }

    if (sppArg != nullptr) {
        const std::string value(sppArg);
        std::size_t parsed = 0;
        try {
            spp = std::stoi(value, &parsed);
        } catch (const std::exception&) {
            std::cerr << "SPP must be a positive integer.\n";
            return 1;
        }
        if (parsed != value.size() || spp <= 0) {
            std::cerr << "SPP must be a positive integer.\n";
            return 1;
        }
    }

    // Change the definition here to change resolution
    Scene scene(784, 784);

    const unsigned threadCount = std::max(
        1u,
        std::min(std::thread::hardware_concurrency(),
                 static_cast<unsigned>(scene.height)));

    Material* red = new Material(DIFFUSE, Vector3f(0.0f));
    red->Kd = Vector3f(0.63f, 0.065f, 0.05f);
    Material* green = new Material(DIFFUSE, Vector3f(0.0f));
    green->Kd = Vector3f(0.14f, 0.45f, 0.091f);
    Material* white = new Material(DIFFUSE, Vector3f(0.0f));
    white->Kd = Vector3f(0.725f, 0.71f, 0.68f);
    Material* light = new Material(DIFFUSE, (8.0f * Vector3f(0.747f+0.058f, 0.747f+0.258f, 0.747f) + 15.6f * Vector3f(0.740f+0.287f,0.740f+0.160f,0.740f) + 18.4f *Vector3f(0.737f+0.642f,0.737f+0.159f,0.737f)));
    light->Kd = Vector3f(0.65f);

    MeshTriangle floor("Assignment7/Code/models/cornellbox/floor.obj", white);
    MeshTriangle shortbox("Assignment7/Code/models/cornellbox/shortbox.obj", white);
    MeshTriangle tallbox("Assignment7/Code/models/cornellbox/tallbox.obj", white);
    MeshTriangle left("Assignment7/Code/models/cornellbox/left.obj", red);
    MeshTriangle right("Assignment7/Code/models/cornellbox/right.obj", green);
    MeshTriangle light_("Assignment7/Code/models/cornellbox/light.obj", light);

    scene.Add(&floor);
    scene.Add(&shortbox);
    scene.Add(&tallbox);
    scene.Add(&left);
    scene.Add(&right);
    scene.Add(&light_);

    scene.buildBVH();

    Renderer r;

    std::cout << "Render worker threads: " << threadCount << '\n';
    std::cout << "Including main thread: " << threadCount + 1 << '\n';

    auto start = std::chrono::system_clock::now();
    r.Render(scene, spp, threadCount);
    auto stop = std::chrono::system_clock::now();

    std::cout << "Render complete: \n";
    std::cout << "Time taken: " << std::chrono::duration_cast<std::chrono::hours>(stop - start).count() << " hours\n";
    std::cout << "          : " << std::chrono::duration_cast<std::chrono::minutes>(stop - start).count() << " minutes\n";
    std::cout << "          : " << std::chrono::duration_cast<std::chrono::seconds>(stop - start).count() << " seconds\n";

    return 0;
}//magick binary.ppm output.png
