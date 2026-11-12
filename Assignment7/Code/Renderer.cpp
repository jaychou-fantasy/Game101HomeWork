//
// Created by goksu on 2/25/20.
//

#include <fstream>
#include "Scene.hpp"
#include "global.hpp"
#include "Renderer.hpp"
//for multi-threads
#include <atomic>
#include <chrono>
#include <thread>
#include <vector>


inline float deg2rad(const float& deg) { return deg * M_PI / 180.0; }

const float EPSILON = 0.00001;

// The main render function. This where we iterate over all pixels in the image,
// generate primary rays and cast these rays into the scene. The content of the
// framebuffer is saved to a file.
void Renderer::Render(const Scene& scene, int spp, unsigned threadCount)
{
    std::vector<Vector3f> framebuffer(scene.width * scene.height);

    float scale = tan(deg2rad(scene.fov * 0.5));
    float imageAspectRatio = scene.width / (float)scene.height;
    Vector3f eye_pos(278, 273, -800);
    int m = 0;

    std::cout << "SPP: " << spp << "\n";
    // Naive Approach:
    // for (uint32_t j = 0; j < scene.height; ++j) {
    //     for (uint32_t i = 0; i < scene.width; ++i) {
    //         // generate primary ray direction
    //         float x = (2 * (i + 0.5) / (float)scene.width - 1) *
    //                   imageAspectRatio * scale;
    //         float y = (1 - 2 * (j + 0.5) / (float)scene.height) * scale;

    //         Vector3f dir = normalize(Vector3f(-x, y, 1));
    //         for (int k = 0; k < spp; k++){
    //             framebuffer[m] += scene.castRay(Ray(eye_pos, dir), 0) / spp;
    //         }
    //         m++;
    //     }
    //     UpdateProgress(j / (float)scene.height);
    // }
    // UpdateProgress(1.f);

    //Multiple Threads
    //nextRow : next row of pixel to be rendered(to be distributed)
    //finishedRows : the rows have been rendered
    std::atomic<int> nextRow{0};
    std::atomic<int> finishedRows{0};
    //what a thread worker should do : keep fecth_add to get new row to be rendered , and then render it(castRay)
    auto renderRows = [&](){
        while(true)//atomic operation guarantee that different threads can't render same rows of pixels
        {
            //fetch_add(1) : fetch the value before add 1,then cnt += 1
            //and atomic can let fetch,add become one operateion --- refrain from two threads do fetch and add apart
            //so two threads won't get the same row
            const int j = nextRow.fetch_add(1);
            if( j >= scene.height)
                break;//render process done --- available thread break in the loop,otherwise while true->get rows continuously
            for(int i = 0 ; i < scene.width ; ++i)
            {
                float x = (2 * (i + 0.5) / (float)scene.width - 1) *
                            imageAspectRatio * scale;
                float y = (1 - 2 * (j + 0.5) / (float)scene.height) * scale;
                const Vector3f dir = normalize(Vector3f(-x,y,1));
                Vector3f color;
                for(int k = 0; k < spp ; ++k)
                {
                    color += scene.castRay(Ray(eye_pos,dir), 0) / spp;
                }
                //one row is only under the responsibility of the coresponding thread(one on one)
                framebuffer[j*scene.width + i] = color;
            }

            finishedRows.fetch_add(1);
        }
    };

    //create theards and start
    std::vector<std::thread> workers;
    workers.reserve(threadCount);//reserve space for theards instance (haven't start it yet)
    for(unsigned t = 0; t < threadCount ; ++t)
    {
        workers.emplace_back(renderRows); // == emplace_back(std::thread(renderRows)) ,like construct function
    }
    //only main thread print "UpdateProgress" --  every 200ms
    while(true)
    {
        const int num_DoneRows = finishedRows.load();
        if(num_DoneRows >= scene.height)
            break;//already done

        UpdateProgress(num_DoneRows / static_cast<float>(scene.height));
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
    //wait till all work done,can save the framebuffer
    for(auto& worker : workers)
    {
        worker.join();//main thread wait till worker thread done his work,and only then you can execute the next line of program
        //not function of "join"
    }
    UpdateProgress(1.0f);


    // save framebuffer to file
    FILE* fp = fopen("binary.ppm", "wb");
    (void)fprintf(fp, "P6\n%d %d\n255\n", scene.width, scene.height);
    for (auto i = 0; i < scene.height * scene.width; ++i) {
        static unsigned char color[3];
        color[0] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].x), 0.6f));
        color[1] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].y), 0.6f));
        color[2] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].z), 0.6f));
        fwrite(color, 1, 3, fp);
    }
    fclose(fp);    
}
