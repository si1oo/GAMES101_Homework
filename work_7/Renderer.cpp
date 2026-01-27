//
// Created by goksu on 2/25/20.
//

#include <fstream>
#include<thread>
#include<atomic>
#include "Scene.hpp"
#include "Renderer.hpp"


inline float deg2rad(const float& deg) { return deg * M_PI / 180.0; }

const float EPSILON = 0.00001;

// The main render function. This where we iterate over all pixels in the image,
// generate primary rays and cast these rays into the scene. The content of the
// framebuffer is saved to a file.
void Renderer::Render(const Scene& scene)
{
    std::vector<Vector3f> framebuffer(scene.width * scene.height);

    float scale = tan(deg2rad(scene.fov * 0.5));
    float imageAspectRatio = scene.width / (float)scene.height;
    Vector3f eye_pos(278, 273, -800);
    int m = 0;

    // change the spp value to change sample ammount
    int spp = 128;
    std::cout << "SPP: " << spp << "\n";

    const int total_pixels = scene.width * scene.height;
    std::atomic<int> completed_pixels(0);
    const int num_threads = std::thread::hardware_concurrency();
    std::vector<std::thread> threads;

    //更新进度
    auto update_progress = [&]() {
        while (completed_pixels < total_pixels) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            float progress = static_cast<float>(completed_pixels) / total_pixels;
            UpdateProgress(progress);
        }
    };

    std::thread progress_thread(update_progress);

    //工作线程函数，pixel_idx = j × width + i
    auto render_worker = [&](int start, int end) {
        for (int pixel_idx = start; pixel_idx < end; pixel_idx++) {
            int i = pixel_idx % scene.width;
            int j = pixel_idx / scene.width;

            float x = (2 * (i + 0.5) / (float)scene.width - 1) *
                imageAspectRatio * scale;
            float y = (1 - 2 * (j + 0.5) / (float)scene.height) * scale;

            Vector3f dir = normalize(Vector3f(-x, y, 1));
            Vector3f color(0.0f);

            for (int k = 0; k < spp; k++) {
                color += scene.castRay(Ray(eye_pos, dir), 0);
            }
            color = color / spp;

            framebuffer[pixel_idx] = color;

            completed_pixels++;
        }
        };

    int chunk_size = total_pixels / num_threads; //单个线程渲染像素数
    for (int t = 0; t < num_threads; t++) {
        int start = t * chunk_size;
        int end = (t == num_threads - 1) ? total_pixels : start + chunk_size;
        threads.emplace_back(render_worker, start, end);
    }

    // 等待所有渲染线程完成
    for (auto& thread : threads) {
        thread.join();
    }

    // 等待进度线程完成
    progress_thread.join();
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
