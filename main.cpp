//
//  main.cpp
//  ConsoleRayTracing
//
//  Created by Ninter6 on 2023/7/20.
//

#include <iostream>
#include <vector>
#include <thread>
#if defined(_WIN32) || defined(_WIN64)
#include <conio.h>
#else
#include <termios.h>
#endif

#define MATHPLS_DEFINITION
#include "mathpls.h"

using namespace std;
using vec = mathpls::vec<double, 3>;
using mathpls::sin;
using mathpls::cos;
using mathpls::tan;

struct Camera
{
    Camera() = default;
    Camera(mathpls::ivec2 imageSize, vec pos, vec dir)
    : asp(imageSize.x/imageSize.y/1.5), pos(pos), direction(dir.normalize()) {
        right = mathpls::cross(direction, {0, 1, 0}).normalize();
        down = mathpls::cross(direction, right).normalize();
    }
    
    vec pos;
    vec direction, right, down;
    double f = 1, fov = mathpls::radians(60), asp;
    
    /**
     * 获取光线方向
     * @param xof 左0, 右1
     * @param yof 上0, 下1
     * @result 对应方向
     */
    vec GetRayDirection(double xof, double yof) const {
        auto height = static_cast<double>(mathpls::tan(fov/2) * f) * 2;
        auto width = height * asp;
        auto leftTop = direction * f - down * height / 2 + - right * width / 2;
        
        return (leftTop + xof * right * width + yof * down * height).normalize();
    }
    
    void Move(vec d) {
        pos += d.x * right - d.y * down + d.z * direction;
    }
    
    void Rotate(vec a) {
        // 预处理
        if(mathpls::abs(direction.y) > .9 && direction.y * a.x < 0) a.x = 0;
        
        mathpls::vec4 v(direction.x, direction.y, direction.z, 1);
        mathpls::quat qx(mathpls::vec3(right.x, 0, right.z), a.x), qy({0, 1, 0}, a.y);
        v = mathpls::rotate(qx) * mathpls::rotate(qy) * v;
        direction = {v.x, v.y, v.z};
        right = mathpls::cross(direction, {0, 1, 0}).normalize();
        down = mathpls::cross(direction, right).normalize();
    }
};

struct Ray {
    vec Origin;
    vec Direction;
    
    vec at(double t) const {return Origin + t * Direction;}
};

struct Ball
{
    Ball() {};
    Ball(vec pos, double r, double albedo)
        : pos(pos), r(r), albedo(albedo) {};
    vec pos = vec(0, 0, 0);
    double r = 0;
    double albedo = 100;
};

class Renderer {
public:
    Renderer() = default;
    Renderer(mathpls::ivec2 imageSize, const Camera& cam, const std::vector<Ball>& balls)
    : imageSize(imageSize), cam(cam), balls(balls) {
        image.resize(imageSize.x * imageSize.y);
    }
    
    void Render() {
        constexpr int tn = 8; // 设置渲染线程数
        const int e = imageSize.y / tn;
        
        static auto func = [&](int n){
            for(int y = n * e; y < (n+1 == tn ? imageSize.y : n * e + e); y++)
                for(int x = 0; x < imageSize.x; x++)
                    PrePixel({x, y});
        };
        
        std::vector<std::thread> t;
        for (int i = 0; i < tn; i++) t.emplace_back(func, i);
        for (auto& i : t) i.join();
    }
    
    Camera& camera() {return cam;}
    
    uint8_t* GetImage() {return image.data();}
    
    mathpls::ivec2 GetImageSize() {return imageSize;}
    
private:
    mathpls::ivec2 imageSize;
    Camera cam;
    std::vector<Ball> balls;
    
    std::vector<uint8_t> image;
    
    struct HitPayload {
        double HitDistance;
        vec WorldPosition;
        vec WorldNormal;
        int ObjectIndex;
    };
    
    HitPayload TracRay(const Ray& ray) {
        float hitDistance = std::numeric_limits<float>::max();
        int clstSphere = -1;
        
        for(int i = 0; i < balls.size(); i++){
            vec origin = ray.Origin - balls[i].pos;
            float radius = balls[i].r;
            
            // (bx^2 + by^2 + bz^2)t^2 + 2(axbx + ayby + azbz)t + (ax^2 + ay^2 +az^2 - r^2) = 0
            // where
            // a = ray origin
            // b = ray direction
            // r = radius
            // t = hit distance
            
            // q = 1
            float l = 2 * mathpls::dot(origin, ray.Direction);
            float c = mathpls::dot(origin, origin) - radius*radius;
            
            float discriminant = l*l - 4*c;
            if(discriminant >= 0 && -l - mathpls::sqrt(discriminant) >= 0){
                auto t = (-l - mathpls::sqrt(discriminant)) / 2;
                if(t < hitDistance){
                    hitDistance = t;
                    clstSphere = i;
                }
            }
        }
        
        if (clstSphere != -1) {
            return ClosestHit(ray, hitDistance, clstSphere);
        } else {
            return Miss(ray);
        }
    }
    HitPayload ClosestHit(const Ray& ray, double hitDistance, int objectIndex) {
        const auto& clstSphere = balls[objectIndex];
        
        HitPayload payload;
        payload.HitDistance = hitDistance;
        payload.ObjectIndex = objectIndex;
        payload.WorldPosition = ray.at(hitDistance);
        payload.WorldNormal = (ray.at(hitDistance) - clstSphere.pos).normalize();
        
        return payload;
    }
    HitPayload Miss(const Ray& ray) {
        return {-1};
    }
    
    void PrePixel(mathpls::ivec2 pos) {
        Ray ray{cam.pos, cam.GetRayDirection((double)pos.x / (imageSize.x-1), (double)pos.y / (imageSize.y-1))};
        constexpr int bounces = 5;
        
        double light = 1;
        
        for (int i = 0; i < bounces; i++) {
            HitPayload payload = TracRay(ray);
            
            double t = (ray.Origin.y + 10) / -ray.Direction.y;
            if ((payload.HitDistance == -1 || t < payload.HitDistance) && t > 0) {
                    vec o = cam.pos + ray.Direction * t;
                    if ((int)(mathpls::abs(o.x/2)+.5)%2 == (int)(mathpls::abs(o.z/2)+.5)%2) {
                        light *= .3779;
                        break;
                    }
            }
            if(payload.HitDistance == -1) {
                double bise = mathpls::dot(ray.Direction, {0, 1, 0}) / 2. + .5;
                light *= vec{1 - .5*bise, 1 - .3*bise, 1}.length() / mathpls::sqrt(3);
                break;
            }
            
            light *= balls[payload.ObjectIndex].albedo;
            ray = {payload.WorldPosition + payload.WorldNormal * .000001, mathpls::reflect(ray.Direction, payload.WorldNormal).normalize()};
        }
        
        constexpr uint8_t color[10]{ '@', '%', '#', '&', '*', '+', '=', '-', '.', ' ' };
        image[pos.x + pos.y*imageSize.x] = color[mathpls::mid<int>(0, mathpls::pow(light, 1 / 2.2) * 10, 9)];
    }
    
};

ostream& operator<<(ostream& os, vec v) {
    return os << v.x << " " << v.y << " " << v.z;
}

class Screen {
public:
    Screen() = default;
    Screen(const Renderer& renderer) : renderer(renderer) {}
    
    void Loop() {
        while (!shouldBreak) {
            renderer.Render();
            auto image = renderer.GetImage();
#if defined(_WIN32) || defined(_WIN64)
            system("cls");
#else
            system("clear");
#endif
            cout<<"Direction: "<<renderer.camera().direction<<endl;
            cout<<"Right: "<<renderer.camera().right<<endl;
            for(int y = 0; y < renderer.GetImageSize().y; y++){
                for(int x = 0; x < renderer.GetImageSize().x; x++)
                    cout<<image[x + y*renderer.GetImageSize().x];
                endl(cout);
            }
            ProcessInput();
        }
    }
    
private:
    Renderer renderer;
    bool shouldBreak = false;
    
    void ProcessInput() {
#if defined(_WIN32) || defined(_WIN64)
        switch (getch())
#else
        switch (scanKeyboard())
#endif
        {
            case 'w':
                renderer.camera().Move({0, 0, .1});
                break;
                
            case 's':
                renderer.camera().Move({0, 0, -.1});
                break;
                
            case 'a':
                renderer.camera().Move({-.1, 0, 0});
                break;
                
            case 'd':
                renderer.camera().Move({.1, 0, 0});
                break;
                
            case 'e':
                renderer.camera().Move({0, .1, 0});
                break;
                
            case 'q':
                renderer.camera().Move({0, -.1, 0});
                break;
                
            case 'i':
                renderer.camera().Rotate({3.1415 / 36, 0, 0});
                break;
                
            case 'k':
                renderer.camera().Rotate({-3.1415 / 36, 0, 0});
                break;
                
            case 'j':
                renderer.camera().Rotate({0, 3.1415 / 36, 0});
                break;
                
            case 'l':
                renderer.camera().Rotate({0, -3.1415 / 36, 0});
                break;
                
            case 27:
                shouldBreak = true;
                break;
                
            default:
                break;
        }
    }
    
    /**
     * Unix无缓冲输入
     */
    char scanKeyboard(){
        termios new_settings;
        termios stored_settings;
        tcgetattr(0,&stored_settings);
        new_settings = stored_settings;
        new_settings.c_lflag &= (~ICANON);
        new_settings.c_cc[VTIME] = 0;
        tcgetattr(0,&stored_settings);
        new_settings.c_cc[VMIN] = 1;
        tcsetattr(0,TCSANOW,&new_settings);

        char in = getchar();

        tcsetattr(0,TCSANOW,&stored_settings);
        return in;
    }
};

int main(int argc, const char * argv[]) {
    const Camera& cam{{200, 72}, {0}, {0, 0, -1}};
    const std::vector<Ball>& balls = {
        // pos, radius, albedo
        {{0, 0, -3}, .5, .5},
        {{0, -8, 4}, 5, .3},
        {{8, 0, 0}, 1, 2},
        {{8, 0, 0}, 1.5, .3},
        {{0, 2, 4}, 5, .1},
        {{0, 0, 10}, 2, 1},
        {{-100, 0, 50}, 50, .1},
        {{8, 0, 0}, 1.5, .3}
    };
    const Renderer& renderer{{200, 72}, cam, balls};
    Screen scr{renderer};
    scr.Loop();
    
    return 0;
}
