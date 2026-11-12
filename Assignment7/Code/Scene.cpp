//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"
#include "Material.hpp"
#include "Object.hpp"
#include "global.hpp"
#include <cstdint>


void Scene::buildBVH() {
    printf(" - Generating BVH...\n\n");
    this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::NAIVE);
}

Intersection Scene::intersect(const Ray &ray) const
{
    return this->bvh->Intersect(ray);
}

void Scene::sampleLight(Intersection &pos, float &pdf) const
{
    float emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
        }
    }
    float p = get_random_float() * emit_area_sum;
    emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
            if (p <= emit_area_sum){
                objects[k]->Sample(pos, pdf);
                break;
            }
        }
    }
}

bool Scene::trace(
        const Ray &ray,
        const std::vector<Object*> &objects,
        float &tNear, uint32_t &index, Object **hitObject)
{
    *hitObject = nullptr;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        float tNearK = kInfinity;
        uint32_t indexK;
        Vector2f uvK;
        if (objects[k]->intersect(ray, tNearK, indexK) && tNearK < tNear) {
            *hitObject = objects[k];
            tNear = tNearK;
            index = indexK;
        }
    }


    return (*hitObject != nullptr);
}

// Implementation of Path Tracing
Vector3f Scene::castRay(const Ray& ray,int depth) const
{
    // TO DO Implement Path Tracing Algorithm here
    //use russian roulette to constrain max depth.()
    //gobal illuminate

    //get ray intersection (the first position to be rendered)
    Intersection intersection = Scene::intersect(ray);
    Material* m = intersection.m;
    Object* hitObject = intersection.obj;
    Vector3f hitColor = this->backgroundColor;
    //float tnear = kInfinity;
    Vector2f uv;
    uint32_t index = 0;//actually of no use
    //calculate intersection's color
    if(intersection.happened)
    {
        Vector3f L_dir = Vector3f();
        if(m->hasEmission())
        {
            return m->getEmission();//if meet a light source, just return its own emission and don't calculate L_d+L_ind
        }
        Vector3f hitPoint = intersection.coords;
        Vector3f N = intersection.normal;
        Vector2f st;//texture coordinate
        hitObject->getSurfaceProperties(hitPoint, ray.direction ,index, uv,  N,  st);

        //as no reflective or refractive material in this model,there's no need for us to calculate the Reflection&Refraction case in the function 
        //and we don't use Blinn-Phong model , so we don't calculate shadow ray
        //case DIFFUSE:
        //Direct Light
        float pdf = 0;
        Intersection light_inter;//a pos in light source
        sampleLight(light_inter,pdf);//get pdf(intersection)
        Vector3f w_i = ray.direction;//ray in
        Vector3f w_o = (light_inter.coords - hitPoint);//ray out
        Vector3f w_o_n = w_o.normalized();
        //generate a ray from hitpoint to light sampled point
        Ray p2light = Ray(hitPoint + N*EPSILON,w_o_n);
        //this ray intersect with another object to calculate indirect light (if hit something)
        Intersection p2light_inter = Scene::intersect(p2light);
        if(p2light_inter.distance > (w_o.norm() - EPSILON))//-epsilon to avoid float error ,, and it includes == case
        //no object blocked.
        {
            //calculat direct illumination
            Vector3f f_r = m->eval(w_i, w_o_n, N);
            float distance2 = dotProduct(w_o, w_o);
            L_dir = light_inter.emit * f_r * dotProduct(N, w_o_n) * dotProduct(light_inter.normal, -w_o_n) / distance2 / pdf;
        }

        //indirect light
        Vector3f L_indir = Vector3f();
        //use russian roulette
        if(get_random_float() > RussianRoulette)//this case we don't  calculate indirect light. other case we suppose a diffuse light
        {
            return L_dir;
        }
        //as a diffuse material ,our sample function actually is a random sample (same propablity)
        Vector3f random_wo = m->sample(w_i, N).normalized();
        //if sth. blocked on the way, this inter.coord return the blocked position(new texel to be rendered)
        Ray indir_ray(hitPoint + N*EPSILON,random_wo);
        Intersection indir_ray_inter = Scene::intersect(indir_ray);
        //if hit the sth., calculate Q point
        if(indir_ray_inter.happened)
        {
            float pdf_o = m->pdf(w_i,random_wo, N);
            Vector3f indir_f_r = m->eval(w_i , random_wo , N);
            L_indir = castRay(indir_ray, depth+1) * indir_f_r * dotProduct(random_wo, N) / pdf_o / RussianRoulette;
        }

        return L_dir + L_indir;
    }
    return Vector3f();//no intersection
    //@fixme：：目前没有把hitpoint按光线在内部外部进行 + epsilon（这个模型全是diffuse，按理说+epsilon即可）
}