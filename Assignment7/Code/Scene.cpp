//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"
#include "Material.hpp"
#include "Object.hpp"
#include "global.hpp"
#include <cmath>
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
    constexpr float rayOriginOffset = 0.001f;
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

        //Direct Light
        float pdf = 0;
        Intersection light_inter;//a pos in light source
        sampleLight(light_inter,pdf);//get pdf(intersection)
        Vector3f w_i = ray.direction;//ray in
        Vector3f w_o = (light_inter.coords - hitPoint);//ray out
        float distance2 = dotProduct(w_o, w_o);
        if (pdf > 0.0f && distance2 > 0.0f)
        {
            float distance = std::sqrt(distance2);
            Vector3f w_o_n = w_o / distance;
            float cosSurface = dotProduct(N, w_o_n);
            float cosLight = dotProduct(light_inter.normal, -w_o_n);
            if (cosSurface > 0.0f && cosLight > 0.0f)
            {
                Ray p2light(hitPoint + N * rayOriginOffset, w_o_n);
                Intersection firstHit = Scene::intersect(p2light);
                // This scene has one emitter. Its first hit means the sample is visible.
                if (firstHit.happened && firstHit.m && firstHit.m->hasEmission())
                {
                    Vector3f f_r = m->eval(w_i, w_o_n, N);
                    L_dir = light_inter.emit * f_r * cosSurface * cosLight / distance2 / pdf;
                }
            }
        }

        //indirect light
        Vector3f L_indir = Vector3f();
        //use russian roulette
        if(get_random_float() > RussianRoulette)//this case we don't  calculate indirect light. other case we suppose a diffuse light
        {
            return L_dir;
        }
        // A microfacet sample can be invalid when its reflected ray leaves the surface.
        Vector3f random_wo = normalize(m->sample(w_i, N));
        float pdf_wo = m->pdf(w_i, random_wo, N);
        if(pdf_wo <= 0.0f)
            return L_dir;
        //if sth. blocked on the way, this inter.coord return the blocked position(new texel to be rendered)
        Ray indir_ray(hitPoint + N*rayOriginOffset,random_wo);
        Intersection next_inter = Scene::intersect(indir_ray);
        //if hit the sth., calculate Q point
        if(next_inter.happened && next_inter.m)
        {
            // Direct lighting from the emitter is already estimated above.
            if(next_inter.m->hasEmission())
                return L_dir;
            Vector3f indir_f_r = m->eval(w_i , random_wo , N);
            L_indir = castRay(indir_ray, depth+1) * indir_f_r * dotProduct(random_wo, N) / pdf_wo / RussianRoulette;
        }

        return L_dir + L_indir;
    }
    return Vector3f();//no intersection
}
