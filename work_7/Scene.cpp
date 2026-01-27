//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"


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

Vector3f Scene::castRay(const Ray &ray, int depth) const
{
    //计算着色点信息
    Intersection p = intersect(ray);
    if (p.happened == false) return Vector3f(0, 0, 0);
    if (p.m->hasEmission()) return p.m->getEmission();

    Vector3f l_dir(0,0,0);
    Vector3f l_indir(0,0,0);

    //计算直接光照(光源到观察点)
    Intersection lightInter;
    float light_pdf=0.0f;
    sampleLight(lightInter, light_pdf);

    Vector3f wo = ray.direction;
    wo = wo.normalized();
    Vector3f p_pos = p.coords; 
    Vector3f light_pos = lightInter.coords;
    Vector3f light_dir = (light_pos - p_pos).normalized(); 
    Vector3f NN = lightInter.normal.normalized(); //area light's normal
    Vector3f N = p.normal.normalized(); //shading point p's normal
    float directDistance = (p_pos - light_pos).norm(); 
    Vector3f light_emit = lightInter.emit; 

    //判断光源是否被遮挡

    Ray wsRay(p_pos, light_dir);
    Intersection wsInter = intersect(wsRay);

    if (wsInter.distance - directDistance > -0.0001) {
        l_dir = light_emit * p.m->eval(wo, light_dir, N) *
            dotProduct(light_dir, N) * dotProduct(-light_dir, NN) /
            (directDistance * directDistance) / light_pdf;
    }
    
    //计算间接光照(其他物体到观察点)
    if (get_random_float() > RussianRoulette)
        return l_dir;

    Vector3f wi = p.m->sample(wo,N).normalized();
    Ray wiRay(p_pos, wi);

    Intersection woInter = intersect(wiRay);
    if (woInter.happened && !woInter.obj->hasEmit()) {
        l_indir = castRay(wiRay, depth + 1) * p.m->eval(wo, wi, N) * dotProduct(wi, N)
            / p.m->pdf(wo, wi, N) / RussianRoulette;
    }
    
    return l_dir + l_indir; 
}