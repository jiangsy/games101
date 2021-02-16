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

// Implementation of Path Tracing
Vector3f Scene::castRay(const Ray &ray, int depth) const
{
    auto isect = intersect(ray);
    auto color = Vector3f(0.f);

    if (isect.happened) {
        float pdf;
        Intersection light_isect;
        sampleLight(light_isect, pdf);
        auto light_line = light_isect.coords - isect.coords;
        auto light_dir = normalize(light_line);
        Ray test_ray = Ray(isect.coords, light_dir);
        auto test_isect = intersect(test_ray);
        if (norm_square(test_isect.coords - light_isect.coords) < 0.0001) {
            auto cos_theta = dotProduct(light_dir, isect.normal);
            auto cos_theta_p = dotProduct(-light_dir, light_isect.normal);
            auto dist = norm_square(light_line);
            color += light_isect.emit * cos_theta * cos_theta_p / dist *
                    isect.m->eval(-ray.direction, light_dir, isect.normal) / pdf;
        }

        if (get_random_float() < RussianRoulette) {
            Vector3f bounce_dir = isect.m->sample(-ray.direction, isect.normal);
            Ray bounce_ray(isect.coords, bounce_dir);
            auto bounce_iscet = intersect(bounce_ray);
            if (bounce_iscet.happened && !bounce_iscet.m->hasEmission()) {
                auto cos_theta = dotProduct(bounce_dir, isect.normal);
                color += castRay(bounce_ray, depth) * isect.m->eval(bounce_dir, -ray.direction, isect.normal) *
                        cos_theta / isect.m->pdf(bounce_dir, -ray.direction, isect.normal) / RussianRoulette;
            }
        }

    }
    return color;
}

