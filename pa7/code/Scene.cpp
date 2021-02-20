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
        if (isect.m->hasEmission()) {
            return isect.m->getEmission();
        }

        float light_pdf;
        Intersection light_isect;
        sampleLight(light_isect, light_pdf);
        auto light_line = light_isect.coords - isect.coords;
        auto light_dir = normalize(light_line);
        Ray test_ray = Ray(isect.coords, light_dir);
        auto test_isect = intersect(test_ray);
        if (norm_square(test_isect.coords - light_isect.coords) < 0.01) {
            auto cos_theta = dotProduct(light_dir, isect.normal);
            auto cos_theta_p = dotProduct(-light_dir, light_isect.normal);
            auto dist = norm_square(light_line);
            color += light_isect.emit * cos_theta_p * cos_theta / dist /
                     light_pdf * (isect.m->eval_microfacet(-ray.direction, light_dir, isect.normal, false)
                     + isect.m->eval_diffuse(-ray.direction, light_dir, isect.normal));
        }

        if (get_random_float() < RussianRoulette) {
            float prob_microfacet = get_random_float();
            if (isect.m->m_type == MICROFACET && prob_microfacet < 0.5) {
                Vector3f bounce_dir = isect.m->sample(-ray.direction, isect.normal, MICROFACET);
                Ray bounce_ray(isect.coords, bounce_dir);
                auto bounce_iscet = intersect(bounce_ray);
                if (bounce_iscet.happened && !bounce_iscet.m->hasEmission()) {
                    auto cos_theta = dotProduct(bounce_dir, isect.normal);
                    auto ray_rad = castRay(bounce_ray, depth);
                    color += ray_rad  *
                             cos_theta * isect.m->eval_microfacet(bounce_dir, -ray.direction, isect.normal, true)
                              / RussianRoulette * 2.0f;
                }
            }
            if (isect.m->m_type == DIFFUSE or prob_microfacet > 0.5f) {
                auto inv_microfacet_prob = isect.m->m_type == DIFFUSE ? 1.0f : 2.0f;
                Vector3f bounce_dir = isect.m->sample(-ray.direction, isect.normal, DIFFUSE);
                Ray bounce_ray(isect.coords, bounce_dir);
                auto bounce_iscet = intersect(bounce_ray);
                if (bounce_iscet.happened && !bounce_iscet.m->hasEmission()) {
                    auto cos_theta = dotProduct(bounce_dir, isect.normal);
                    color += castRay(bounce_ray, depth) * cos_theta / RussianRoulette *
                            isect.m->eval_diffuse(bounce_dir, -ray.direction, isect.normal) /
                            isect.m->pdf(bounce_dir, -ray.direction, isect.normal) *
                            inv_microfacet_prob;
                }
            }
        }

    }
    return color;
}

