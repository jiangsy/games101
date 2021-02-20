#include <iostream>
#include <vector>

#include "CGL/vector2D.h"

#include "mass.h"
#include "rope.h"
#include "spring.h"

namespace CGL {

    Rope::Rope(Vector2D start, Vector2D end, int num_nodes, float node_mass, float k, vector<int> pinned_nodes)
    {
        float kd = 0.05;
        auto interval = (end - start) / (num_nodes - 1);
        auto current_start(start);
        for (int i=0; i<num_nodes; i++) {
            masses.emplace_back(new Mass(current_start, node_mass, false));
            current_start += interval;
        }
        for (int i=0; i<num_nodes-1; i++) {
            springs.emplace_back(new Spring(masses[i], masses[i+1], k, kd));
        }
        for (auto &i : pinned_nodes) {
            masses[i]->pinned = true;
        }
    }

    void Rope::simulateEuler(float delta_t, Vector2D gravity)
    {
        for (auto &s : springs)
        {
            auto m1 = s->m1, m2 = s->m2;
            auto vec = m2->position - m1->position;
            auto dir = vec.unit();
            auto length = vec.norm();
            auto force = s->k * dir * (length - s->rest_length);
            m1->forces += force;
            m2->forces -= force;

            auto proj_velocity = dot(m2->velocity - m1->velocity, dir);
            auto damping_force = s->kd * proj_velocity * dir;
            m1->forces += damping_force;
            m2->forces -= damping_force;
        }

        for (auto &m : masses)
        {
            if (!m->pinned)
            {
                m->forces += gravity * m->mass;
                m->position += m->velocity * delta_t;
                m->velocity += (m->forces / m->mass) * delta_t;
            }

            // Reset all forces on each mass
            m->forces = Vector2D(0, 0);
        }
    }

    void Rope::simulateSemiImplicitEuler(float delta_t, Vector2D gravity)
    {
        for (auto &s : springs)
        {
            auto m1 = s->m1, m2 = s->m2;
            auto vec = m2->position - m1->position;
            auto dir = vec.unit();
            auto length = vec.norm();
            auto force = s->k * dir * (length - s->rest_length);
            m1->forces += force;
            m2->forces -= force;

            auto proj_velocity = dot(m2->velocity - m1->velocity, dir);
            auto damping_force = s->kd * proj_velocity * dir;
            m1->forces += damping_force;
            m2->forces -= damping_force;
        }

        for (auto &m : masses)
        {
            if (!m->pinned)
            {
                m->forces += gravity * m->mass;
                m->velocity += (m->forces / m->mass) * delta_t;
                m->position += m->velocity * delta_t;
            }

            // Reset all forces on each mass
            m->forces = Vector2D(0, 0);
        }
    }

    void Rope::simulateVerlet(float delta_t, Vector2D gravity )
    {
        for (auto &s : springs)
        {
            auto vec_12 = s->m2->position - s->m1->position;
            auto vec_12_norm = vec_12.norm();
            Vector2D f = s->k * vec_12 / vec_12_norm * (vec_12_norm - s->rest_length);
            s->m1->forces += f;
            s->m2->forces += -f;
        }

        for (auto &m : masses)
        {
            if (!m->pinned)
            {
                Vector2D temp_position = m->position;
                m->forces += gravity * m->mass;
                m->position = m->position + (1-0.0005 * delta_t) * (m->position - m->last_position) +
                        (m->forces / m->mass) * delta_t * delta_t;
                m->last_position = temp_position;
            }
            m->forces = Vector2D(0, 0);
        }
    }
}
