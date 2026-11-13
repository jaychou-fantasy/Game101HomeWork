#include <iostream>
#include <vector>

#include "CGL/vector2D.h"
#include "application.h"
#include "mass.h"
#include "rope.h"
#include "spring.h"

namespace CGL {

    Rope::Rope(Vector2D start, Vector2D end, int num_nodes, float node_mass, float k, vector<int> pinned_nodes)
    {
        // TODO (Part 1): Create a rope starting at `start`, ending at `end`, and containing `num_nodes` nodes.

        //        Comment-in this part when you implement the constructor
        //        for (auto &i : pinned_nodes) {
        //            masses[i]->pinned = true;
        //        }
        Vector2D distance = (end - start) / (num_nodes - 1);
        for(int i = 0;i < num_nodes; ++i)
        {
            Mass* mass = new Mass(start + distance*i,node_mass,false);
            masses.push_back(mass);
            if(i>0)
            {
                Spring* spring = new Spring(masses[i-1],masses[i],k);
                springs.push_back(spring);
            }
        }
        for(auto& i : pinned_nodes)
        {
            masses[i]->pinned = true;
        }
    }

    void Rope::simulateEuler(float delta_t, Vector2D gravity)
    {
        for (auto &s : springs)
        {
            // TODO (Part 2): Use Hooke's law to calculate the force on a node
            Vector2D M1toM2 = s->m2->position - s->m1->position;
            //this m1 is the last spring'm2
            //so accumulate the force
            Vector2D f = s->k * (M1toM2).unit() * ((M1toM2).norm() - s->rest_length);
            s->m1->forces += f;
            s->m2->forces += -f;
        }

        float k_damp = 0.0005;

        for (auto &m : masses)
        {
            if (!m->pinned)
            {
                // TODO (Part 2): Add the force due to gravity, then compute the new velocity and position
                m->forces += gravity * m->mass;
                // TODO (Part 2): Add global damping
                m->forces += -k_damp * m->velocity;
                Vector2D a = m->forces / m->mass;

                //explict Euler
                // m->position += m->velocity * delta_t;//update position with last velocity first
                // m->velocity += a * delta_t;

                //implcit Euler
                m->velocity += a*delta_t;//update Velocity first
                m->position += m->velocity * delta_t;
            }
            // Reset all forces on each mass
            m->forces = Vector2D(0, 0);
        }
    }

    void Rope::simulateVerlet(float delta_t, Vector2D gravity)
    {
        for (auto &s : springs)
        {
            // TODO (Part 3): Simulate one timestep of the rope using explicit Verlet （solving constraints)
            Vector2D M1toM2 = s->m2->position - s->m1->position;
            Vector2D f = s->k * (M1toM2).unit() * ((M1toM2).norm() - s->rest_length);
            s->m1->forces += f;
            s->m2->forces += -f;
        }

        double k_damp = 0.00005;

        for (auto &m : masses)
        {
            if (!m->pinned)
            {
                m->forces += gravity * m->mass;
                Vector2D a = m->forces / m->mass;
                
                Vector2D temp_position = m->position;
                // TODO (Part 3.1): Set the new position of the rope mass
                // TODO (Part 4): Add global Verlet damping
                m->position += (1 - k_damp) * (m->position - m->last_position) + a*delta_t*delta_t;
                m->last_position = temp_position;
            }
             // Reset all forces on each mass
            m->forces = Vector2D(0, 0);
        }
    }
}
