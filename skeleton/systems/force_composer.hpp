#ifndef FORCE_COMPOSER_HPP
#define FORCE_COMPOSER_HPP

#include <tuple>
#include <type_traits>
#include <cmath>

#include "particle_system.hpp"
#include "../objects/particle.hpp"
#include "../objects/mass_particle.hpp"
#include "../objects/solid_particle.hpp"

namespace systems {
    template <typename Generator>
    struct force_composer {
        Generator generator;

        force_composer(Generator&& generator) : generator(generator) {

        }

        void apply_to_particles(particle_system &particle_system, float delta_time_seconds) {
            for (size_t i = 0; i < particle_system.particles.particle_count(); ++i) {
                particle_system.set(
                    i,
                    objects::generators::particle_force{0.0f, 0.0f, 0.0f}
                );
                
                if (particle_system.particles.particle_has_attribute<
                        objects::solid_dynamic_particle::deconstruct_rigid_dynamic
                    >(i)) {
                    particle_system.set(
                        i,
                        objects::generators::particle_torque{0.0f, 0.0f, 0.0f}
                    );
                }
            }
            generator.apply_to_particles(particle_system, delta_time_seconds);
        }

        void compose_forces(particle_system &particle_system, float delta_time_seconds) {
            particle_system.iter<
                objects::generators::particle_force const,
                objects::mass_particle::deconstruct_inverse_mass,
                objects::particle::deconstruct_previous_position,
                objects::particle::deconstruct_position,
                objects::particle::deconstruct_velocity
            >(
                [delta_time_seconds](objects::generators::particle_force const &force,
                    objects::mass_particle::deconstruct_inverse_mass &inverse_mass,
                    objects::particle::deconstruct_previous_position &previous_position,
                    objects::particle::deconstruct_position &position,
                    objects::particle::deconstruct_velocity &velocity) {
                    objects::mass_particle particle = objects::mass_particle(
                        previous_position,
                        position,
                        velocity,
                        inverse_mass
                    );
                    particle.integrate(force * inverse_mass.value, 0.9875f, delta_time_seconds);
                    
                    previous_position = particle.particle.previous_position;
                    position = particle.particle.position;
                    velocity = particle.particle.velocity;
                    inverse_mass.value = particle.inverse_mass;
                }
            );

            // PHYSX
            particle_system.iter<
                objects::generators::particle_force const,
                objects::generators::particle_torque const,
                objects::solid_dynamic_particle::deconstruct_rigid_dynamic
            >(
                [delta_time_seconds](objects::generators::particle_force const &force,
                    objects::generators::particle_torque const &torque,
                    objects::solid_dynamic_particle::deconstruct_rigid_dynamic &rigid_dynamic) {
                    if (!std::isnan(force.x) && !std::isnan(force.y) && !std::isnan(force.z)) {
                        rigid_dynamic->addForce(force);
                    }

                    if (!std::isnan(torque.x) && !std::isnan(torque.y) && !std::isnan(torque.z)) {
                        rigid_dynamic->addTorque(torque);
                    }
                }
            );
        }
    };
}

#endif