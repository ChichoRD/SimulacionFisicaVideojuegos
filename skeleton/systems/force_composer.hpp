#ifndef FORCE_COMPOSER_HPP
#define FORCE_COMPOSER_HPP

#include <tuple>
#include <type_traits>

#include "particle_system.hpp"
#include "../objects/particle.hpp"
#include "../objects/mass_particle.hpp"

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
        }
    };
}

#endif