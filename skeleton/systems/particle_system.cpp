#include "particle_system.hpp"

namespace systems {
    particle_system::particle_system(particle_generator generator, size_t lifetime_milliseconds)
        : particles(),
        dead_particles(),
        generator(generator),
        start_time(std::time(nullptr)),
        lifetime_milliseconds(lifetime_milliseconds) {}

    particle_id particle_system::add_particle() {
        if (dead_particles.empty()) {
            particle_id particle = particles.particle_count();
            particles.set_particle_attribute(particle, particle_meta{true, false});
            return particle;
        } else {
            particle_id particle = dead_particles.front();
            dead_particles.pop();
            return particle;
        }
    }

    particle_id particle_system::remove_particle(particle_id particle) {
        particles.clear_particle_attributes(particle);
        particles.set_particle_attribute(particle, particle_meta{false, false});
        dead_particles.push(particle);
        return particle;
    }
}