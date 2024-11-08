#ifndef GRAVITY_GENERATOR_HPP
#define GRAVITY_GENERATOR_HPP

#include "../systems/particle_system.hpp"
#include "../types/v3_f32.hpp"
#include "../objects/mass_particle.hpp"

namespace generators {
    struct gravity_generator {
        objects::mass_f32 mass;
        types::v3_f32 position;
        types::f32 gravitational_constant;

        static constexpr types::f32 earth_mass = 5.972e24f;
        static constexpr types::f32 earth_radius = 6371000.0f;
        static constexpr types::f32 default_gravitational_constant = 6.6743e-11f;
        gravity_generator(
            types::v3_f32 position,
            objects::mass_f32 mass,
            types::f32 gravitational_constant = default_gravitational_constant
        );

        void apply_to_particles(
            systems::particle_system &particle_system,
            objects::seconds_f64 delta_time
        );
    };
}

#endif