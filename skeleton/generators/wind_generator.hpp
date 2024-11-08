#ifndef WIND_GENERATOR_HPP
#define WIND_GENERATOR_HPP

#include "../systems/particle_system.hpp"
#include "../types/v3_f32.hpp"
#include "../objects/mass_particle.hpp"

namespace generators {
    struct wind_generator {
        types::v3_f32 area_centre;
        types::f32 area_radius;

        types::f32 k1 = 1.0f;
        types::f32 k2 = 0.0f;

        types::v3_f32 wind_velocity;

        wind_generator(
            types::v3_f32 area_centre,
            types::f32 area_radius,
            types::v3_f32 wind_velocity,
            types::f32 k1 = 1.0f,
            types::f32 k2 = 0.0f
        );

        void apply_to_particles(systems::particle_system &particle_system, objects::seconds_f64 delta_time);
    };
    
    struct tornado_generator {
        types::v3_f32 column_base;
        types::v3_f32 column_direction;
        types::f32 height;
        types::f32 radius;
        types::f32 K;

        tornado_generator(
            types::v3_f32 column_base,
            types::v3_f32 column_direction,
            types::f32 height,
            types::f32 radius,
            types::f32 K
        );

        void apply_to_particles(systems::particle_system &particle_system, objects::seconds_f64 delta_time);
    };
}

#endif