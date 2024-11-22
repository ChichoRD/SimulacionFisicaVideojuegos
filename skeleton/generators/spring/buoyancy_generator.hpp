#ifndef BUOYANCY_GENERATOR_HPP
#define BUOYANCY_GENERATOR_HPP

#include "../../systems/particle_system.hpp"
#include "../../types/v3_f32.hpp"
#include "../../objects/mass_particle.hpp"

namespace generators {
    struct buoyancy_generator {
        objects::position3_f32 base_point;
        types::v3_f32 liquid_height;

        types::f32 liquid_volume;
        types::f32 liquid_density;

        buoyancy_generator(
            objects::position3_f32 base_point,
            types::v3_f32 liquid_height,
            types::f32 liquid_volume,
            types::f32 liquid_density
        );

        void apply_to_particles(
            systems::particle_system &particle_system,
            objects::seconds_f64 delta_time
        );

        static types::v3_f32 buoyancy_force(
            objects::position3_f32 const &particle_position,
            types::f32 particle_radius,
            objects::position3_f32 const &base_point,
            types::v3_f32 const &liquid_height,
            types::f32 liquid_volume,
            types::f32 liquid_density
        );
    };
}

#endif