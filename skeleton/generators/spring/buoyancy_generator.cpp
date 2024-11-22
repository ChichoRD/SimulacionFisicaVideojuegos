#include <iostream>

#include "buoyancy_generator.hpp"

generators::buoyancy_generator::buoyancy_generator(
    objects::position3_f32 base_point,
    types::v3_f32 liquid_height,
    types::f32 liquid_volume,
    types::f32 liquid_density
) : base_point(base_point), liquid_height(liquid_height), liquid_volume(liquid_volume), liquid_density(liquid_density) { }

void generators::buoyancy_generator::apply_to_particles(
    systems::particle_system &particle_system,
    objects::seconds_f64 delta_time
) {
    particle_system.iter<
        objects::particle::deconstruct_position const,
        objects::generators::particle_force
    >(
        [this](objects::particle::deconstruct_position const &position,
            objects::generators::particle_force &force) {
            types::v3_f32 f = buoyancy_force(
                position,
                2.0f,
                this->base_point,
                this->liquid_height,
                this->liquid_volume,
                this->liquid_density
            );
            force += f;
        }
    );
}

types::v3_f32 generators::buoyancy_generator::buoyancy_force(
    objects::position3_f32 const &particle_position,
    types::f32 particle_radius,
    objects::position3_f32 const &base_point,
    types::v3_f32 const &liquid_height,
    types::f32 liquid_volume,
    types::f32 liquid_density
) {
    types::v3_f32 local_position = types::v3_f32{particle_position} - base_point;
    types::f32 height = liquid_height.magnitude();
    types::f32 local_height = types::v3_f32::dot(local_position, liquid_height) / height;

    types::f32 submersion = 0.0f;
    if (local_height - particle_radius > height) {
        submersion = 0.0f;
    } else if (local_height + particle_radius < height) {
        submersion = 1.0f;
    } else {
        submersion = (height - local_height + particle_radius) / (2.0f * particle_radius);
    }

    return liquid_density * liquid_volume * submersion * liquid_height.normalized();
}
