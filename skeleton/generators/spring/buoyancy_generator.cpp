#include "buoyancy_generator.hpp"

generators::buoyancy_generator::buoyancy_generator(
    objects::position3_f32 base_point,
    types::v3_f32 liquid_height,
    types::f32 liquid_density,
    types::f32 const &gravity
) : base_point(base_point), liquid_height(liquid_height), liquid_density(liquid_density), gravity(gravity) { }

void generators::buoyancy_generator::apply_to_particles(
    systems::particle_system &particle_system,
    objects::seconds_f64 delta_time
) {
    types::f32 liquid_density = this->liquid_density;
    types::f32 gravity = this->gravity;
    particle_system.iter_indexed<
        objects::particle::deconstruct_position const,
        objects::generators::particle_force
    >(
        [this, liquid_density, gravity](systems::particle_id particle_index,
            objects::particle::deconstruct_position const &position,
            objects::generators::particle_force &force) {
            force += buoyancy_force(position, this->base_point, this->liquid_height, liquid_density, gravity);
        }
    );
}

types::v3_f32 generators::buoyancy_generator::buoyancy_force(
    objects::position3_f32 const &particle_position,
    objects::position3_f32 const &base_point,
    types::v3_f32 const &liquid_height,
    types::f32 liquid_density,
    types::f32 gravity
) {
    types::v3_f32 local_position = types::v3_f32{particle_position} - base_point;
    types::f32 height = liquid_height.magnitude();

    types::f32 local_height = types::v3_f32::project(local_position, liquid_height).magnitude();
    types::f32 submersion = 0.0f;
    if (local_height < height) {
        submersion = 1.0f;
    } else if (local_height > height) {
        submersion = 0.0f;
    } else {
        submersion = 1.0f - (local_height / height);
    }

    return liquid_density * gravity * submersion * liquid_height.normalized();
}
