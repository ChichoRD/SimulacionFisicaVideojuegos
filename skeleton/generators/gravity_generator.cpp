#include <iostream>
#include "gravity_generator.hpp"
#include "../objects/solid_particle.hpp"

generators::gravity_generator::gravity_generator(
    types::v3_f32 position,
    objects::mass_f32 mass,
    types::f32 gravitational_constant
) : mass(mass), position(position), gravitational_constant(gravitational_constant) { }

static types::v3_f32 gravity_force(
    types::v3_f32 from_position,
    types::v3_f32 to_position,
    objects::inverse_mass_f32 from_inverse_mass,
    objects::inverse_mass_f32 to_inverse_mass,
    types::f32 gravitational_constant
) {
    // F = K * m1 * m2 / r^2
    types::v3_f32 displacement = from_position - to_position;
    types::v3_f32 gravitational_force =
        gravitational_constant
        * displacement.normalized()
        / (from_inverse_mass * to_inverse_mass * displacement.magnitude_sqr());
    return gravitational_force;
}

void generators::gravity_generator::apply_to_particles(
    systems::particle_system &particle_system,
    objects::seconds_f64 delta_time
) {
    particle_system.iter<
        objects::mass_particle::deconstruct_inverse_mass const,
        objects::particle::deconstruct_position const,
        objects::generators::particle_force
    >(
        [this](objects::mass_particle::deconstruct_inverse_mass const &inverse_mass,
            objects::particle::deconstruct_position const &position,
            objects::generators::particle_force &force) {

            force += gravity_force(
                this->position,
                types::v3_f32{position},
                1.0f / this->mass,
                inverse_mass.value,
                this->gravitational_constant
            );
        }
    );

    particle_system.iter<
        objects::solid_dynamic_particle::deconstruct_rigid_dynamic const,
        objects::generators::particle_force
    >(
        [this](objects::solid_dynamic_particle::deconstruct_rigid_dynamic const &rigid_dynamic,
            objects::generators::particle_force &force) {
            types::v3_f32 position = types::v3_f32{rigid_dynamic->getGlobalPose().p};
            force += gravity_force(
                this->position,
                position,
                1.0f / this->mass,
                rigid_dynamic->getInvMass(),
                this->gravitational_constant
            );
        }
    );
}
