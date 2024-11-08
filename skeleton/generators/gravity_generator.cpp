#include <iostream>
#include "gravity_generator.hpp"

generators::gravity_generator::gravity_generator(
    types::v3_f32 position,
    objects::mass_f32 mass,
    types::f32 gravitational_constant
) : mass(mass), position(position), gravitational_constant(gravitational_constant) { }

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

            // F = K * m1 * m2 / r^2
            types::v3_f32 displacement = this->position - types::v3_f32{position};
            types::v3_f32 gravitational_force =
                gravitational_constant
                * displacement.normalized()
                * (this->mass / (inverse_mass.value * displacement.magnitude_sqr()));
        
            force += gravitational_force;
        }
    );
}
