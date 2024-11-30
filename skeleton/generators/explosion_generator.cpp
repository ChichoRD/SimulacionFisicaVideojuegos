#include <cassert>
#include "explosion_generator.hpp"

generators::explosion_generator::explosion_generator(
    types::v3_f32 explosion_centre,
    types::f32 K,
    types::f32 radius,
    types::f32 fade_time_constant
) : explosion_centre(explosion_centre), K(K), radius_sqr(radius * radius), fade_time_constant(fade_time_constant) { }

void generators::explosion_generator::apply_to_particles(
    systems::particle_system &particle_system,
    objects::seconds_f64 delta_time
) {
    particle_system.iter_indexed<
        objects::particle::deconstruct_position const,
        objects::generators::particle_force
    >(
        [this, &particle_system , delta_time](systems::particle_id particle_id,
            objects::particle::deconstruct_position const &position,
            objects::generators::particle_force &force) {

            generators::explosion_generator::explosion_seconds_f64 *explosion_time = nullptr;
            if (particle_system.particles.particle_has_attribute
                <generators::explosion_generator::explosion_seconds_f64>(particle_id)) {
                explosion_time = particle_system.particles.get_particle_attribute_ptr
                    <generators::explosion_generator::explosion_seconds_f64>(particle_id);
            } else {
                explosion_time = &particle_system.particles.set_particle_attribute(
                    particle_id,
                    generators::explosion_generator::explosion_seconds_f64{0.0f}
                );
            }

            if ((types::v3_f32{position} - this->explosion_centre).magnitude_sqr() > this->radius_sqr) {
                explosion_time->value = 0.0f;
            } else {
                explosion_time->value += delta_time;
            }
        }
    );

    particle_system.iter<
        generators::explosion_generator::explosion_seconds_f64 const,
        objects::particle::deconstruct_position const,
        objects::generators::particle_force
    >(
        [this](generators::explosion_generator::explosion_seconds_f64 const &explosion_time,
            objects::particle::deconstruct_position const &position,
            objects::generators::particle_force &force) {
            // assert(
            //     !std::isnan(force.x)
            //     && !std::isnan(force.y)
            //     && !std::isnan(force.z)
            //     && "error: force is NaN"
            // );

            types::v3_f32 local_position = types::v3_f32{position} - this->explosion_centre;
            types::f32 distance_sqr = local_position.magnitude_sqr();
            if (distance_sqr > this->radius_sqr)
                return;

            if (distance_sqr > 0.001f) {
                types::v3_f32 explosion_force =
                    (K / distance_sqr)
                    * local_position.normalized()
                    * std::expf(-explosion_time.value / this->fade_time_constant);
                force += explosion_force;
            }
            // assert(
            //     !std::isnan(force.x)
            //     && !std::isnan(force.y)
            //     && !std::isnan(force.z)
            //     && "error: force is NaN"
            // );
        }
    );
}
