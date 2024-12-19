#include <cassert>
#include "explosion_generator.hpp"
#include "../objects/solid_particle.hpp"

generators::explosion_generator::explosion_generator(
    types::v3_f32 explosion_centre,
    types::f32 K,
    types::f32 radius,
    types::f32 fade_time_constant
) : explosion_centre(explosion_centre), K(K), radius_sqr(radius * radius), fade_time_constant(fade_time_constant) { }

static generators::explosion_generator::explosion_seconds_f64 *apply_or_register_explosion_time(
    objects::position3_f32 explosion_centre,
    types::f32 radius_sqr,
    systems::particle_system &particle_system,
    objects::seconds_f64 delta_time,
    systems::particle_id particle_id,
    objects::position3_f32 const &position
) {
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

    if ((types::v3_f32{position} - explosion_centre).magnitude_sqr() > radius_sqr) {
        explosion_time->value = 0.0f;
    } else {
        explosion_time->value += delta_time;
    }
    return explosion_time;
}

static bool explosion_force_applies(
    objects::position3_f32 position,
    types::v3_f32 explosion_centre,
    types::f32 radius_sqr,
    objects::position3_f32 &out_local_position,
    types::f32 &out_distance_sqr
) {
    out_local_position = types::v3_f32{position} - explosion_centre;
    out_distance_sqr = out_local_position.magnitude_sqr();
    return out_distance_sqr <= radius_sqr;
}

static types::v3_f32 explosion_force(
    types::v3_f32 local_position,
    types::f32 distance_sqr,
    types::f32 K,
    types::f32 fade_time_constant,
    generators::explosion_generator::explosion_seconds_f64 explosion_time
) {
    if (distance_sqr > 0.001f) {
        types::v3_f32 explosion_force =
            (K / distance_sqr)
            * local_position.normalized()
            * std::expf(-explosion_time.value / fade_time_constant);
        return explosion_force;
    } else {
        return types::v3_f32{0.0f, 0.0f, 0.0f};
    }
}

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
            
            generators::explosion_generator::explosion_seconds_f64 *explosion_time = apply_or_register_explosion_time(
                this->explosion_centre, this->radius_sqr, particle_system, delta_time, particle_id, position
            );
        }
    );

    particle_system.iter_indexed<
        objects::solid_dynamic_particle::deconstruct_rigid_dynamic const,
        objects::generators::particle_force
    >(
        [this, &particle_system, delta_time](systems::particle_id particle_id,
            objects::solid_dynamic_particle::deconstruct_rigid_dynamic const &rigid_dynamic,
            objects::generators::particle_force &force) {
            objects::position3_f32 position = rigid_dynamic->getGlobalPose().p;
            generators::explosion_generator::explosion_seconds_f64 *explosion_time = apply_or_register_explosion_time(
                this->explosion_centre, this->radius_sqr, particle_system, delta_time, particle_id, position
            );
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
            
            objects::position3_f32 local_position;
            types::f32 distance_sqr;
            if (!explosion_force_applies(position, this->explosion_centre, this->radius_sqr, local_position, distance_sqr)) {
                return;
            }

            force += explosion_force(
                types::v3_f32{local_position}, distance_sqr, this->K, this->fade_time_constant, explosion_time
            );
        }
    );

    particle_system.iter<
        generators::explosion_generator::explosion_seconds_f64 const,
        objects::solid_dynamic_particle::deconstruct_rigid_dynamic const,
        objects::generators::particle_force
    >(
        [this](generators::explosion_generator::explosion_seconds_f64 const &explosion_time,
            objects::solid_dynamic_particle::deconstruct_rigid_dynamic const &rigid_dynamic,
            objects::generators::particle_force &force) {
            objects::position3_f32 position = rigid_dynamic->getGlobalPose().p;
            objects::position3_f32 local_position;
            types::f32 distance_sqr;
            if (!explosion_force_applies(position, this->explosion_centre, this->radius_sqr, local_position, distance_sqr)) {
                return;
            }

            force += explosion_force(
                types::v3_f32{local_position}, distance_sqr, this->K, this->fade_time_constant, explosion_time
            );
        }
    );
}
