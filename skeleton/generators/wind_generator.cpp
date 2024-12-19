#include "wind_generator.hpp"
#include "../objects/solid_particle.hpp"

generators::wind_generator::wind_generator(
    types::v3_f32 area_centre,
    types::f32 area_radius,
    types::v3_f32 wind_velocity,
    types::f32 k1,
    types::f32 k2
) : area_centre(area_centre), area_radius(area_radius), wind_velocity(wind_velocity), k1(k1), k2(k2) { }

static bool wind_force_applies(objects::position3_f32 position, types::v3_f32 area_centre, types::f32 area_radius) {
    return (area_centre - types::v3_f32{position}).magnitude_sqr() <= area_radius * area_radius;
}

static types::v3_f32 wind_force(
    types::v3_f32 wind_velocity,
    types::v3_f32 velocity,
    types::f32 k1,
    types::f32 k2
) {
    // F = k1 * (v_w - v) + k2 * ||v_w - v|| * (v_w - v)
    types::v3_f32 relative_velocity = wind_velocity - velocity;
    return k1 * relative_velocity + k2 * relative_velocity.magnitude() * relative_velocity;
}

void generators::wind_generator::apply_to_particles(
    systems::particle_system &particle_system,
    objects::seconds_f64 delta_time
) {
    particle_system.iter<
        objects::particle::deconstruct_velocity const,
        objects::particle::deconstruct_position const,
        objects::generators::particle_force
    >(
        [this, delta_time](objects::particle::deconstruct_velocity const &velocity,
            objects::particle::deconstruct_position const &position,
            objects::generators::particle_force &force) {
            if (!wind_force_applies(position, this->area_centre, this->area_radius))
                return;
            
            force += wind_force(this->wind_velocity, types::v3_f32{velocity}, this->k1, this->k2);
        }
    );

    particle_system.iter<
        objects::solid_dynamic_particle::deconstruct_rigid_dynamic const,
        objects::generators::particle_torque
    >(
        [this, delta_time](objects::solid_dynamic_particle::deconstruct_rigid_dynamic const &rigid_dynamic,
            objects::generators::particle_torque &torque) {
            types::v3_f32 velocity = types::v3_f32{rigid_dynamic->getLinearVelocity()};
            if (!wind_force_applies(rigid_dynamic->getGlobalPose().p, this->area_centre, this->area_radius))
                return;

            torque += wind_force(this->wind_velocity, velocity, this->k1, this->k2);
        }
    );
}

generators::tornado_generator::tornado_generator(
    types::v3_f32 column_base,
    types::v3_f32 column_direction,
    types::f32 height,
    types::f32 radius,
    types::f32 K
) : column_base(column_base), column_direction(column_direction), height(height), radius(radius), K(K) { }

static bool tornado_force_applies(
    objects::position3_f32 position,
    types::v3_f32 column_base,
    types::v3_f32 column_direction,
    types::f32 height,
    types::f32 radius,
    types::v3_f32 &out_outwards_displacement,
    types::f32 &out_distance_sqr
) {
    types::v3_f32 local_position = types::v3_f32{position} - column_base;
    out_outwards_displacement = local_position - types::v3_f32::project(local_position, column_direction);

    out_distance_sqr = out_outwards_displacement.magnitude_sqr();
    types::f32 altitude = types::v3_f32::dot(local_position, column_direction);
    return out_distance_sqr <= radius * radius && altitude <= height;
}

static types::v3_f32 tornado_force(
    types::v3_f32 outwards_displacement,
    types::v3_f32 column_direction,
    types::f32 K,
    types::f32 distance_sqr,
    types::f32 radius
) {
    // F = K * (||r||^2 / R^2) * (r x d)
    types::v3_f32 tangent_force = 
        types::v3_f32::cross(outwards_displacement, column_direction)
        * (K * (distance_sqr / (radius * radius)));
    return tangent_force;
}

void generators::tornado_generator::apply_to_particles(
    systems::particle_system &particle_system,
    objects::seconds_f64 delta_time
) {
    particle_system.iter<
        objects::particle::deconstruct_position const,
        objects::generators::particle_force
    >(
        [this](objects::particle::deconstruct_position const &position,
            objects::generators::particle_force &force) {
            types::v3_f32 outwards_displacement;
            types::f32 distance_sqr;
            if (!tornado_force_applies(
                position, this->column_base, this->column_direction, this->height, this->radius,
                outwards_displacement, distance_sqr
            )) {
                return;
            }

            force += tornado_force(
                outwards_displacement, this->column_direction, this->K, distance_sqr, this->radius
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
            types::v3_f32 outwards_displacement;
            types::f32 distance_sqr;
            if (!tornado_force_applies(
                position, this->column_base, this->column_direction, this->height, this->radius,
                outwards_displacement, distance_sqr
            )) {
                return;
            }

            force += tornado_force(
                outwards_displacement, this->column_direction, this->K, distance_sqr, this->radius
            );
        }
    );
}
