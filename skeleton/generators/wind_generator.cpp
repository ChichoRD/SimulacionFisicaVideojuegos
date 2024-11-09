#include "wind_generator.hpp"

generators::wind_generator::wind_generator(
    types::v3_f32 area_centre,
    types::f32 area_radius,
    types::v3_f32 wind_velocity,
    types::f32 k1,
    types::f32 k2
) : area_centre(area_centre), area_radius(area_radius), wind_velocity(wind_velocity), k1(k1), k2(k2) { }

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
            if ((this->area_centre - types::v3_f32{position}).magnitude_sqr() > this->area_radius * this->area_radius)
                return;
            
            // F = k1 * (v_w - v) + k2 * ||v_w - v|| * (v_w - v)
            types::v3_f32 relative_velocity = this->wind_velocity - types::v3_f32{velocity};
            types::v3_f32 wind_force =
                this->k1 * relative_velocity + this->k2 * relative_velocity.magnitude() * relative_velocity;

            force += wind_force;
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
            types::v3_f32 local_position = types::v3_f32{position} - this->column_base;
            types::v3_f32 outwards_displacement = local_position - types::v3_f32::project(local_position, this->column_direction);

            types::f32 distance_sqr = outwards_displacement.magnitude_sqr();
            types::f32 altitude = types::v3_f32::dot(local_position, this->column_direction);
            if (distance_sqr > this->radius * this->radius || altitude > this->height)
                return;

            types::v3_f32 tangent_force = 
                types::v3_f32::cross(outwards_displacement, this->column_direction)
                * (K * (distance_sqr / (this->radius * this->radius)));
            force += tangent_force;
        }
    );
}
