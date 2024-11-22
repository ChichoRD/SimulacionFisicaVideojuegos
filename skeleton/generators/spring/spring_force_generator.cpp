#include "spring_force_generator.hpp"

generators::static_spring_force_generator::static_spring_force_generator(types::f32 spring_constant)
    : spring_constant(spring_constant) { }

void generators::static_spring_force_generator::apply_to_particles(
    systems::particle_system &particle_system,
    objects::seconds_f64 delta_time
) {
    types::f32 spring_constant = this->spring_constant;
    particle_system.iter_indexed<
        static_spring_anchors<MaxAnchors> const,
        objects::particle::deconstruct_position const,
        objects::generators::particle_force
    >(
        [spring_constant, particle_system](systems::particle_id particle_index,
            static_spring_anchors<MaxAnchors> const &anchors,
            objects::particle::deconstruct_position const &position,
            objects::generators::particle_force &force) {
            for (size_t i = 0; i < anchors.anchor_count; i++) {
                force += spring_force(position, spring_constant, anchors.anchors[i]);
            }
        }
    );
}

types::v3_f32 generators::static_spring_force_generator::spring_force(
    types::v3_f32 const &particle_position,
    types::f32 spring_constant,
    spring_anchor const &anchor
) {
    types::v3_f32 local_position = particle_position - anchor.anchor;
    types::f32 distance = local_position.magnitude();
    types::f32 displacement = distance - anchor.rest_distance;
    return -spring_constant * displacement * local_position.normalized();
}

generators::dynamic_spring_force_generator::dynamic_spring_force_generator(types::f32 spring_constant)
    : spring_constant(spring_constant) { }

void generators::dynamic_spring_force_generator::apply_to_particles(
    systems::particle_system &particle_system,
    objects::seconds_f64 delta_time
) {
    types::f32 spring_constant = this->spring_constant;
    particle_system.iter_indexed<
        dynamic_spring_anchors<MaxAnchors> const,
        objects::particle::deconstruct_position const,
        objects::generators::particle_force
    >(
        [spring_constant, particle_system](systems::particle_id particle_index,
            dynamic_spring_anchors<MaxAnchors> const &anchors,
            objects::particle::deconstruct_position const &position,
            objects::generators::particle_force &force) {
            for (size_t i = 0; i < anchors.anchor_count; i++) {
                dynamic_spring_anchor const &anchor = anchors.particle_id_anchors[i];
                auto other_position =
                    particle_system.particles.get_particle_attribute<objects::particle::deconstruct_position>(anchor.anchor);
                force += static_spring_force_generator::spring_force(position, spring_constant, {
                    types::v3_f32{other_position},
                    anchor.rest_distance
                });
            }
        }
    );
}
