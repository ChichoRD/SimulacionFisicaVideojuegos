#ifndef SPRING_FORCE_GENERATOR_HPP
#define SPRING_FORCE_GENERATOR_HPP

#include <array>
#include "../../systems/particle_system.hpp"
#include "../../types/v3_f32.hpp"
#include "../../objects/mass_particle.hpp"

namespace generators {
    struct spring_anchor {
        objects::position3_f32 anchor;
        types::f32 rest_distance;
    };

    template <size_t MaxAnchors>
    struct static_spring_anchors {
        std::array<spring_anchor, MaxAnchors> anchors;
        size_t anchor_count;
    };

    struct static_spring_force_generator {
        types::f32 spring_constant;
        constexpr static size_t MaxAnchors = 8; 

        static_spring_force_generator(types::f32 spring_constant);

        template <size_t MaxAnchors>
        size_t add_anchor(
            systems::particle_system &particle_system,
            systems::particle_id particle_index,
            objects::position3_f32 const &anchor
        ) {
            types::f32 rest_distance = (
                anchor
                - objects::position3_f32{
                    particle_system.particles.get_particle_attribute<objects::particle::deconstruct_position>(particle_index)
                }
            ).magnitude();
            if (particle_system.particles.particle_has_attribute<static_spring_anchors<MaxAnchors>>(particle_index)) {
                auto &anchors = particle_system.particles.get_particle_attribute<static_spring_anchors<MaxAnchors>>(particle_index);
                if (anchors.anchor_count < anchors.anchors.size()) {
                    anchors.anchors[anchors.anchor_count] = {anchor, rest_distance};
                    anchors.anchor_count++;
                    return anchors.anchor_count - 1;
                }
            } else {
                auto &anchors = particle_system.particles.set_particle_attribute<static_spring_anchors<MaxAnchors>>(
                    particle_index,
                    static_spring_anchors<MaxAnchors>{}
                );
                anchors.anchor_count = 1;
                anchors.anchors[0] = {anchor, rest_distance};
                return 0;
            }
        }

        template <size_t MaxAnchors>
        size_t remove_anchor(
            systems::particle_system &particle_system,
            systems::particle_id particle_index,
            objects::position3_f32 const &anchor
        ) {
            if (particle_system.particles.particle_has_attribute<static_spring_anchors<MaxAnchors>>(particle_index)) {
                auto &anchors = particle_system.particles.get_attribute<static_spring_anchors<MaxAnchors>>(particle_index);
                for (size_t i = 0; i < anchors.anchor_count; i++) {
                    if (anchors.anchors[i] == anchor) {
                        anchors.anchors[i] = anchors.anchors[anchors.anchor_count - 1];
                        anchors.anchor_count--;
                        return i;
                    }
                }
            }
            return -1;
        }
        void apply_to_particles(systems::particle_system &particle_system, objects::seconds_f64 delta_time);

        static types::v3_f32 spring_force(
            types::v3_f32 const &particle_position,
            types::f32 spring_constant,
            spring_anchor const &anchor
        );
    };

    struct dynamic_spring_anchor {
        systems::particle_id anchor;
        types::f32 rest_distance;
    };
    template <size_t MaxAnchors>
    struct dynamic_spring_anchors {
        std::array<dynamic_spring_anchor, MaxAnchors> particle_id_anchors;
        size_t anchor_count;
    };

    struct dynamic_spring_force_generator {
        types::f32 spring_constant;
        constexpr static size_t MaxAnchors = 8;

        dynamic_spring_force_generator(types::f32 spring_constant);

        template <size_t MaxAnchors>
        size_t add_anchor(
            systems::particle_system &particle_system,
            systems::particle_id particle_index,
            systems::particle_id anchor
        ) {
            types::f32 rest_distance = (
                particle_system.particles.get_particle_attribute<objects::particle::deconstruct_position>(anchor)
                - particle_system.particles.get_particle_attribute<objects::particle::deconstruct_position>(particle_index)
            ).magnitude();
            if (particle_system.particles.particle_has_attribute<dynamic_spring_anchors<MaxAnchors>>(particle_index)) {
                auto &anchors = particle_system.particles.get_particle_attribute<dynamic_spring_anchors<MaxAnchors>>(particle_index);
                if (anchors.anchor_count < anchors.particle_id_anchors.size()) {
                    anchors.particle_id_anchors[anchors.anchor_count] = {anchor, rest_distance};
                    anchors.anchor_count++;
                    return anchors.anchor_count - 1;
                }
            } else {
                auto &anchors = particle_system.particles.set_particle_attribute<dynamic_spring_anchors<MaxAnchors>>(
                    particle_index,
                    dynamic_spring_anchors<MaxAnchors>{}
                );
                anchors.anchor_count = 1;
                anchors.particle_id_anchors[0] = {anchor, rest_distance};
                return 0;
            }
        }

        template <size_t MaxAnchors>
        size_t remove_anchor(
            systems::particle_system &particle_system,
            systems::particle_id particle_index,
            systems::particle_id anchor
        ) {
            if (particle_system.particles.particle_has_attribute<dynamic_spring_anchors<MaxAnchors>>(particle_index)) {
                auto &anchors = particle_system.particles.get_attribute<dynamic_spring_anchors<MaxAnchors>>(particle_index);
                for (size_t i = 0; i < anchors.anchor_count; i++) {
                    if (anchors.particle_id_anchors[i] == anchor) {
                        anchors.particle_id_anchors[i] = anchors.particle_id_anchors[anchors.anchor_count - 1];
                        anchors.anchor_count--;
                        return i;
                    }
                }
            }
            return -1;
        }
        void apply_to_particles(systems::particle_system &particle_system, objects::seconds_f64 delta_time);
    };

}

#endif