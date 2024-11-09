#ifndef EXPLOSION_GENERATOR_HPP
#define EXPLOSION_GENERATOR_HPP

#include "../systems/particle_system.hpp"
#include "../types/v3_f32.hpp"
#include "../objects/mass_particle.hpp"

namespace generators {
    struct explosion_generator {
        types::v3_f32 explosion_centre;
        types::f32 K;
        types::f32 radius_sqr;
        types::f32 fade_time_constant;

        explosion_generator(
            types::v3_f32 explosion_centre,
            types::f32 K,
            types::f32 radius,
            types::f32 fade_time_constant
        );

        void apply_to_particles(systems::particle_system &particle_system, objects::seconds_f64 delta_time);

    private:
        template <typename Tag = void>
		struct f64_tag {
			types::f64 value;
		};

    public:
        using explosion_seconds_f64 = f64_tag<struct explosion_seconds>;
    };
}


#endif