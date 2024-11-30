#ifndef PARTICLE_SYSTEM_HPP
#define PARTICLE_SYSTEM_HPP

#include <queue>
#include <ctime>
#include <tuple>
#include <functional>

#include "particle_storage.hpp"
#include "particle_generator.hpp"

namespace systems {
    struct particle_meta {
        bool is_alive : 1;
        bool is_prefab : 1;
    };
    
    struct particle_system {
    private:
        std::time_t start_time;
        size_t lifetime_milliseconds;
    public:
        template <typename Tag = void>
        struct tag_particle_id {
            particle_id id;
        };
        
    public:
        particle_storage particles;
        particle_generator generator;
        std::queue<particle_id> dead_particles;

    public:
        particle_system(particle_generator generator, size_t lifetime_milliseconds);

    public:
        particle_id add_particle();

        template <
			typename T,
			typename Tuple = T::particle_deconstruct
		>
        particle_id add_particle(T const &attributes) {
            particle_id particle = add_particle();
            particles.set_particle_attributes_deconstruct(particle, attributes);
            return particle;
        }

        template <
            typename T,
            typename Position = types::v3_f32,
            typename Outwards = types::v3_f32,
            typename F = std::function<T(Position, Outwards)>,
            typename Tuple = T::particle_deconstruct
        >
        particle_id add_particle_random(
            F const &particle_contruction,
            types::f32 begin_normalized = 0.0f,
            types::f32 end_normalized = 1.0f,
            types::f32 standard_deviation = 1.0f
        ) {
            types::v3_f32 out_outwards_direction;
            types::v3_f32 position = generator.generate_position_outwards(
                out_outwards_direction,
                begin_normalized,
                end_normalized,
                standard_deviation
            );
            return add_particle<T>(particle_contruction(position, out_outwards_direction));
        }

        particle_id remove_particle(particle_id particle);

    public:
        template <typename ...Attributes>
        std::tuple<Attributes& ...> set(particle_id particle, Attributes &&... attributes) {
            return {
                particles.set_particle_attribute<Attributes>(particle, std::move(attributes))...
            };
        }

        template <typename ...Attributes>
        std::tuple<Attributes& ...> get(particle_id particle) {
            return {
                particles.get_particle_attribute<Attributes>(particle)...
            };
        }

        template <typename ...Attributes>
        std::tuple<Attributes ...> remove(particle_id particle) {
            return {
                [&]() {
                    Attributes attribute;
                    particles.remove_particle_attribute<Attributes>(particle, attribute);
                    return attribute;
                }()...
            };
        }

    public:
        inline bool active(std::time_t current_time) const {
            return current_time - start_time <= (std::time_t)lifetime_milliseconds;
        }
        inline particle_count_t alive_particle_count() const {
            return particles.particle_count() - dead_particles.size();
        }

    public:
        template <
			typename ...Ts,
			typename F = std::function<void(Ts& ...)>
		>	
		inline particle_count_t iter(F const &func) {
            return particles.iter<Ts...>(func);
        }

        template <
			typename ...Ts,
			typename F = std::function<void(particle_id, Ts& ...)>
		>	
		inline particle_count_t iter_indexed(F const &func) {
            return particles.iter_indexed<Ts...>(func);
        }
    };
}

#endif