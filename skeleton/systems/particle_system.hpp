#include <queue>
#include <ctime>
#include <tuple>

#include "particle_storage.hpp"

namespace systems {
    struct particle_meta {
        bool is_alive : 1;
        bool is_prefab : 1;
    };
    
    struct particle_system {
    private:
        std::time_t start_time;
        std::size_t lifetime_milliseconds;
    public:
        particle_storage particles;
        std::queue<particle_id> dead_particles;

    public:
        particle_system()
            : particles(), dead_particles() {}

    public:
        particle_id add_particle();

        template <
			typename T,
			typename ...Attributes,
			typename Tuple = T::particle_deconstruct
		>
        particle_id add_particle(T const &attributes) {
            particle_is particle = add_particle();
            particles.set_particle_attributes_deconstruct(particle, attributes);
            return particle;
        }
        particle_id remove_particle(particle_id particle);

    public:
        template <typename ...Attributes>
        std::tuple<Attributes& ...> set(particle_id particle, Attributes const &... attributes) {
            return {
                particles.set_particle_attribute<Attributes>(particle, attributes)...
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
            return current_time - start_time <= lifetime_milliseconds;
        }

    public:
        template <
			typename ...Ts,
			typename F = std::function<void(Ts& ...)>
		>	
		inline particle_count_t iter(F const &func) {
            return particles.iter<Ts...>(func);
        }
    };
}