#include "particle_system.hpp"
namespace systems {
	systems::particle_system::particle_system() noexcept
		: particle_data(), particle_data_mask(), attribute_map() { }
	particle_system::particle_system(size_t particle_capacity, size_t attribute_capacity) noexcept
		: particle_data(), particle_data_mask(), attribute_map(attribute_capacity) {
		particle_data.reserve(attribute_capacity);
		particle_data_mask.reserve(particle_capacity);
	}
}