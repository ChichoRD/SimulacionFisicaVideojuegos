#include "particle_system.hpp"
namespace systems {
	systems::particle_system::particle_system() noexcept
		: particles(), attribute_map() { }
	particle_system::particle_system(size_t attribute_capacity) noexcept
		: particles(), attribute_map(attribute_capacity) {
		particles.reserve(attribute_capacity);
	}
}