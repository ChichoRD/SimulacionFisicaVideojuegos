#include "particle.hpp"

namespace objects {
	particle::particle(v3_f32 position, v3_f32 velocity)
		: _position(position), _velocity(velocity) { }

	particle::particle(particle const& other)
		: _position(other._position), _velocity(other._velocity) { }

	particle::particle(particle&& other) noexcept
		: _position(std::move(other._position)), _velocity(std::move(other._velocity)) { }

	particle::~particle() = default;

	position3_f32 particle::integrate(seconds_f64 delta_time) {
		this->_position += this->_velocity * delta_time;
	}
}

