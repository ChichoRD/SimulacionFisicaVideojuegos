#include <cmath>
#include "particle.hpp"

namespace objects {
	particle::particle() noexcept
		: position(), velocity() { }

	particle::particle(position3_f32 position, velocity3_f32 velocity) noexcept
		: position(position), velocity(velocity) { }

	particle::particle(particle const& other) noexcept
		: position(other.position), velocity(other.velocity) { }

	particle::particle(particle&& other) noexcept
		: position(std::move(other.position)), velocity(std::move(other.velocity)) { }

	particle::~particle() = default;

	position3_f32 particle::step(seconds_f64 delta_time) noexcept {
		return this->position += this->velocity * delta_time;
	}

	velocity3_f32 particle::add_acceleration(acceleration3_f32 acceleration, seconds_f64 delta_time) noexcept {
		// m * dv = F * dt
		// m * dv = (m * a) * dt
		// dv = a * dt
		
		return this->velocity += acceleration * delta_time;
	}

	velocity3_f32 particle::damp(f32 damping_factor, seconds_f64 delta_time) noexcept {
		return this->velocity *= std::powf(damping_factor, delta_time);
	}


	position3_f32 particle::integrate_semi_implicit_euler(
		acceleration3_f32 acceleration,
		f32 damping_factor,
		seconds_f64 delta_time
	) noexcept {
		this->damp(damping_factor, delta_time);
		this->add_acceleration(acceleration, delta_time);
		return this->step(delta_time);
	}

	position3_f32 particle::integrate_velocity_verlet(
		acceleration3_f32 acceleration,
		f32 damping_factor,
		seconds_f64 delta_time
	) noexcept {
		// TODO
		return this->step(delta_time);
	}

}

