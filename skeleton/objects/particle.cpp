#include <cmath>
#include "particle.hpp"

namespace objects {
	particle::particle() noexcept
		: previous_position(), position(), velocity() { }

	particle::particle(position3_f32 position, velocity3_f32 velocity) noexcept
		: previous_position(position), position(position), velocity(velocity) { }

	particle::particle(
		deconstruct_previous_position previous_position,
		deconstruct_position position,
		deconstruct_velocity velocity
	) noexcept
		: previous_position(previous_position), position(position), velocity(velocity) { }

	particle::particle(particle const& other) noexcept
		: previous_position(other.position), position(other.position), velocity(other.velocity) { }

	particle::~particle() = default;

	position3_f32 particle::integrate_semi_implicit_euler(
		acceleration3_f32 acceleration,
		f32 damping_factor,
		seconds_f64 delta_time
	) noexcept {
		velocity *= std::powf(damping_factor, delta_time);
		velocity += acceleration * delta_time;

		previous_position = position;
		return position += velocity * delta_time;
	}

	position3_f32 particle::integrate_euler(
		acceleration3_f32 acceleration,
		f32 damping_factor,
		seconds_f64 delta_time
	) noexcept {
		previous_position = position;
		position += velocity * delta_time;

		velocity *= std::powf(damping_factor, delta_time);
		velocity += acceleration * delta_time;
		return position;
	}

	position3_f32 particle::integrate_midpoint(
		acceleration3_f32 acceleration,
		f32 damping_factor,
		seconds_f64 delta_time
	) noexcept {
		velocity *= std::powf(damping_factor, delta_time * 0.5f);
		velocity += acceleration * delta_time * 0.5f;

		previous_position = position;
		position += velocity * delta_time;

		velocity *= std::powf(damping_factor, delta_time * 0.5f);
		velocity += acceleration * delta_time * 0.5f;

		return position;
	}

	position3_f32 particle::integrate_verlet(
		acceleration3_f32 acceleration,
		f32 damping_factor,
		seconds_f64 delta_time
	) noexcept {
		position3_f32 previous_position = position;
		position += position + acceleration * delta_time * delta_time - this->previous_position;

		this->previous_position = previous_position;
		velocity = (position - previous_position) / delta_time;

		return position;
	}

	particle::particle_deconstruct particle::deconstruct() const {
		return std::make_tuple(
			v3<f32, struct previous_position>(previous_position),
			v3<f32, struct position>(position),
			v3<f32, struct velocity>(velocity)
		);
	}
}

