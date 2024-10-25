#ifndef PARTICLE_HPP
#define PARTICLE_HPP

#include <PxPhysicsAPI.h>
#include "../types/v3_f32.hpp"
#include "../types/particle_defs.hpp"
#include "../RenderUtils.hpp"

namespace objects {
	using namespace types;

	typedef v3_f32 position3_f32;
	typedef v3_f32 velocity3_f32;
	typedef v3_f32 acceleration3_f32;

	typedef double seconds_f64;

	struct particle {
	public:
		using deconstruct_previous_position = v3<f32, struct previous_position>;
		using deconstruct_position = v3<f32, struct position>;
		using deconstruct_velocity = v3<f32, struct velocity>;

	public:
		position3_f32 previous_position;
		position3_f32 position;
		velocity3_f32 velocity;

	public:
		particle() noexcept;
		particle(position3_f32 position, velocity3_f32 velocity) noexcept;
		particle(
			deconstruct_previous_position previous_position,
			deconstruct_position position,
			deconstruct_velocity velocity
		) noexcept;
		particle(particle const &other) noexcept;
		~particle();

	public:
		position3_f32 integrate_semi_implicit_euler(
			acceleration3_f32 acceleration,
			f32 damping_factor,
			seconds_f64 delta_time
		) noexcept;
		position3_f32 integrate_euler(
			acceleration3_f32 acceleration,
			f32 damping_factor,
			seconds_f64 delta_time
		) noexcept;
		position3_f32 integrate_midpoint(
			acceleration3_f32 acceleration,
			f32 damping_factor,
			seconds_f64 delta_time
		) noexcept;
		position3_f32 integrate_verlet(
			acceleration3_f32 acceleration,
			f32 damping_factor,
			seconds_f64 delta_time
		) noexcept;

	public:
		particle &operator=(particle const& other) noexcept {
			this->previous_position = other.previous_position;
			this->position = other.position;
			this->velocity = other.velocity;
			return *this;
		}

		particle const& operator>>(physx::PxTransform& transform) {
			transform.p = this->position;
			return *this;
		}

	public:
		using particle_deconstruct = systems::particle_trait::particle_deconstruct
			<deconstruct_previous_position, deconstruct_position, deconstruct_velocity>;
		particle_deconstruct deconstruct() const;
	};
}

#endif // !PARTICLE_HPP