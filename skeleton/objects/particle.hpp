#include <PxPhysicsAPI.h>
#include "../types/v3_f32.hpp"
#include "../RenderUtils.hpp"

namespace objects {
	using namespace types;

	typedef v3_f32 position3_f32;
	typedef v3_f32 velocity3_f32;
	typedef v3_f32 acceleration3_f32;

	typedef double seconds_f64;

	struct particle {
	public:
		position3_f32 position;
		velocity3_f32 velocity;

	public:
		particle() noexcept;
		particle(v3_f32 position, v3_f32 velocity) noexcept;
		particle(particle const &other) noexcept;
		particle(particle&& other) noexcept;
		~particle();

	public:
		position3_f32 step(seconds_f64 delta_time) noexcept;
		velocity3_f32 add_acceleration(acceleration3_f32 acceleration, seconds_f64 delta_time) noexcept;
		velocity3_f32 damp(f32 damping_factor, seconds_f64 delta_time) noexcept;

	public:
		position3_f32 integrate_semi_implicit_euler(
			acceleration3_f32 acceleration,
			f32 damping_factor,
			seconds_f64 delta_time
		) noexcept;
		position3_f32 integrate_velocity_verlet(
			acceleration3_f32 acceleration,
			f32 damping_factor,
			seconds_f64 delta_time
		) noexcept;

	public:
		particle &operator=(particle const& other) noexcept {
			this->position = other.position;
			this->velocity = other.velocity;
			return *this;
		}

		particle& operator=(particle&& other) noexcept {
			this->position = std::move(other.position);
			this->velocity = std::move(other.velocity);
			return *this;
		}

		particle const& operator>>(physx::PxTransform& transform) {
			transform.p = this->position;
			return *this;
		}
	};
}