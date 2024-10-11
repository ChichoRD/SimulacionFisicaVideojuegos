#include "mass_particle.hpp"

namespace objects {
	struct projectile
	{
	public:
		mass_particle particle;
		f32 gravity_scale;

	public:
		projectile() noexcept;
		projectile(mass_particle particle, f32 speed_scale_factor) noexcept;
		projectile(mass_f32 mass, position3_f32 position, velocity3_f32 velocity, f32 speed_scale_factor) noexcept;
		projectile(projectile const& other) noexcept;

	public:
		position3_f32 integrate(
			acceleration3_f32 gravity,
			acceleration3_f32 acceleration,
			f32 damping_factor,
			seconds_f64 delta_time
		) noexcept;

	public:
		projectile& operator=(projectile const& other) noexcept = default;
	};
}