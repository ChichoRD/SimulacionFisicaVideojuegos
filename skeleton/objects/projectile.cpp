#include "mass_particle.hpp"
#include "projectile.hpp"

namespace objects {
	projectile::projectile() noexcept
		: particle(), gravity_scale(1) { }
	projectile::projectile(mass_particle particle, f32 speed_scale_factor) noexcept
		: particle(particle), gravity_scale() {
		particle = mass_particle::scale_preserve_cinetic_energy(
			particle,
			speed_scale_factor,
			gravity_scale
		);
	}
	projectile::projectile(mass_f32 mass, position3_f32 position, velocity3_f32 velocity, f32 speed_scale_factor) noexcept
		: particle(objects::particle(position, velocity), mass), gravity_scale() {
		particle = mass_particle::scale_preserve_cinetic_energy(
			particle,
			speed_scale_factor,
			gravity_scale
		);
	}
	projectile::projectile(projectile const& other) noexcept = default;

	position3_f32 projectile::integrate(
		acceleration3_f32 gravity,
		acceleration3_f32 acceleration,
		f32 damping_factor,
		seconds_f64 delta_time
	) noexcept {
		return particle.integrate(
			gravity * gravity_scale + acceleration,
			damping_factor,
			delta_time,
			integrator_model::MIDPOINT
		);
	}
}