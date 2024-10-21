#include <cassert>
#include <stdlib.h>
#include "mass_particle.hpp"

namespace objects {
	mass_particle::mass_particle() noexcept
		: particle(), inverse_mass(1.0f) { }
    mass_particle::mass_particle(
		v3<f32, struct previous_position> previous_position,
		v3<f32, struct position> position,
		v3<f32, struct velocity> velocity,
		f32_tag<struct inverse_mass> inverse_mass
	) noexcept
		: particle(previous_position, position, velocity), inverse_mass(inverse_mass.value) { }
    mass_particle::mass_particle(objects::particle particle, mass_f32 mass) noexcept
        : particle(particle), inverse_mass(1.0f / mass) {}
    mass_particle::mass_particle(mass_particle const& other) noexcept
		: particle(other.particle), inverse_mass(other.inverse_mass) { }

	void mass_particle::add_impulse(impulse3_f32 impulse) noexcept {
		// m * dv = F * dt
		// m * dv = I
		// dv = I / m

		particle.velocity += impulse * inverse_mass;
	}
	void mass_particle::add_velocity(velocity3_f32 velocity) noexcept {
		particle.velocity += velocity;
	}

	position3_f32 mass_particle::integrate(
		acceleration3_f32 acceleration,
		f32 damping_factor,
		seconds_f64 delta_time,
		integrator_model integrator
	) noexcept {
		switch (integrator)
		{
		case objects::EULER:
			return particle.integrate_euler(acceleration, damping_factor, delta_time);
		case objects::SEMI_IMPLICIT_EULER:
			return particle.integrate_semi_implicit_euler(acceleration, damping_factor, delta_time);
		case objects::MIDPOINT:
			return particle.integrate_midpoint(acceleration, damping_factor, delta_time);
		case objects::VERLET:
			return particle.integrate_verlet(acceleration, damping_factor, delta_time);
		default: {
			assert(false && "unreachable: unrecognised integrator");
			std::exit(EXIT_FAILURE);
			return position3_f32();
		}
		}
	}

	mass_particle mass_particle::scale_preserve_cinetic_energy(
		mass_particle const& particle,
		f32 speed_scale_factor,
		f32& out_gravity_scale_factor
	) noexcept {
		// E_c = 0.5f * m * v^2
		// E_c0 = E_c1

		// 0.5f * m0 * v0^2 = 0.5f * m1 * (v0 * factor)^2
		// m1 = (m0 * v0^2) / (v0^2 * factor^2)
		// m1 = m0 / (factor^2)

		// m1^-1 = (factor^2) * (m0^-1)
		mass_particle scaled_particle = mass_particle(particle);
		scaled_particle.particle.velocity *= speed_scale_factor;
		scaled_particle.inverse_mass *= speed_scale_factor * speed_scale_factor;

		out_gravity_scale_factor = (scaled_particle.particle.velocity.magnitude_sqr() / particle.particle.velocity.magnitude_sqr());

		return scaled_particle;
	}

	mass_particle::particle_deconstruct mass_particle::deconstruct() const {
		return std::tuple_cat(
			particle.deconstruct(),
			std::make_tuple(f32_tag<struct inverse_mass>{ inverse_mass })
		);
	}
}