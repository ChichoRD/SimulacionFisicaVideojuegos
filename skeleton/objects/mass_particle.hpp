#ifndef MASS_PARTICLE_HPP
#define MASS_PARTICLE_HPP

#include "particle.hpp"

namespace objects {
	typedef f32 mass_f32;
	typedef f32 inverse_mass_f32;

	typedef v3_f32 impulse3_f32;

	typedef acceleration3_f32 gravity3_f32;

	enum integrator_model {
		EULER,
		SEMI_IMPLICIT_EULER,
		MIDPOINT,
		VERLET
	};

	struct mass_particle {
	public:
		particle particle;
		inverse_mass_f32 inverse_mass;

	public:
		mass_particle() noexcept;
		mass_particle(objects::particle particle, mass_f32 mass) noexcept;
		mass_particle(mass_particle const& other) noexcept;

	public:
		void add_impulse(impulse3_f32 impulse) noexcept;
		void add_velocity(velocity3_f32 velocity) noexcept;

	public:
		position3_f32 integrate(
			acceleration3_f32 acceleration,
			f32 damping_factor,
			seconds_f64 delta_time,
			integrator_model integrator = integrator_model::MIDPOINT
		) noexcept;

	public:
		static mass_particle scale_preserve_cinetic_energy(
			mass_particle const& particle,
			f32 speed_scale_factor,
			f32 & out_gravity_scale_factor
		) noexcept;
	};
}

#endif // !MASS_PARTICLE_HPP