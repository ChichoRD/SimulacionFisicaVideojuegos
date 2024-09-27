#include "../types/v3_f32.hpp"

namespace objects {
	using namespace types;

	typedef v3_f32 position3_f32;
	typedef v3_f32 velocity3_f32;

	typedef double seconds_f64;

	struct particle {
	private:
		v3_f32 _position;
		v3_f32 _velocity;

	public:
		particle(v3_f32 position, v3_f32 velocity);
		particle(particle const &other);
		particle(particle&& other) noexcept;
		~particle();

	public:
		position3_f32 integrate(seconds_f64 delta_time);

	public:
		particle &operator=(particle const& other) noexcept {
			this->_position = other._position;
			this->_velocity = other._velocity;
		}

		particle& operator=(particle&& other) noexcept {
			this->_position = std::move(other._position);
			this->_velocity = std::move(other._position);
		}
	};
}