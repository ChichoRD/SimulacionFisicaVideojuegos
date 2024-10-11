#include <cmath>
#include "v3_f32.hpp"

namespace types {
	f32 v3_f32::magnitude_sqr() const {
		return v3_f32::dot(*this, *this);
	}

	f32 v3_f32::magnitude() const {
		return std::sqrtf(magnitude_sqr());
	}

	v3_f32 v3_f32::normalized() const {
		return *this / this->magnitude();
	}


	v3_f32::v3_f32() noexcept
		: x(0), y(0), z(0) { }

	v3_f32::v3_f32(f32 x, f32 y, f32 z) noexcept
		: x(x), y(y), z(z) { }

	v3_f32::v3_f32(physx::PxVec3 v) noexcept
		: x(v.x), y(v.y), z(v.z) { }

	v3_f32::v3_f32(v3_f32 const& v) noexcept
		: x(v.x), y(v.y), z(v.z) { }

	v3_f32::~v3_f32() = default;

	f32 v3_f32::dot(v3_f32 u, v3_f32 v) {
		return u.x * v.x
			+ u.y * v.y
			+ u.z * v.z;
	}
}
