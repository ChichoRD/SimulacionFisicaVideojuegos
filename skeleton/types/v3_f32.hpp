#pragma once

#include <array>
#include <cstdint>
#include <PxPhysicsAPI.h>

namespace types {

	typedef uint8_t u8;
	typedef uint16_t u16;
	typedef uint32_t u32;
	typedef uint64_t u64;

	typedef int8_t i8;
	typedef int16_t i16;
	typedef int32_t i32;
	typedef int64_t i64;

	typedef float f32;
	typedef double f64;

	typedef size_t usize;
	typedef ptrdiff_t isize;

	struct v3_f32 {
	public:
		f32 x, y, z;
	public:
		f32 magnitude_sqr();
		f32 magnitude();

		v3_f32 normalized();
	public:
		v3_f32(f32 x, f32 y, f32 z);
		v3_f32(v3_f32 const &v);
		v3_f32(v3_f32 &&v);

		static f32 dot(v3_f32 u, v3_f32 v);
	public:
		constexpr f32& operator[](usize i) {
			return *(&(this->x) + i);
		}

		v3_f32& operator+=(v3_f32 const& v) noexcept {
			this->x += v.x;
			this->y += v.y;
			this->z += v.z;
			return *this;
		}
		v3_f32& operator-=(v3_f32 const& v) noexcept {
			this->x -= v.x;
			this->y -= v.y;
			this->z -= v.z;
			return *this;
		}
		v3_f32& operator*=(f32 const& s) noexcept {
			this->x *= s;
			this->y *= s;
			this->z *= s;
			return *this;
		}
		v3_f32& operator/=(f32 const& s) noexcept {
			this->x /= s;
			this->y /= s;
			this->z /= s;
			return *this;
		}

		friend v3_f32 operator+(v3_f32 u, v3_f32 const& v) noexcept {
			u += v;
			return u;
		}
		friend v3_f32 operator-(v3_f32 u, v3_f32 const& v) noexcept {
			u -= v;
			return u;
		}
		friend v3_f32 operator*(v3_f32 u, f32 const& s) noexcept {
			u *= s;
			return u;
		}
		friend v3_f32 operator/(v3_f32 u, f32 const& s) noexcept {
			u /= s;
			return u;
		}

		operator physx::PxVec3() {
			return physx::PxVec3(x, y, z);
		}
	};

}
