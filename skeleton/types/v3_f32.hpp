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

	union v3_f32 {
	private:
		std::array<f32, 3> _elements;

	public:
		struct {
			f32 x, y, z;
		};

	public:
		f32 magnitude_sqr();
		f32 magnitude();

		v3_f32 normalized();

	public:
		v3_f32() noexcept;
		v3_f32(f32 x, f32 y, f32 z) noexcept;
		v3_f32(v3_f32 const &v) noexcept;
		~v3_f32();

		static f32 dot(v3_f32 u, v3_f32 v);

	public:
		inline f32& operator[](usize i) {
			return _elements.at(i);
		}
		constexpr f32 const& operator[](usize i) const {
			return _elements.at(i);
		}

		v3_f32 &operator=(v3_f32 const &v) noexcept {
			_elements = v._elements;
			return *this;
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

		v3_f32 operator-() noexcept {
			return v3_f32(-x, -y, -z);
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
		friend v3_f32 operator*(f32 s, v3_f32 &u) noexcept {
			u *= s;
			return u;
		}

		friend v3_f32 operator/(v3_f32 u, f32 const& s) noexcept {
			u /= s;
			return u;
		}
		friend v3_f32 operator/(f32 s, v3_f32 &u) noexcept {
			u /= s;
			return u;
		}


		operator physx::PxVec3() {
			return physx::PxVec3(x, y, z);
		}
	};

}
