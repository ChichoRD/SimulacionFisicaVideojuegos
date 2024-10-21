#ifndef v3_HPP
#define v3_HPP

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

	template <
		typename T,
		typename Tag = void,
		typename = typename std::enable_if_t<std::is_arithmetic_v<T>>
	>
	union v3 {
	private:
		std::array<T, 3> _elements;

	public:
		struct {
			T x, y, z;
		};

	public:
		constexpr T magnitude_sqr() const {
			return v3::dot(*this, *this);
		}

		constexpr v3 dot_inverse() const {
			return *this / this->magnitude_sqr();
		}

		T magnitude() const {
			return std::sqrtf(magnitude_sqr());
		}

		v3 normalized() const {
			return *this / this->magnitude();
		}


	public:
		constexpr v3() noexcept
			: x(T()), y(T()), z(T()) { }
		constexpr v3(T x, T y, T z) noexcept
			: x(x), y(y), z(z) { }
		
		template <typename Other_Tag>
		constexpr v3(v3<T, Other_Tag> const &v)
			: x(v.x), y(v.y), z(v.z) { }

		v3(physx::PxVec3 v) noexcept
			: x(v.x), y(v.y), z(v.z) { }
		~v3() = default;

		constexpr static T dot(v3 u, v3 v) {
			return u.x * v.x
				+ u.y * v.y
				+ u.z + v.z;
		}

		constexpr static v3 cross(v3 u, v3 v) {
			return {
				u.y * v.z - u.z * v.y,
				u.z * v.x - u.x * v.z,
				u.x * v.y - u.y * v.x
			};
		}

		constexpr static v3 lerp(v3 u, v3 v, T t) {
			return u + (v - u) * t;
		}

	public:
		inline T& operator[](usize i) {
			return _elements.at(i);
		}
		constexpr T const& operator[](usize i) const {
			return _elements.at(i);
		}

		v3 &operator=(v3 const &v) noexcept {
			_elements = v._elements;
			return *this;
		}

		constexpr v3& operator+=(v3 const& v) noexcept {
			this->x += v.x;
			this->y += v.y;
			this->z += v.z;
			return *this;
		}
		constexpr v3& operator-=(v3 const& v) noexcept {
			this->x -= v.x;
			this->y -= v.y;
			this->z -= v.z;
			return *this;
		}
		constexpr v3& operator*=(T const& s) noexcept {
			this->x *= s;
			this->y *= s;
			this->z *= s;
			return *this;
		}
		constexpr v3& operator*=(v3 const& v) noexcept {
			this->x *= v.x;
			this->y *= v.y;
			this->z *= v.z;
			return *this;
		}
		constexpr v3& operator/=(T const& s) noexcept {
			this->x /= s;
			this->y /= s;
			this->z /= s;
			return *this;
		}

		constexpr v3 operator-() noexcept {
			return v3(-x, -y, -z);
		}

		friend constexpr v3 operator+(v3 u, v3 const& v) noexcept {
			u += v;
			return u;
		}

		friend constexpr v3 operator-(v3 u, v3 const& v) noexcept {
			u -= v;
			return u;
		}

		friend constexpr v3 operator*(v3 u, T const& s) noexcept {
			u *= s;
			return u;
		}
		friend constexpr v3 operator*(T s, v3 &u) noexcept {
			u *= s;
			return u;
		}

		friend constexpr v3 operator*(v3 u, v3 const& v) noexcept {
			u *= v;
			return u;
		}

		friend constexpr v3 operator/(v3 u, T const& s) noexcept {
			u /= s;
			return u;
		}

		operator physx::PxVec3() {
			return physx::PxVec3(x, y, z);
		}
	};

	//template <typename Tag = void>
	using v3_f32 = v3<f32>;
}

#endif // !v3_HPP