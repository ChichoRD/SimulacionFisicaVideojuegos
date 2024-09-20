#pragma once

#include <array>

struct v3_f32 {
private:
	std::array<float, 3> elements;
public:
	float &x();
	float &y();
	float &z();

	float &operator[](size_t i);
};