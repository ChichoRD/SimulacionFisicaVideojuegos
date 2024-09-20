#include "v3_f32.hpp"

float &v3_f32::x() {
	return elements.at(0);
}

float &v3_f32::y() {
	return elements.at(1);
}

float &v3_f32::z() {
	return elements.at(2);
}

float& v3_f32::operator[](size_t i) {
	return elements.at(i);
}