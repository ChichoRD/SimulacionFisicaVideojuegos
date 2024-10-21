#include <random>
#include <cmath>
#include <array>

#include "../types/v3_f32.hpp"

namespace systems {
    struct generation_box {
        types::v3_f32 centre;
        types::v3_f32 half_extents;

        template <typename Generator>
        inline types::v3_f32 random_uniform(Generator &generator) const {
            return {
                std::uniform_real<types::f32>(centre.x - half_extents.x, centre.x + half_extents.x)(generator),
                std::uniform_real<types::f32>(centre.y - half_extents.y, centre.y + half_extents.y)(generator),
                std::uniform_real<types::f32>(centre.z - half_extents.z, centre.z + half_extents.z)(generator)
            };
        }

        template <typename Generator>
        inline types::v3_f32 random_normal(Generator &generator, types::f32 standard_deviation) const {
            return {
                std::normal_distribution<types::f32>(centre.x, half_extents.x * standard_deviation)(generator),
                std::normal_distribution<types::f32>(centre.y, half_extents.y * standard_deviation)(generator),
                std::normal_distribution<types::f32>(centre.z, half_extents.z * standard_deviation)(generator)
            };
        }
    };

    struct generation_ellipsoid {
        types::v3_f32 centre;
        types::v3_f32 radii;
        template <typename Generator>
        types::v3_f32 random_unit_uniform(Generator &generator) const {
            types::f32 u = std::uniform_real<types::f32>(0.0f, 1.0f)(generator);
            types::f32 v = std::uniform_real<types::f32>(0.0f, 1.0f)(generator);

            types::f32 theta = u * 2.0f * std::_Pi_val;
            types::f32 phi = std::acosf(2.0f * v - 1.0f);
            
            types::f32 r = std::cbrtf(std::uniform_real<types::f32>(0.0f, 1.0f)(generator));

            types::f32 sin_theta = std::sinf(theta);
            types::f32 cos_theta = std::cosf(theta);
            types::f32 sin_phi = std::sinf(phi);
            types::f32 cos_phi = std::cosf(phi);

            types::f32 x = r * sinPhi * cosTheta;
            types::f32 y = r * sinPhi * sinTheta;
            types::f32 z = r * cosPhi;
            return { x, y, z };
        }

        template <typename Generator>
        inline types::v3_f32 random_uniform(Generator &generator) const {
            return centre + (random_unit_uniform(generator) * radii);
        }

        template <typename Generator>
        inline types::v3_f32 random_normal(Generator &generator, types::f32 standard_deviation) const {
            return centre + (random_unit_uniform(generator) * {
                std::normal_distribution<types::f32>(radii.x, standard_deviation)(generator),
                std::normal_distribution<types::f32>(radii.y, standard_deviation)(generator),
                std::normal_distribution<types::f32>(radii.z, standard_deviation)(generator)
            });
        }
    };

    struct generation_cylinder {
        types::v3_f32 bottom;
        types::v3_f32 top;

        types::f32 radius_x;
        types::f32 radius_z;

        inline void tangents(types::v3_f32 &out_tangent, types::v3_f32 &out_bitangent) const {
            types::v3_f32 const axis = top - bottom;
            std::array<types::v3_f32, 3> constexpr basis = {
                types::v3_f32{1.0f, 0.0f, 0.0f},
                {0.0f, 1.0f, 0.0f},
                {0.0f, 0.0f, 1.0f}
            };
            size_t min_basis_index = 0;
            types::f32 min_basis_dot = std::numeric_limits<types::f32>::max();
            for (size_t i = 0; i < basis.size(); ++i) {
                types::f32 dot = types::v3_f32::dot(axis, basis[i]);
                if (dot < min_basis_dot) {
                    min_basis_dot = dot;
                    min_basis_index = i;
                }
            }

            out_tangent = types::v3_f32::cross(axis, basis[min_basis_index]).normalized();
            out_bitangent = types::v3_f32::cross(out_tangent, axis).normalized();  
        }

        template <typename Generator>
        inline types::v3_f32 random_uniform(
            Generator &generator,
            types::f32 begin_normalized = 0.0f,
            types::f32 end_normalized = 1.0f
        ) const {
            types::f32 const theta = std::uniform_real<types::f32>(0.0f, 2.0f * std::_Pi_val)(generator);
            types::f32 const r = std::sqrtf(std::uniform_real<types::f32>(0.0f, 1.0f)(generator));

            types::f32 const x = r * std::cosf(theta) * radius_x;
            types::f32 const y = r * std::sinf(theta) * radius_z;

            types::v3_f32 tangent;
            types::v3_f32 bitangent;  
            tangents(tangent, bitangent);
            
            return types::v3_f32::lerp(
                bottom,
                top,
                std::uniform_real<types::f32>(begin_normalized, end_normalized)(generator)
            ) + tangent * x + bitangent * y;
        }

        template <typename Generator>
        inline types::v3_f32 random_normal(
            Generator &generator,
            types::f32 begin_normalized = 0.0f,
            types::f32 end_normalized = 1.0f,
            types::f32 standard_deviation
        ) const {
            types::f32 const theta = std::uniform_real<types::f32>(0.0f, 2.0f * std::_Pi_val)(generator);
            types::f32 const r = std::sqrtf(std::normal_distribution<types::f32>(1.0f, standard_deviation)(generator));

            types::f32 const x = r * std::cosf(theta) * radius_x;
            types::f32 const y = r * std::sinf(theta) * radius_z;

            types::v3_f32 tangent;
            types::v3_f32 bitangent;  
            tangents(tangent, bitangent);
            
            return types::v3_f32::lerp(
                bottom,
                top,
                std::uniform_real<types::f32>(begin_normalized, end_normalized)(generator)
            ) + tangent * x + bitangent * y;
        }
    };

    struct generation_cone {
        types::v3_f32 base;
        types::v3_f32 vertex;

        types::f32 radius_x;
        types::f32 radius_z;

        template <typename Generator>
        inline types::v3_f32 random_uniform(
            Generator &generator,
            types::f32 begin_normalized = 0.0f,
            types::f32 end_normalized = 1.0f
        ) const {
            types::f32 const theta = std::uniform_real<types::f32>(0.0f, 2.0f * std::_Pi_val)(generator);
            types::f32 const r = std::sqrtf(std::uniform_real<types::f32>(0.0f, 1.0f)(generator));

            types::f32 const x = r * std::cosf(theta) * radius_x;
            types::f32 const y = r * std::sinf(theta) * radius_z;

            types::v3_f32 tangent;
            types::v3_f32 bitangent;  
            generation_cylinder{base, vertex, radius_x, radius_z}.tangents(tangent, bitangent);

            types::v3_f32 const base_point = base + tangent * x + bitangent * y;
            types::v3_f32 const begin_point = types::v3_f32::lerp(base_point, vertex, begin_normalized);
            types::v3_f32 const end_point = types::v3_f32::lerp(base_point, vertex, end_normalized);

            return types::v3_f32::lerp(
                begin_point,
                end_point,
                1.0f - std::sqrtf(std::uniform_real<types::f32>(0.0f, 1.0f)(generator))
            );
        }

        template <typename Generator>
        inline types::v3_f32 random_normal(
            Generator &generator,
            types::f32 begin_normalized = 0.0f,
            types::f32 end_normalized = 1.0f,
            types::f32 standard_deviation
        ) const {
            types::f32 const theta = std::uniform_real<types::f32>(0.0f, 2.0f * std::_Pi_val)(generator);
            types::f32 const r = std::sqrtf(std::normal_distribution<types::f32>(1.0f, standard_deviation)(generator));

            types::f32 const x = r * std::cosf(theta) * radius_x;
            types::f32 const y = r * std::sinf(theta) * radius_z;

            types::v3_f32 tangent;
            types::v3_f32 bitangent;  
            generation_cylinder{base, vertex, radius_x, radius_z}.tangents(tangent, bitangent);

            types::v3_f32 const base_point = base + tangent * x + bitangent * y;
            types::v3_f32 const begin_point = types::v3_f32::lerp(base_point, vertex, begin_normalized);
            types::v3_f32 const end_point = types::v3_f32::lerp(base_point, vertex, end_normalized);

            return types::v3_f32::lerp(
                begin_point,
                end_point,
                1.0f - std::sqrtf(std::uniform_real<types::f32>(0.0f, 1.0f)(generator))
            );
        }
    };

    struct particle_generator {
        enum distribution_type {
            UNIFORM,
            NORMAL
        };

        enum generation_shape {
            BOX,
            ELLIPSOID,
            CYLINDER,
            CONE,
        };
    };
}