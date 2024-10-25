#include <cassert>
#include <cstdlib>

#include "particle_generator.hpp"

namespace systems {
    particle_generator::particle_generator(distribution_type distribution, generation_shape shape, generation_volume volume)
        : distribution(distribution), shape(shape), volume(volume), generator() { }

    types::v3_f32 particle_generator::generate_position_outwards(
        types::v3_f32 &out_outwards_direction,
        types::f32 begin_normalized,
        types::f32 end_normalized,
        types::f32 standard_deviation
    ) {
        switch (distribution) {
        case UNIFORM: {
            return generate_uniform_position_outwards(
                *this,
                out_outwards_direction,
                begin_normalized,
                end_normalized
            );
        }
        case NORMAL: {
            return generate_normal_position_outwards(
                *this,
                out_outwards_direction,
                begin_normalized,
                end_normalized,
                standard_deviation
            );
        }   
        default: {
            assert(false && "unreachable: invalid distribution type");
            std::exit(EXIT_FAILURE);
        }
        }
    }

    static types::v3_f32 generate_uniform_position_outwards(
        particle_generator &generator,
        types::v3_f32 &out_outwards_direction,
        types::f32 begin_normalized,
        types::f32 end_normalized
    ) {
        types::v3_f32 position;
        switch (generator.shape) {
        case particle_generator::BOX: {
            position = generator.volume.box.random_uniform(generator.generator);
            out_outwards_direction = position - generator.volume.box.centre;
            return position;
        }
        case particle_generator::ELLIPSOID: {
            position = generator.volume.ellipsoid.random_uniform(generator.generator);
            out_outwards_direction = position - generator.volume.ellipsoid.centre;
            return position;
        }
        case particle_generator::CYLINDER: {
            position = generator.volume.cylinder.random_uniform(generator.generator, begin_normalized, end_normalized);
            out_outwards_direction = position - generator.volume.cylinder.bottom;
            return position;
        }
        case particle_generator::CONE: {
            position = generator.volume.cone.random_uniform(generator.generator, begin_normalized, end_normalized);
            out_outwards_direction = position - generator.volume.cone.vertex;
            return position;
        }
        default: {
            assert(false && "unreachable: invalid generation shape");
            std::exit(EXIT_FAILURE);
            return position;
        }
        }
    }

    static types::v3_f32 generate_normal_position_outwards(
        particle_generator &generator,
        types::v3_f32 &out_outwards_direction,
        types::f32 begin_normalized,
        types::f32 end_normalized,
        types::f32 standard_deviation
    ) {
        types::v3_f32 position;
        switch (generator.shape) {
        case particle_generator::BOX: {
            position = generator.volume.box.random_normal(generator.generator, standard_deviation);
            out_outwards_direction = position - generator.volume.box.centre;
            return position;
        }
        case particle_generator::ELLIPSOID: {
            position = generator.volume.ellipsoid.random_normal(generator.generator, standard_deviation);
            out_outwards_direction = position - generator.volume.ellipsoid.centre;
            return position;
        }
        case particle_generator::CYLINDER: {
            position = generator.volume.cylinder.random_normal(generator.generator, begin_normalized, end_normalized, standard_deviation);
            out_outwards_direction = position - generator.volume.cylinder.bottom;
            return position;
        }
        case particle_generator::CONE: {
            position = generator.volume.cone.random_normal(generator.generator, begin_normalized, end_normalized, standard_deviation);
            out_outwards_direction = position - generator.volume.cone.vertex;
            return position;
        }
        default: {
            assert(false && "unreachable: invalid generation shape");
            std::exit(EXIT_FAILURE);
            return position;
        }
        }
    }
}