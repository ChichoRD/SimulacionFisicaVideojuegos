#ifndef PAN_HPP
#define PAN_HPP

#include <PxPhysicsAPI.h>
#include "../objects/solid_particle.hpp"
#include "../types/v3_f32.hpp"

struct pan {
private:
    constexpr static types::f32 pan_length_unit = 0.5f;

    constexpr static size_t pan_handle_pieces_count = 2;
    constexpr static size_t pan_handle_straight_pieces_count = 1;
    static_assert(pan_handle_pieces_count > pan_handle_straight_pieces_count);
    constexpr static size_t pan_handle_diagonal_pieces_count = pan_handle_pieces_count - pan_handle_straight_pieces_count;

    constexpr static types::v3_f32 pan_handle_displacement = {0.0f, -pan_length_unit, pan_length_unit * 8.0f};
    
    constexpr static types::v3_f32 straight_piece_size = {pan_length_unit, pan_length_unit, pan_length_unit * 2.5f};
    constexpr static types::v3_f32 pan_handle_remaining_displacement =
        pan_handle_displacement
        - types::v3_f32{0.0f, 0.0f, straight_piece_size.z} * pan_handle_straight_pieces_count;
    constexpr static types::v3_f32 pan_handle_diagonal_piece_size = {pan_length_unit, pan_length_unit, pan_handle_remaining_displacement.z / pan_handle_diagonal_pieces_count};

    constexpr static types::v3_f32 pan_base_size = {pan_length_unit * 8.0f, pan_length_unit, pan_length_unit * 8.0f};
    constexpr static types::v3_f32 pan_base_wall_size = {pan_length_unit, pan_length_unit, pan_base_size.z};

    static objects::solid_dynamic_multishape_particle create_pan(physx::PxPhysics &physics, physx::PxTransform const &transform);

public:
    objects::solid_dynamic_multishape_particle pan_solid;

    pan(physx::PxPhysics &physics, physx::PxTransform const &transform);
};

#endif