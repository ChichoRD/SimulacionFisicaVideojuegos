#include "pan.hpp"
#include <cmath>

pan::pan(physx::PxPhysics &physics, physx::PxTransform const &transform)
    : pan_solid(create_pan(physics, transform, base_shape))
{
}

objects::solid_dynamic_multishape_particle pan::create_pan(
    physx::PxPhysics &physics,
    physx::PxTransform const &transform,
    physx::PxShape *&out_base_shape
) {
    std::vector<physx::PxGeometry const *> geometries;
    std::vector<physx::PxTransform const *> local_poses;
    std::vector<physx::PxVec4> colors;
    std::vector<physx::PxVec3> mass_space_inertia_tensors;

    physx::PxMaterial *material = physics.createMaterial(0.5f, 0.5f, 0.6f);
    types::v3_f32 local_attachment_point = {0.0f, 0.0f, -pan::pan_length_unit * 0.75f};

    types::v3_f32 handle_color = {0.25f, 0.25f, 0.25f};
    types::f32 handle_mass = 1.0f;
    for (size_t i = 0; i < pan::pan_handle_straight_pieces_count; ++i) {
        auto handle_box = new physx::PxBoxGeometry(pan::straight_piece_size + types::v3_f32{0.0f, 0.0f, pan::straight_piece_size.z});
        geometries.push_back(handle_box);

        auto handle_local_pose = new physx::PxTransform(local_attachment_point);
        local_poses.push_back(handle_local_pose);

        colors.push_back(physx::PxVec4(handle_color, 1.0f));
        mass_space_inertia_tensors.push_back(objects::box_mass_space_inertia_tensor(
            *handle_box, handle_mass
        ));

        local_attachment_point.z += pan::straight_piece_size.z * 4.0f;
        local_attachment_point.y -= pan::straight_piece_size.y;
    }

    physx::PxVec3 initial_direction = {0.0f, 0.0f, 1.0f};
    physx::PxVec3 rotated_direction = physx::PxVec3(pan::pan_handle_remaining_displacement).getNormalized();
    types::f32 dot = initial_direction.dot(rotated_direction);
    types::f32 angle = std::acos(dot);
    physx::PxVec3 axis = initial_direction.cross(rotated_direction).getNormalized();

    physx::PxQuat rotation_quaternion = physx::PxQuat(angle, axis);
    for (size_t i = 0; i < pan::pan_handle_diagonal_pieces_count; ++i) {
        auto handle_box = new physx::PxBoxGeometry(pan::pan_handle_diagonal_piece_size);
        geometries.push_back(handle_box);

        auto handle_local_pose = new physx::PxTransform(
            local_attachment_point,
            rotation_quaternion
        );
        local_poses.push_back(handle_local_pose);

        colors.push_back(physx::PxVec4(handle_color, 1.0f));
        mass_space_inertia_tensors.push_back(objects::box_mass_space_inertia_tensor(
            *handle_box, handle_mass
        ));
        
        local_attachment_point.z += rotation_quaternion.rotate({0.0f, 0.0f, pan::pan_handle_diagonal_piece_size.z}).z;
    }

    // TODO: maybe cache base
    auto base_box = new physx::PxBoxGeometry(pan::pan_base_size);
    geometries.push_back(base_box);

    auto base_local_pose = new physx::PxTransform(
        local_attachment_point
    );
    local_poses.push_back(base_local_pose);

    types::v3_f32 base_color = handle_color * 0.5f;
    colors.push_back(physx::PxVec4(base_color, 1.0f));

    types::f32 base_mass = 10.0f;
    mass_space_inertia_tensors.push_back(objects::box_mass_space_inertia_tensor(
        *base_box, base_mass
    ));
    size_t base_index = geometries.size() - 1;

    
    types::v3_f32 base_centre = local_attachment_point;
    for (size_t i = 0; i < 4; ++i) {
        auto base_wall_box = new physx::PxBoxGeometry(pan::pan_base_wall_size);
        geometries.push_back(base_wall_box);

        constexpr double pi = 3.1415926535897932384626433832795028841971693993751058209;
        types::f32 side_angle = i * pi * 0.5f;

        constexpr types::f32 wall_offset = 0.75f;
        auto base_wall_local_pose = new physx::PxTransform(
            base_centre + types::v3_f32{
                std::cos(side_angle) * (pan::pan_base_size.x - pan::pan_base_wall_size.x * wall_offset),
                pan::pan_base_size.y + pan::pan_base_wall_size.y * 0.5f,
                std::sin(side_angle) * (pan::pan_base_size.z - pan::pan_base_wall_size.y * wall_offset)
            },
            physx::PxQuat(side_angle, {0.0f, 1.0f, 0.0f})
        );
        local_poses.push_back(base_wall_local_pose);

        colors.push_back(physx::PxVec4(base_color, 1.0f));
        mass_space_inertia_tensors.push_back(objects::box_mass_space_inertia_tensor(
            *base_wall_box, base_mass
        ));
    }

    auto pan = objects::solid_dynamic_multishape_particle(
        physics,
        transform,
        *material,
        geometries,
        local_poses,
        colors,
        mass_space_inertia_tensors
    );
    pan.rigid_dynamic->setRigidBodyFlag(physx::PxRigidBodyFlag::eKINEMATIC, true);
    pan.rigid_dynamic->setActorFlag(physx::PxActorFlag::eDISABLE_GRAVITY, true);

    pan.rigid_dynamic->getShapes(&out_base_shape, 1, base_index);

    for (physx::PxTransform const *local_pose : local_poses) {
        delete local_pose;
    }
    for (physx::PxGeometry const *geometry : geometries) {
        delete geometry;
    }
    return pan;
}
