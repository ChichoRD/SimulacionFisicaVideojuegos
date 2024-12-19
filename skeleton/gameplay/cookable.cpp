#include "cookable.hpp"

cookable::cookable(objects::seconds_f64 cook_time, objects::solid_dynamic_multishape_particle &&solid)
    : cook_time(cook_time), solid(std::move(solid)), current_cook_time(0.0), previous_cook_time(0.0) {
}

objects::seconds_f64 cookable::cook(objects::seconds_f64 delta_time) {
    previous_cook_time = current_cook_time;
    current_cook_time += delta_time;
    return current_cook_time;
}

cookable cookable::create_egg(
    physx::PxPhysics &physics,
    physx::PxTransform const &transform,
    physx::PxBoxGeometry const &scale_bounds,
    objects::seconds_f64 cook_time
) {
    auto sphere = physx::PxSphereGeometry(scale_bounds.halfExtents.y * 0.5f);
    auto box = physx::PxBoxGeometry(scale_bounds.halfExtents.x, scale_bounds.halfExtents.y * 0.25f, scale_bounds.halfExtents.z);
    return cookable(cook_time, objects::solid_dynamic_multishape_particle::builder()
        .add_shape(
            sphere,
            physx::PxTransform(),
            physx::PxVec4(1.0f, 1.0f, 0.0f, 1.0f),
            objects::hollow_sphere_mass_space_inertia_tensor(sphere, 1.2f)
        )
        .add_shape(
            box,
            physx::PxTransform(
                physx::PxVec3(0.0f, -box.halfExtents.y, 0.0f)
            ),
            physx::PxVec4(1.0f, 1.0f, 1.0f, 1.0f),
            objects::box_mass_space_inertia_tensor(box, 1.0f)
        )
        .build(physics, transform, *physics.createMaterial(0.7f, 0.45f, 0.1f))
    );
}

cookable cookable::create_steak(
    physx::PxPhysics &physics,
    physx::PxTransform const &transform,
    physx::PxBoxGeometry const &scale_bounds,
    objects::seconds_f64 cook_time
) {
    auto box0 = physx::PxBoxGeometry(scale_bounds.halfExtents.x * 0.75f, scale_bounds.halfExtents.y * 0.25f, scale_bounds.halfExtents.z * 0.95f);
    auto box1 = physx::PxBoxGeometry(scale_bounds.halfExtents.x * 0.55f, scale_bounds.halfExtents.y * 0.5f, scale_bounds.halfExtents.z * 0.65f);
    return cookable(cook_time, objects::solid_dynamic_multishape_particle::builder()
        .add_shape(
            box0,
            physx::PxTransform(),
            physx::PxVec4(0.5f, 0.25f, 0.0f, 1.0f) * 0.5f,
            objects::box_mass_space_inertia_tensor(box0, 4.0f)
        )
        .add_shape(
            box1,
            physx::PxTransform(
                physx::PxVec3(-box0.halfExtents.z * 0.5f, 0.0f, -box1.halfExtents.x)
            ),
            physx::PxVec4(0.5f, 0.25f, 0.0f, 1.0f) * 0.5f,
            objects::box_mass_space_inertia_tensor(box1, 2.0f)
        )
        .build(physics, transform, *physics.createMaterial(0.65f, 0.55f, 0.25f))
    );
}

cookable cookable::create_fries(
    physx::PxPhysics &physics,
    physx::PxTransform const &transform,
    physx::PxBoxGeometry const &scale_bounds,
    objects::seconds_f64 cook_time
) {
    return cookable(cook_time, objects::solid_dynamic_multishape_particle::builder()
        .add_shape(
            physx::PxCapsuleGeometry(scale_bounds.halfExtents.y * 0.25f, scale_bounds.halfExtents.x * 0.75f),
            physx::PxTransform(),
            physx::PxVec4(1.0f, 1.0f, 0.0f, 1.0f),
            objects::box_mass_space_inertia_tensor(physx::PxBoxGeometry(scale_bounds.halfExtents.x * 0.25f, scale_bounds.halfExtents.y * 0.25f, scale_bounds.halfExtents.z * 0.25f), 0.5f)
        )
        .add_shape(
            physx::PxCapsuleGeometry(scale_bounds.halfExtents.y * 0.25f, scale_bounds.halfExtents.x * 0.65f),
            physx::PxTransform(
                physx::PxVec3(0.0f, 0.0f, -scale_bounds.halfExtents.z * 0.5f),
                physx::PxQuat(physx::PxPi * 0.25f, physx::PxVec3(0.0f, 1.0f, 0.0f))
            ),
            physx::PxVec4(1.0f, 1.0f, 0.0f, 1.0f),
            objects::box_mass_space_inertia_tensor(physx::PxBoxGeometry(scale_bounds.halfExtents.x * 0.25f, scale_bounds.halfExtents.y * 0.25f, scale_bounds.halfExtents.z * 0.25f), 0.5f)
        )
        .add_shape(
            physx::PxCapsuleGeometry(scale_bounds.halfExtents.y * 0.25f, scale_bounds.halfExtents.x * 0.25f),
            physx::PxTransform(
                physx::PxVec3(0.0f, 0.0f, scale_bounds.halfExtents.z * 0.5f),
                physx::PxQuat(-physx::PxPi * 0.25f * 0.25f, physx::PxVec3(0.0f, 1.0f, 0.0f))
            ),
            physx::PxVec4(1.0f, 1.0f, 0.0f, 1.0f),
            objects::box_mass_space_inertia_tensor(physx::PxBoxGeometry(scale_bounds.halfExtents.x * 0.65f, scale_bounds.halfExtents.y * 0.25f, scale_bounds.halfExtents.z * 0.25f), 0.5f)
        )
        .add_shape(
            physx::PxCapsuleGeometry(scale_bounds.halfExtents.y * 0.25f, scale_bounds.halfExtents.x * 0.25f),
            physx::PxTransform(
                physx::PxVec3(0.0f, 0.0f, scale_bounds.halfExtents.z * 1.5f),
                physx::PxQuat(physx::PxPi * 0.25f * 0.5f, physx::PxVec3(0.0f, 1.0f, 0.0f))
            ),
            physx::PxVec4(1.0f, 1.0f, 0.0f, 1.0f),
            objects::box_mass_space_inertia_tensor(physx::PxBoxGeometry(scale_bounds.halfExtents.x * 0.85f, scale_bounds.halfExtents.y * 0.25f, scale_bounds.halfExtents.z * 0.25f), 0.5f)
        )
        .add_shape(
            physx::PxCapsuleGeometry(scale_bounds.halfExtents.y * 0.25f, scale_bounds.halfExtents.x * 0.85f),
            physx::PxTransform(
                physx::PxVec3(0.0f, 0.0f, scale_bounds.halfExtents.z * 1.0f),
                physx::PxQuat(physx::PxPi * 0.15f, physx::PxVec3(0.0f, 1.0f, 0.0f))
            ),
            physx::PxVec4(1.0f, 1.0f, 0.0f, 1.0f),
            objects::box_mass_space_inertia_tensor(physx::PxBoxGeometry(scale_bounds.halfExtents.x * 0.25f, scale_bounds.halfExtents.y * 0.25f, scale_bounds.halfExtents.z * 0.25f), 0.5f)
        )
        .build(physics, transform, *physics.createMaterial(0.85f, 0.25f, 0.35f))
    );
}
