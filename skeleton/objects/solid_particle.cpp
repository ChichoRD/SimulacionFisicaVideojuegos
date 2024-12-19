#include "solid_particle.hpp"

static physx::PxShape *init_rigid_dynamic(
    physx::PxPhysics &physics,
    physx::PxRigidDynamic *rigid_dynamic,
    physx::PxMaterial const &material,
    physx::PxGeometry const &geometry,
    physx::PxVec3 const &mass_space_inertia_tensor
) {
    physx::PxShape *shape = rigid_dynamic->createShape(geometry, material);
    rigid_dynamic->attachShape(*shape);
    rigid_dynamic->setMassSpaceInertiaTensor(mass_space_inertia_tensor);
    return shape;
}

objects::solid_dynamic_particle::solid_dynamic_particle(
    physx::PxPhysics &physics,
    physx::PxTransform const &transform,
    physx::PxMaterial const &material,
    physx::PxGeometry const &geometry,
    physx::PxVec4 const &color,
    physx::PxVec3 const &mass_space_inertia_tensor
) : rigid_dynamic(physics.createRigidDynamic(transform)),
    render_item(new RenderItem(
        init_rigid_dynamic(physics, rigid_dynamic, material, geometry, mass_space_inertia_tensor), rigid_dynamic, color
    )) {
}

static physx::PxShape *init_rigid_static(
    physx::PxPhysics &physics,
    physx::PxRigidStatic *rigid_static,
    physx::PxMaterial const &material,
    physx::PxGeometry const &geometry
) {
    physx::PxShape *shape = rigid_static->createShape(geometry, material);
    rigid_static->attachShape(*shape);
    
    return shape;
}

objects::solid_static_particle::solid_static_particle(
    physx::PxPhysics &physics,
    physx::PxTransform const &transform,
    physx::PxMaterial const &material,
    physx::PxGeometry const &geometry,
    physx::PxVec4 const &color
) : rigid_static(physics.createRigidStatic(transform)),
    render_item(new RenderItem(
        init_rigid_static(physics, rigid_static, material, geometry), rigid_static, color
    )) {
}
