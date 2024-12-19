#include "solid_particle.hpp"
#include <cassert>
#include <cmath>

static physx::PxShape *init_rigid_dynamic(
    physx::PxPhysics &physics,
    physx::PxRigidDynamic *rigid_dynamic,
    physx::PxMaterial const &material,
    physx::PxGeometry const &geometry,
    physx::PxVec3 const &mass_space_inertia_tensor
) {
    physx::PxShape *shape = rigid_dynamic->createShape(geometry, material);
    //rigid_dynamic->attachShape(*shape);
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
    //rigid_static->attachShape(*shape);
    
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

objects::solid_dynamic_multishape_particle::solid_dynamic_multishape_particle()
    : rigid_dynamic(nullptr), render_items{} { }

objects::solid_dynamic_multishape_particle::solid_dynamic_multishape_particle(
    physx::PxPhysics &physics,
    physx::PxTransform const &transform,
    physx::PxMaterial const &material,
    std::vector<physx::PxGeometry const *> const &geometries,
    std::vector<physx::PxTransform const *> const &local_poses,
    std::vector<physx::PxVec4> const &colors,
    std::vector<physx::PxVec3> const &mass_space_inertia_tensors) : rigid_dynamic(physics.createRigidDynamic(transform)),
                                                                    render_items{}
{
    assert(
        geometries.size() == local_poses.size()
        && local_poses.size() == colors.size()
        && colors.size() == mass_space_inertia_tensors.size()
        && "error: different number of geometries, colors, and mass space inertia tensors"
    );

    physx::PxVec3 mass_space_inertia_tensor = physx::PxVec3(0.0f);
    for (size_t i = 0; i < geometries.size(); ++i) {
        physx::PxShape *shape = rigid_dynamic->createShape(*geometries[i], material);

        physx::PxTransform const &local_pose = *local_poses[i];
        shape->setLocalPose(local_pose);

        render_items.push_back(new RenderItem(shape, rigid_dynamic, colors[i]));
        mass_space_inertia_tensor += mass_space_inertia_tensors[i];
    }
    rigid_dynamic->setMassSpaceInertiaTensor(mass_space_inertia_tensor);
}

objects::solid_dynamic_multishape_particle::~solid_dynamic_multishape_particle() {
    for (RenderItem *render_item : render_items) {
        DeregisterRenderItem(render_item);
        delete render_item;
    }
}

objects::solid_dynamic_multishape_particle::solid_dynamic_multishape_particle(
    solid_dynamic_multishape_particle &&other
) noexcept : rigid_dynamic(other.rigid_dynamic), render_items(std::move(other.render_items)) {
    other.rigid_dynamic = nullptr;
    other.render_items.clear();
}

objects::solid_dynamic_multishape_particle &objects::solid_dynamic_multishape_particle::operator=(
    solid_dynamic_multishape_particle &&other
) noexcept {
    if (this != &other) {
        rigid_dynamic = other.rigid_dynamic;
        render_items = std::move(other.render_items);

        other.rigid_dynamic = nullptr;
        other.render_items.clear();
    }
    return *this;
}

physx::PxVec3 objects::box_mass_space_inertia_tensor(physx::PxBoxGeometry const &geometry, physx::PxReal mass) {
    return (mass / 12.0f) * physx::PxVec3(
        geometry.halfExtents.y * geometry.halfExtents.y + geometry.halfExtents.z * geometry.halfExtents.z,
        geometry.halfExtents.x * geometry.halfExtents.x + geometry.halfExtents.z * geometry.halfExtents.z,
        geometry.halfExtents.x * geometry.halfExtents.x + geometry.halfExtents.y * geometry.halfExtents.y
    );
}

physx::PxVec3 objects::hollow_sphere_mass_space_inertia_tensor(physx::PxSphereGeometry const &geometry, physx::PxReal mass) {
    return physx::PxVec3(2.0f * mass * geometry.radius * geometry.radius / 3.0f);
}

physx::PxVec3 objects::solid_sphere_mass_space_inertia_tensor(physx::PxSphereGeometry const &geometry, physx::PxReal mass) {
    return physx::PxVec3(2.0f * mass * geometry.radius * geometry.radius / 5.0f);
}

physx::PxVec3 objects::shell_sphere_mass_space_inertia_tensor(
    physx::PxSphereGeometry const &inner_geometry,
    physx::PxSphereGeometry const &outter_geometry,
    physx::PxReal mass
) {
    return physx::PxVec3(
        (2.0f * mass * (std::pow(outter_geometry.radius, 5.0f) - std::pow(inner_geometry.radius, 5.0f)))
        / (5.0f * (std::pow(outter_geometry.radius, 3.0f) - std::pow(inner_geometry.radius, 3.0f)))
    );
}

physx::PxVec3 objects::solid_cylinder_mass_space_inertia_tensor(physx::PxCapsuleGeometry const &geometry, physx::PxReal mass) {
    float symmetric_part = mass * (geometry.radius * geometry.radius * 3.0f + geometry.halfHeight * geometry.halfHeight * 4.0f) / 12.0f;
    return physx::PxVec3(
        symmetric_part,
        mass * geometry.radius * geometry.radius * 0.5f,
        symmetric_part
    );
}

physx::PxVec3 objects::hollow_tube_mass_space_inertia_tensor(
    physx::PxCapsuleGeometry const &inner_geometry,
    physx::PxCapsuleGeometry const &outter_geometry,
    physx::PxReal mass
) {
    float sqr_radii_sum = outter_geometry.radius * outter_geometry.radius + inner_geometry.radius * inner_geometry.radius;
    float symmetric_part = mass * (sqr_radii_sum * 3.0f + inner_geometry.halfHeight * outter_geometry.halfHeight * 4.0f) / 12.0f;
    return physx::PxVec3(
        symmetric_part,
        mass * sqr_radii_sum * 0.5f,
        symmetric_part
    );
}
