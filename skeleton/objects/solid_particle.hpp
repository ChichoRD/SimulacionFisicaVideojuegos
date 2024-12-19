#ifndef SOLID_PARTICLE_HPP
#define SOLID_PARTICLE_HPP

#include <PxPhysicsAPI.h>
#include "../types/particle_defs.hpp"
#include "../RenderUtils.hpp"

namespace objects {
    using deconstruct_render_item = RenderItem *;

    struct solid_dynamic_particle {
        using deconstruct_rigid_dynamic = physx::PxRigidDynamic *;

        physx::PxRigidDynamic *rigid_dynamic;
        RenderItem *render_item;

        solid_dynamic_particle(
            physx::PxPhysics &physics,
            physx::PxTransform const &transform,
            physx::PxMaterial const &material,
            physx::PxGeometry const &geometry,
            physx::PxVec4 const &color,
            physx::PxVec3 const &mass_space_inertia_tensor
        );

        solid_dynamic_particle(deconstruct_rigid_dynamic rigid_dynamic, deconstruct_render_item render_item)
            : rigid_dynamic(rigid_dynamic), render_item(render_item) { }

        using particle_deconstruct = systems::particle_trait::particle_deconstruct<
            deconstruct_rigid_dynamic,
            deconstruct_render_item
        >;

        particle_deconstruct deconstruct() const {
            return std::make_tuple(rigid_dynamic, render_item);
        }
    };

    struct solid_static_particle {
        using deconstruct_rigid_static = physx::PxRigidStatic *;

        physx::PxRigidStatic *rigid_static;
        RenderItem *render_item;

        solid_static_particle(
            physx::PxPhysics &physics,
            physx::PxTransform const &transform,
            physx::PxMaterial const &material,
            physx::PxGeometry const &geometry,
            physx::PxVec4 const &color
        );

        solid_static_particle(deconstruct_rigid_static rigid_static, deconstruct_render_item render_item)
            : rigid_static(rigid_static), render_item(render_item) { }

        using particle_deconstruct = systems::particle_trait::particle_deconstruct<
            deconstruct_rigid_static,
            deconstruct_render_item
        >;

        particle_deconstruct deconstruct() const {
            return std::make_tuple(rigid_static, render_item);
        }
    };

    struct solid_dynamic_multishape_particle {
        physx::PxRigidDynamic *rigid_dynamic;
        std::vector<RenderItem *> render_items;

        solid_dynamic_multishape_particle(
            physx::PxPhysics &physics,
            physx::PxTransform const &transform,
            physx::PxMaterial const &material,
            std::vector<physx::PxGeometry const *> const &geometries,
            std::vector<physx::PxTransform const *> const &local_poses,
            std::vector<physx::PxVec4> const &colors,
            std::vector<physx::PxVec3> const &mass_space_inertia_tensors
        );

        ~solid_dynamic_multishape_particle();

        solid_dynamic_multishape_particle(solid_dynamic_multishape_particle const &) = delete;
        solid_dynamic_multishape_particle &operator=(solid_dynamic_multishape_particle const &) = delete;

        solid_dynamic_multishape_particle(solid_dynamic_multishape_particle &&other) noexcept;
        solid_dynamic_multishape_particle &operator=(solid_dynamic_multishape_particle &&other) noexcept;
    };

    physx::PxVec3 box_mass_space_inertia_tensor(physx::PxBoxGeometry const &geometry, physx::PxReal mass);
}

#endif