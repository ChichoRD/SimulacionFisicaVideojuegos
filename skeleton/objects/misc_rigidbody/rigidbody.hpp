#include <PxPhysicsAPI.h>
#include "../../types/v3_f32.hpp"
#include "../../objects/particle.hpp"

namespace objects::rb {
    struct rigidbody {
        physx::PxGeometry *geometry;

        objects::position3_f32 centre_of_mass;
        objects::v3_f32 velocity;

        types::f32 rotation_matrix[3][3];
        types::v3_f32 angular_velocity;
    };
}