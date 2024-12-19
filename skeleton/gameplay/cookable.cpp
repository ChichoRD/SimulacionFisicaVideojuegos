#include "cookable.hpp"

cookable::cookable(objects::seconds_f64 cook_time, objects::solid_dynamic_multishape_particle &&solid)
    : cook_time(cook_time), solid(std::move(solid)) {
}

cookable cookable::create_egg(physx::PxPhysics &physics, physx::PxTransform const &transform, objects::seconds_f64 cook_time) {
    return cookable();
}
