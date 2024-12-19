#include "game_scene.hpp"
#include <cassert>
#include <iostream>

static bool init_game_scene_physics(
    physx::PxPhysics &physics,
    physx::PxScene *&out_scene,
    physx::PxDefaultCpuDispatcher *&out_dispatcher
) {
	physx::PxSceneDesc sceneDesc(physics.getTolerancesScale());
	sceneDesc.gravity = physx::PxVec3(0.0f, -9.81f, 0.0f);

	out_dispatcher = physx::PxDefaultCpuDispatcherCreate(2);
	sceneDesc.cpuDispatcher = out_dispatcher;
	sceneDesc.filterShader = contactReportFilterShader;

	sceneDesc.simulationEventCallback = &gContactReportCallback;

	out_scene = physics.createScene(sceneDesc);
	return true;
}

game_scene::game_scene(physx::PxPhysics &physics)
    : physics(physics),
    scene(nullptr),
    last_delta_time(0.0),
    next_pan_rotation(physx::PxQuat(0.0f, physx::PxVec3(0.0f, 0.0f, 1.0f))) {
    assert(init_game_scene_physics(physics, scene, gDispatcher));
    scene->setSimulationEventCallback(this);
}

game_scene::~game_scene() {
    scene->release();
}

bool game_scene::init() {
    ground = objects::solid_static_particle(
        physics,
        physx::PxTransform(physx::PxVec3(0.0f, -50.0f, 0.0f)),
        *physics.createMaterial(0.5f, 0.5f, 0.6f),
        physx::PxBoxGeometry(500.0f, 1.0f, 500.0f),
        physx::PxVec4(0.5f, 0.5f, 0.5f, 1.0f)
    );
    scene->addActor(*ground.rigid_static);

    frying_pan = pan(physics, physx::PxTransform(physx::PxVec3(0.0f, 5.0f, -2.5f)));
    scene->addActor(*frying_pan.pan_solid.rigid_dynamic);
    pan_particle_system = new systems::particle_system(
        systems::particle_generator(
            systems::particle_generator::NORMAL,
            systems::particle_generator::BOX,
            systems::particle_generator::generation_volume(
                systems::generation_box{ 
                    frying_pan.pan_solid.rigid_dynamic->getGlobalPose().transform(frying_pan.base_shape->getLocalPose()).p,
                    frying_pan.base_shape->getGeometry().box().halfExtents
                }
            )
        ),
        0.0f
    );

    scene->addActor(*add_cookable(cookable::create_egg(
        physics,
        physx::PxTransform(physx::PxVec3(0.0f, 10.0f, 0.0f)),
        physx::PxBoxGeometry(2.0f, 2.0f, 2.0f),
        10.0
    )).solid.rigid_dynamic);
    
    scene->addActor(*add_cookable(cookable::create_steak(
        physics,
        physx::PxTransform(physx::PxVec3(0.0f, 15.0f, 0.0f)),
        physx::PxBoxGeometry(2.0f, 1.0f, 2.0f),
        10.0
    )).solid.rigid_dynamic);

    scene->addActor(*add_cookable(cookable::create_fries(
        physics,
        physx::PxTransform(physx::PxVec3(0.0f, 20.0f, 0.0f)),
        physx::PxBoxGeometry(2.5f, 1.0f, 1.0f),
        10.0
    )).solid.rigid_dynamic);
    
    return true;
}

bool game_scene::update(objects::seconds_f64 delta_time) {
    // TODO
    last_delta_time = delta_time;
    for (auto it = cookables_to_remove.begin(); it != cookables_to_remove.end(); ++it) {
        auto cookable_it = cookable_map.find(*it);
        if (cookable_it != cookable_map.end()) {
            cookables.erase(cookable_it->second);
            cookable_map.erase(cookable_it);
        }
    }
    cookables_to_remove.clear();

    auto &frying_pan_rb = frying_pan.pan_solid.rigid_dynamic;
	frying_pan_rb->setKinematicTarget(physx::PxTransform(
		frying_pan_rb->getGlobalPose().p,
        next_pan_rotation
	));

    scene->simulate(delta_time);
	physx::PxU32 error_state = 0;
	scene->fetchResults(true, &error_state);

    return error_state == 0;
}

bool game_scene::shutdown() {
    {
        pan_particle_system->iter<RenderItem *>(
            [](RenderItem *r) {
                DeregisterRenderItem(r);
                delete r;
            }
        );

        for (size_t i = 0; i < pan_particle_system->particles.particle_count(); ++i) {
            pan_particle_system->remove_particle(i);
        }
        delete pan_particle_system;
    }

    // {
    //     cookables_particle_system->iter<RenderItem *>(
    //         [](RenderItem *r) {
    //             DeregisterRenderItem(r);
    //             delete r;
    //         }
    //     );

    //     for (size_t i = 0; i < cookables_particle_system->particles.particle_count(); ++i) {
    //         cookables_particle_system->remove_particle(i);
    //     }
    //     delete cookables_particle_system;
    // }

    return false;
}

cookable &game_scene::add_cookable(cookable &&cookable) {
    auto it = cookables.insert(cookables.end(), std::move(cookable));
    cookable_map[it->solid.rigid_dynamic] = it;
    return *it;
}

void game_scene::on_cookable_pan_contact(cookable &cookable, physx::PxContactPair const &pair) {
    cookable.cook(last_delta_time);

    // std::cout << "Cooked: " << cookable.current_cook_time << std::endl;
    // std::cout << "Previous: " << cookable.previous_cook_time << std::endl;
    // std::cout << "%: " << cookable.current_cook_time / cookable.cook_time << std::endl << std::endl;
}

void game_scene::on_cookable_ground_contact(cookable &cookable) {
    //std::cout << "Cooked: " << cookable.current_cook_time << std::endl;
    
    float cooking_ratio = cookable.current_cook_time / cookable.cook_time;
    if (cooking_ratio > 1.75f) {
        // TODO
    } else if (cooking_ratio > 0.8f) {
    } else if (cooking_ratio > 0.5f) {
    } else {
    }
    cookables_to_remove.push_back(cookable.solid.rigid_dynamic);

    size_t rnd = rand() % 3;
    switch (rnd) {
    case 0:
        scene->addActor(*add_cookable(cookable::create_egg(
            physics,
            physx::PxTransform(physx::PxVec3(0.0f, 10.0f, 5.0f)),
            physx::PxBoxGeometry(2.0f, 2.0f, 2.0f),
            10.0
        )).solid.rigid_dynamic);
        break;
    case 1:
        scene->addActor(*add_cookable(cookable::create_steak(
            physics,
            physx::PxTransform(physx::PxVec3(0.0f, 15.0f, 5.0f)),
            physx::PxBoxGeometry(2.0f, 1.0f, 2.0f),
            10.0
        )).solid.rigid_dynamic);
        break;
    case 2:
        scene->addActor(*add_cookable(cookable::create_fries(
            physics,
            physx::PxTransform(physx::PxVec3(0.0f, 20.0f, 5.0f)),
            physx::PxBoxGeometry(2.5f, 1.0f, 1.0f),
            10.0
        )).solid.rigid_dynamic);
        break;
    default:
        break;
    }
}

void game_scene::on_passive_mouse_motion(int x, int y) {
    float window_width = float(glutGet(GLUT_WINDOW_WIDTH));
    float window_height = float(glutGet(GLUT_WINDOW_HEIGHT));
    
    float uv_x = float(x) / window_width;
    float uv_y = float(y) / window_height;

    float ndc_x = uv_x * 2.0f - 1.0f;
    float ndc_y = 1.0f - uv_y * 2.0f;
    //std::cout << "NDC: " << ndc_x << ", " << ndc_y << std::endl;

    // rotate the pan based on the mouse ndc position
    // ndc y means the pan will rotate around the x axis
    // ndc x means the pan will rotate around the y axis

    auto pan_rb = frying_pan.pan_solid.rigid_dynamic;
    physx::PxTransform current_transform = pan_rb->getGlobalPose();
    physx::PxVec3 current_position = current_transform.p;
    physx::PxQuat current_rotation = current_transform.q;

    physx::PxVec3 current_direction = current_rotation.rotate({0.0f, 0.0f, 1.0f});
    physx::PxVec3 target_direction = physx::PxVec3(ndc_x, ndc_y, 1.0f).getNormalized();

    physx::PxVec3 axis = current_direction.cross(target_direction).getNormalized();
    double dot = current_direction.dot(target_direction);
    double angle = std::acos((std::min)(1.0, (std::max)(-1.0, dot)));

    physx::PxQuat rotation_quaternion = physx::PxQuat(angle, axis);
    physx::PxQuat new_rotation;

    if (reset_pan_rotation_requested) {
        new_rotation = physx::PxQuat(0.0f, {0.0f, 1.0f, 0.0f});
    } else {
        new_rotation = rotation_quaternion * current_rotation;
    }
    next_pan_rotation = new_rotation;
}

void game_scene::on_key_press(unsigned char key, const physx::PxTransform &camera) {
    switch (towlower(key)) {
    case ' ': {
        reset_pan_rotation_requested = true;
        // FIXME!
        // spawn_particle_burst(
        //     *pan_particle_system,
        //     2, 15,
        //     0.5f, 1.0f,
        //     0.01f, 0.1f,
        //     {0.15f, 0.05f, 0.25f},
        //     {0.15f, 0.15f, 0.35f}
        // );
        break;
    }
    default:
        break;
    }
}

void game_scene::on_key_release(unsigned char key) {
    switch (towlower(key)) {
    case ' ': {
        reset_pan_rotation_requested = false;
        break;
    }
    default:
        break;
    }
}

void game_scene::spawn_particle_burst(
    systems::particle_system &particle_system,
    size_t count_min,
    size_t count_max,
    objects::seconds_f64 lifetime_min,
    objects::seconds_f64 lifetime_max,
    objects::mass_f32 min_mass,
    objects::mass_f32 max_mass,
    types::v3_f32 const &colour_min,
    types::v3_f32 const &colour_max
) {
    size_t count = count_min + (rand() % (count_max - count_min));

    for (size_t i = 0; i < count; ++i) {
		float normalized_mass = std::uniform_real_distribution<float>(0.0f, 1.0f)(particle_system.generator.generator);
		objects::mass_f32 mass = min_mass + normalized_mass * (max_mass - min_mass);

		types::v3_f32 particle_position = {0.0f, 0.0f, 0.0f};
		size_t particle_id = particle_system.add_particle_random<objects::mass_particle>(
			[mass, &particle_position](objects::position3_f32 position, objects::velocity3_f32 velocity) {
				particle_position = position;
				return objects::mass_particle(objects::particle{position, {0.0f, 0.0f, 0.0f}}, mass);
			}, 0.5f, 1.0f
		);

		physx::PxTransform *&particle_transform = std::get<physx::PxTransform *&>(particle_system.set<physx::PxTransform *>(
			particle_id,
			new physx::PxTransform(particle_position)
		));

		types::v3_f32 colour = types::v3_f32::lerp(colour_min, colour_max, normalized_mass);
		RenderItem *particle_render_item = new RenderItem(CreateShape(physx::PxSphereGeometry(0.25f)), particle_transform, {
			colour.x,
			colour.y,
			colour.z,
			1.0f
		});
		particle_system.set<RenderItem *>(particle_id, std::move(particle_render_item));
    }
}

void game_scene::onConstraintBreak(physx::PxConstraintInfo *constraints, physx::PxU32 count) {
    (void)constraints;
    (void)count;
}

void game_scene::onWake(physx::PxActor **actors, physx::PxU32 count) {
    (void)actors;
    (void)count;
}

void game_scene::onSleep(physx::PxActor **actors, physx::PxU32 count) {
    (void)actors;
    (void)count;
}

void game_scene::onContact(const physx::PxContactPairHeader &pairHeader, const physx::PxContactPair *pairs, physx::PxU32 nbPairs) {
    
    auto actor0_it = cookable_map.find(pairHeader.actors[0]);
    auto actor1_it = cookable_map.find(pairHeader.actors[1]);
    if (actor0_it == cookable_map.end() && actor1_it == cookable_map.end()) {
        return;
    } else {
        auto cookable_it = actor0_it != cookable_map.end() ? actor0_it : actor1_it;

        if (pairHeader.actors[0] == frying_pan.pan_solid.rigid_dynamic
            || pairHeader.actors[1] == frying_pan.pan_solid.rigid_dynamic) {
            for (physx::PxU32 i = 0; i < nbPairs; ++i) {
                const physx::PxContactPair &pair = pairs[i];
                if (pair.shapes[0] == frying_pan.base_shape
                    || pair.shapes[1] == frying_pan.base_shape) {
                    on_cookable_pan_contact(*cookable_it->second, pair);
                }
            }
        } else if (pairHeader.actors[0] == ground.rigid_static
            || pairHeader.actors[1] == ground.rigid_static) {
            on_cookable_ground_contact(*cookable_it->second);
        }

    }
}

void game_scene::onTrigger(physx::PxTriggerPair *pairs, physx::PxU32 count) {
    (void)pairs;
    (void)count;
}

void game_scene::onAdvance(const physx::PxRigidBody *const *bodyBuffer, const physx::PxTransform *poseBuffer, const physx::PxU32 count) {
    (void)bodyBuffer;
    (void)poseBuffer;
    (void)count;
}
