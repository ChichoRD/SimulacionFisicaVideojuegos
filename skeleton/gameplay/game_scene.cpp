#include "game_scene.hpp"
#include <cassert>
#include <iostream>

#include "../systems/force_composer.hpp"

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

static game_scene::combined_generator create_default_pan_force_generator() {
    return game_scene::combined_generator{
        generators::gravity_generator(
            types::v3_f32(0.0f, -generators::gravity_generator::earth_radius, 0.0f),
            generators::gravity_generator::earth_mass
        ),
        generators::wind_generator(
            types::v3_f32(0.0f, 0.0f, 0.0f),
            50.0f,
            types::v3_f32(-2.0f, 0.0f, 0.0f),
            1.0f,
            0.025f
        ),
        generators::tornado_generator(
            types::v3_f32(0.0f, -5.0f, 0.0f),
            types::v3_f32(0.0f, 1.0f, 0.0f),
            50.0f,
            20.0f,
            0.15f
        ),
        generators::buoyancy_generator(
            objects::position3_f32(0.0f, 0.0f, 0.0f),
            types::v3_f32(0.0f, 50.0f, 0.0f),
            1000.0f,
            0.0350f
        )
    };
}

static systems::particle_generator create_deafault_pan_distribution_generator(systems::generation_box const &pan_box) {
    return systems::particle_generator(
        systems::particle_generator::NORMAL,
        systems::particle_generator::BOX,
        systems::particle_generator::generation_volume(
            pan_box
        )
    );
}

game_scene::game_scene(physx::PxPhysics &physics)
    : physics(physics),
    scene(nullptr),
    last_delta_time(0.0),
    next_pan_rotation(physx::PxQuat(0.0f, physx::PxVec3(0.0f, 0.0f, 1.0f))),
    last_cookable_contact(),
    last_cookable_contact_valid(false),
    pan_generator(create_default_pan_force_generator()),
    pan_particle_system(
        create_deafault_pan_distribution_generator(systems::generation_box{}), 0.0f
    ),
    cookable_generator(create_default_pan_force_generator()),
    cookables_particle_system(
        create_deafault_pan_distribution_generator(systems::generation_box{}), 0.0f
    ) {
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

    spawn_cookable();
    spawn_cookable();
    spawn_cookable();
    
    return true;
}

bool game_scene::update(objects::seconds_f64 delta_time) {
    last_delta_time = delta_time;
    time += delta_time;

    physx::PxBoxGeometry cookable_purge_box = physx::PxBoxGeometry(physx::PxVec3(80.0f, 60.0f, 80.0f));
    for (auto it = cookables.begin(); it != cookables.end(); ++it) {
        auto pose = it->solid.rigid_dynamic->getGlobalPose().p;
        if (
            pose.x < -cookable_purge_box.halfExtents.x || pose.x > cookable_purge_box.halfExtents.x
            || pose.y < -cookable_purge_box.halfExtents.y || pose.y > cookable_purge_box.halfExtents.y
            || pose.z < -cookable_purge_box.halfExtents.z || pose.z > cookable_purge_box.halfExtents.z
        ) {
            cookables_to_remove.push_back(it->solid.rigid_dynamic);
            spawn_cookable();
        }
        else if (it->current_cook_time >= it->cook_time * 2.0f) {
            cookables_to_remove.push_back(it->solid.rigid_dynamic);
            spawn_cookable();
        }
    }

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

    pan_particle_system.generator.volume.box = systems::generation_box{ 
        frying_pan.pan_solid.rigid_dynamic->getGlobalPose().transform(frying_pan.base_shape->getLocalPose()).p,
        frying_pan.base_shape->getGeometry().box().halfExtents * 0.5f
    };

    // change with a sin wave from left to right
    cookable_generator.wind.wind_velocity =
        types::v3_f32{8.0f, 0.0f, 0.0f} * std::sin(0.15f * time);

    purge_particles(pan_particle_system);
    purge_particles(cookables_particle_system);

    handle_particle_tasks(pan_particle_system, particle_task::PAN_FRYING | particle_task::COOKABLE_COOKED | particle_task::COOKABLE_BURNT);
    if (last_cookable_contact_valid) {
        cookables_particle_system.generator.volume.box = systems::generation_box{
            last_cookable_contact,
            {1.5f, 0.5f, 1.5f}
        };
        handle_particle_tasks(cookables_particle_system, particle_task::COOKABLE_COOKING | particle_task::COOKABLE_BURNING);

        last_cookable_contact_valid = false;
    }

    auto pan_force_composer = systems::force_composer<game_scene::combined_generator>(pan_generator);
    pan_force_composer.apply_to_particles(pan_particle_system, delta_time);
    pan_force_composer.compose_forces(pan_particle_system, delta_time);

    auto cookable_force_composer = systems::force_composer<game_scene::combined_generator>(cookable_generator);
    cookable_force_composer.apply_to_particles(cookables_particle_system, delta_time);
    cookable_force_composer.compose_forces(cookables_particle_system, delta_time);

    pan_particle_system.iter<
        objects::particle::deconstruct_position const,
        physx::PxTransform *
    >(
        [](objects::particle::deconstruct_position const &position, physx::PxTransform *&particle_transform) {
            particle_transform->p = physx::PxVec3(position.x, position.y, position.z);
        }
    );

    cookables_particle_system.iter<
        objects::particle::deconstruct_position const,
        physx::PxTransform *
    >(
        [](objects::particle::deconstruct_position const &position, physx::PxTransform *&particle_transform) {
            particle_transform->p = physx::PxVec3(position.x, position.y, position.z);
        }
    );

    scene->simulate(delta_time);
	physx::PxU32 error_state = 0;
	scene->fetchResults(true, &error_state);

    return error_state == 0;
}

bool game_scene::shutdown() {
    {
        pan_particle_system.iter<RenderItem *>(
            [](RenderItem *r) {
                DeregisterRenderItem(r);
                delete r;
            }
        );

        for (size_t i = 0; i < pan_particle_system.particles.particle_count(); ++i) {
            pan_particle_system.remove_particle(i);
        }
    }

    {
        cookables_particle_system.iter<RenderItem *>(
            [](RenderItem *r) {
                DeregisterRenderItem(r);
                delete r;
            }
        );

        for (size_t i = 0; i < cookables_particle_system.particles.particle_count(); ++i) {
            cookables_particle_system.remove_particle(i);
        }
    }

    return false;
}

cookable &game_scene::add_cookable(cookable &&cookable) {
    auto it = cookables.insert(cookables.end(), std::move(cookable));
    cookable_map[it->solid.rigid_dynamic] = it;
    return *it;
}

void game_scene::on_cookable_pan_contact(cookable &cookable, physx::PxContactPair const &pair) {
    cookable.cook(last_delta_time);

    float previous_cooking_ratio = cookable.previous_cook_time / cookable.cook_time;
    float cooking_ratio = cookable.current_cook_time / cookable.cook_time;
    current_particle_task |= PAN_FRYING;
    
    if (cooking_ratio < 1.0f && previous_cooking_ratio < 1.0f) {
        current_particle_task |= COOKABLE_COOKING;
    } else if (cooking_ratio >= 1.0f && previous_cooking_ratio < 1.0f) {
        current_particle_task |= COOKABLE_COOKED;
    } else if (cooking_ratio >= 1.25f && previous_cooking_ratio >= 1.0f) {
        current_particle_task |= COOKABLE_BURNING;
    } else if (cooking_ratio >= 1.5f && previous_cooking_ratio < 1.5f) {
        current_particle_task |= COOKABLE_BURNT;
    }

    last_cookable_contact = cookable.solid.rigid_dynamic->getGlobalPose().p;
    last_cookable_contact_valid = true;
    // std::cout << "Cooked: " << cookable.current_cook_time << std::endl;
    // std::cout << "Previous: " << cookable.previous_cook_time << std::endl;
    // std::cout << "%: " << cookable.current_cook_time / cookable.cook_time << std::endl << std::endl;
}

void game_scene::on_cookable_ground_contact(cookable &cookable) {
    //std::cout << "Cooked: " << cookable.current_cook_time << std::endl;
    
    //float cooking_ratio = cookable.current_cook_time / cookable.cook_time;
    cookables_to_remove.push_back(cookable.solid.rigid_dynamic);
    spawn_cookable();
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

void game_scene::handle_particle_tasks(systems::particle_system &particle_system, size_t mask) {
    size_t masked_task = current_particle_task & mask;
    if (masked_task & PAN_FRYING) {
        constexpr float fry_threshold = 0.3f;
        if (std::uniform_real_distribution<float>(0.0f, 1.0f)(particle_system.generator.generator) < fry_threshold) {
            spawn_particle_burst(
                particle_system,
                1, 4,
                0.2f, 0.35f,
                0.1f, 0.75f,

                // from darkish yellow to reddish yellow
                types::v3_f32{0.15f, 0.15f, 0.00f} * 1.50f,
                types::v3_f32{0.45f, 0.35f, 0.075f} * 1.50f
            );

            current_particle_task &= ~PAN_FRYING;
        }
    }

    if (masked_task & COOKABLE_COOKING) {
        constexpr float cook_threshold = 0.3f;
        if (std::uniform_real_distribution<float>(0.0f, 1.0f)(particle_system.generator.generator) < cook_threshold) {
            spawn_particle_burst(
                particle_system,
                1, 3,
                0.25f, 0.55f,
                0.25f, 2.0f,

                // from oily yellow to brown
                {0.95f, 0.35f, 0.05f},
                {0.675f, 0.25f, 0.15f}
            );

            current_particle_task &= ~COOKABLE_COOKING;
        }
    }

    if (masked_task & COOKABLE_COOKED) {
        spawn_particle_burst(
            particle_system,
            5, 10,
            0.75f, 1.5f,
            0.1f, 0.25f,

            // light colour yellow tones indicating success
            {0.65f, 0.35f, 0.05f},
            {0.95f, 0.65f, 0.075f}
        );

        current_particle_task &= ~COOKABLE_COOKED;
    }

    if (masked_task & COOKABLE_BURNING) {
        constexpr float burn_threshold = 0.3f;
        if (std::uniform_real_distribution<float>(0.0f, 1.0f)(particle_system.generator.generator) < burn_threshold) {
            spawn_particle_burst(
                particle_system,
                2, 8,
                0.35f, 0.65f,
                0.1f, 0.15f,

                // from darkish yellow to black
                {0.0f, 0.05f, 0.05f},
                {0.35f, 0.15f, 0.25f}
            );

            current_particle_task &= ~COOKABLE_BURNING;
        }
    }

    if (masked_task & COOKABLE_BURNT) {
        spawn_particle_burst(
            particle_system,
            4, 16,
            0.1f, 0.25f,
            0.1f, 1.0f,

            // black purple
            {0.0f, 0.0f, 0.0f},
            {0.15f, 0.0f, 0.35f}
        );

        current_particle_task &= ~COOKABLE_BURNT;
    }
}

void game_scene::purge_particles(systems::particle_system &particle_system)
{
    // if they are too far

    const physx::PxBoxGeometry purge_box(150.0f, 150.0f, 150.0f);
    particle_system.iter_indexed<
        objects::particle::deconstruct_position const,
        RenderItem *
    >(
        [&particle_system, purge_box](systems::particle_id id,
            objects::particle::deconstruct_position const &position,
            RenderItem *&render_item) {
            if (position.x < -purge_box.halfExtents.x || position.x > purge_box.halfExtents.x
                || position.y < -purge_box.halfExtents.y || position.y > purge_box.halfExtents.y
                || position.z < -purge_box.halfExtents.z || position.z > purge_box.halfExtents.y) {
                DeregisterRenderItem(render_item);
                delete render_item;
                particle_system.remove_particle(id);
            }
        }
    );
}

void game_scene::spawn_cookable(cookable_type type) {
    size_t rnd = type == cookable_type::COOKABLE_TYPE_NONE
        ? std::uniform_int_distribution<size_t>(cookable_type::FIRST, cookable_type::LAST)(pan_particle_system.generator.generator)
        : type;

    physx::PxVec3 centre = frying_pan.pan_solid.rigid_dynamic->getGlobalPose()
        .transform(frying_pan.base_shape->getLocalPose()).p
        + physx::PxVec3(0.0f, 10.0f, 0.0f);
    physx::PxVec3 extents = frying_pan.base_shape->getGeometry().box().halfExtents * 0.5f;
    systems::generation_box pan_box = {centre, extents};

    physx::PxVec3 position = pan_box.random_uniform(pan_particle_system.generator.generator);
    switch (rnd) {
    case cookable_type::EGG:
        scene->addActor(*add_cookable(cookable::create_egg(
            physics,
            physx::PxTransform(position),
            physx::PxBoxGeometry(2.0f, 2.0f, 2.0f),
            15.0
        )).solid.rigid_dynamic);
        break;
    case cookable_type::STEAK:
        scene->addActor(*add_cookable(cookable::create_steak(
            physics,
            physx::PxTransform(position),
            physx::PxBoxGeometry(2.0f, 1.0f, 2.0f),
            20.0
        )).solid.rigid_dynamic);
        break;
    case cookable_type::FRIES:
        scene->addActor(*add_cookable(cookable::create_fries(
            physics,
            physx::PxTransform(position),
            physx::PxBoxGeometry(2.5f, 1.0f, 1.0f),
            10.0
        )).solid.rigid_dynamic);
        break;
    default:
        break;
    }
}

void game_scene::spawn_particle_burst(
    systems::particle_system &particle_system,
    size_t count_min, size_t count_max,
    objects::f32 size_min, objects::f32 size_max,
    objects::mass_f32 min_mass, objects::mass_f32 max_mass,
    types::v3_f32 const &colour_min, types::v3_f32 const &colour_max)
{
    size_t count = std::uniform_int_distribution<size_t>(count_min, count_max)(particle_system.generator.generator);

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
        types::f32 size = size_min + normalized_mass * (size_max - size_min);
		RenderItem *particle_render_item = new RenderItem(
            CreateShape(physx::PxSphereGeometry(size)), particle_transform, physx::PxVec4(colour, 1.0f)
        );
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
