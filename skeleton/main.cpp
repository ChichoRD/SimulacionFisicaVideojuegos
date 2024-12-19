#include <ctype.h>

#include <PxPhysicsAPI.h>

#include <vector>

#include "core.hpp"
#include "RenderUtils.hpp"
#include "callbacks.hpp"

#include <iostream>

#include <cassert>
#include <cstdlib>
#include "types/v3_f32.hpp"
#include "objects/particle.hpp"
#include "objects/mass_particle.hpp"
#include "objects/projectile.hpp"
//#include "systems/particle_storage.hpp"
#include "systems/particle_system.hpp"
//#include "systems/particle_generator.hpp"

#include "systems/force_composer.hpp"
#include "generators/gravity_generator.hpp"
#include "generators/wind_generator.hpp"
#include "generators/explosion_generator.hpp"
#include "generators/spring/spring_force_generator.hpp"
#include "generators/spring/buoyancy_generator.hpp"
#include "objects/solid_particle.hpp"
#include "gameplay/pan.hpp"

std::string display_text = "This is a test";


using namespace physx;

PxDefaultAllocator		gAllocator;
PxDefaultErrorCallback	gErrorCallback;

PxFoundation*			gFoundation = NULL;
PxPhysics*				gPhysics	= NULL;


PxMaterial*				gMaterial	= NULL;

PxPvd*                  gPvd        = NULL;

PxDefaultCpuDispatcher*	gDispatcher = NULL;
PxScene*				gScene      = NULL;
ContactReportCallback gContactReportCallback;

systems::particle_system particle_system = systems::particle_system(
	systems::particle_generator(
		systems::particle_generator::UNIFORM,
		systems::particle_generator::BOX,
		systems::particle_generator::generation_volume(
			systems::generation_box{ 
				types::v3_f32{0.0f, 0.0f, 0.0f},
				types::v3_f32{8.0f, 8.0f, 8.0f}
			}
		)
	),
	10.0f * 1000.0f
);

generators::wind_generator wind_generator = generators::wind_generator(
	types::v3_f32(0.0f, 0.0f, 0.0f),
	100.0f,
	types::v3_f32(-10.0f, 0.0f, 0.0f),
	1.0f,
	0.025f
);

generators::gravity_generator gravity_generator = generators::gravity_generator(
	types::v3_f32(0.0f, -generators::gravity_generator::earth_radius, 0.0f),
	generators::gravity_generator::earth_mass
);

generators::tornado_generator tornado_generator = generators::tornado_generator(
	types::v3_f32(0.0f, -5.0f, 0.0f),
	types::v3_f32(0.0f, 1.0f, 0.0f),
	100.0f,
	50.0f,
	10.0f	
);

generators::explosion_generator explosion_generator = generators::explosion_generator(
	types::v3_f32(0.0f, 0.0f, 0.0f),
	1500.0f,
	50.0f,
	4.0f
);
bool in_explosion_time = false;

generators::static_spring_force_generator static_spring_force_generator =
	generators::static_spring_force_generator(100.0f);
generators::dynamic_spring_force_generator dynamic_spring_force_generator =
	generators::dynamic_spring_force_generator(100.0f);

types::f32 buoyancy_gravity = 9.8f;
generators::buoyancy_generator buoyancy_generator = generators::buoyancy_generator(
	objects::position3_f32(0.0f, -15.0f, 0.0f),
	types::v3_f32(0.0f, 15.0f, 0.0f),
	1000.0f,
	1.0f
);

struct combined_generator {
	generators::gravity_generator gravity_generator;
	generators::wind_generator wind_generator;
	generators::tornado_generator tornado_generator;
	generators::explosion_generator explosion_generator;

	generators::static_spring_force_generator static_spring_force_generator;
	generators::dynamic_spring_force_generator dynamic_spring_force_generator;
	generators::buoyancy_generator buoyancy_generator;

	void apply_to_particles(systems::particle_system &particle_system, objects::seconds_f64 delta_time) {
		gravity_generator.apply_to_particles(particle_system, delta_time);
		wind_generator.apply_to_particles(particle_system, delta_time);
		
		//tornado_generator.apply_to_particles(particle_system, delta_time);
		explosion_generator.apply_to_particles(particle_system, delta_time);

		// static_spring_force_generator.apply_to_particles(particle_system, delta_time);
		// dynamic_spring_force_generator.apply_to_particles(particle_system, delta_time);
		buoyancy_generator.apply_to_particles(particle_system, delta_time);
	}
};

RenderItem *origin_render_item = NULL;
PxTransform origin_transform;

RenderItem* positive_x_render_item = NULL;
PxTransform positive_x_transform;

RenderItem* positive_y_render_item = NULL;
PxTransform positive_y_transform;

RenderItem* positive_z_render_item = NULL;
PxTransform positive_z_transform;

std::vector<objects::projectile> projectiles;
std::vector<PxTransform> projectile_transforms;
std::vector<RenderItem *> projectile_render_items;

enum projectile_type {
	BULLET,
	FIRE_BALL,
	GAMING_MOUSE,
	PINEAPPLE
};

static objects::projectile create_projectile(objects::position3_f32 position, types::v3_f32 direction, projectile_type type) {
	constexpr types::f32 speed_scale_factor = 0.9f;
	switch (type) {
	case BULLET:
		return objects::projectile(0.03f, position, direction * 260, speed_scale_factor);
	case FIRE_BALL:
		return objects::projectile(0.0013f, position, direction * 24.38, speed_scale_factor * 3.0f);
	case GAMING_MOUSE:
		return objects::projectile(0.1f, position, direction * 17.88, speed_scale_factor * 2.0f);
	case PINEAPPLE:
		return objects::projectile(1.36078f, position, direction * 17.88, speed_scale_factor * 2.0f);
	default: {
		assert(false && "unreachable: invalid projectile type");
		std::exit(EXIT_FAILURE);
		return objects::projectile();
	}
	}
}

static objects::projectile create_projectile_from_camera(Camera const& camera, projectile_type type) {
	return create_projectile(
		camera.getEye(),
		-camera.getDir(),
		type
	);
}
static objects::projectile create_projectile_from_camera(PxTransform const& camera, projectile_type type) {
	return create_projectile(
		camera.p,
		-camera.rotate(positive_z_transform.p).getNormalized(),
		type
	);
}

typedef size_t proyectile_count;
static proyectile_count register_projectile(objects::projectile projectile, types::v3_f32 color, types::f32 scale) {
	projectiles.push_back(projectile);
	projectile_transforms.push_back(PxTransform(projectile.particle.particle.position));
	projectile_render_items.push_back(
		new RenderItem(CreateShape(PxSphereGeometry(scale)),
		&projectile_transforms.back(),
		Vector4(color, 1.0f))
	);
	return projectiles.size();
}

static void projectile_appearance(projectile_type type, types::v3_f32& out_color, types::f32 &out_scale) {
	constexpr types::f32 meters_to_world_factor = 10.0f;
	switch (type)
	{
	case BULLET: {
		out_color = { 0.35, 0.35, 0.1 };
		out_scale = 0.07035f * meters_to_world_factor;
		return;
	}
	case FIRE_BALL: {
		out_color = { 0.95, 0.3, 0.1 };
		out_scale = 0.235f * meters_to_world_factor;
		return;
	}
	case GAMING_MOUSE: {
		out_color = { 0.025, 0.075, 0.1 };
		out_scale = 0.12f * meters_to_world_factor;
		return;
	}
	case PINEAPPLE: {
		out_color = { 0.8, 0.8, 0.1 };
		out_scale = 0.27f * meters_to_world_factor;
		return;
	}
	default: {
		assert(false && "unreachable: invalid projectile type");
		std::exit(EXIT_FAILURE);
		return;
	}
	}
}

typedef size_t projectile_index;
static projectile_index instantiate_projectile(Camera const& camera, projectile_type type) {
	objects::projectile p = create_projectile_from_camera(camera, type);

	types::v3_f32 color;
	types::f32 size;
	projectile_appearance(type, color, size);
	return register_projectile(p, color, size) - 1;
}

static projectile_index instantiate_projectile(PxTransform const& camera, projectile_type type) {
	objects::projectile p = create_projectile_from_camera(camera, type);

	types::v3_f32 color;
	types::f32 size;
	projectile_appearance(type, color, size);
	return register_projectile(p, color, size) - 1;
}

struct world {
	pan frying_pan;
} *g_world;

bool init_sample_physics(PxScene *&out_scene, PxDefaultCpuDispatcher *&out_dispatcher) {
	assert(gPhysics && "error: physics must be initialized before calling init_sample_physics");
	
	PxSceneDesc sceneDesc(gPhysics->getTolerancesScale());
	sceneDesc.gravity = PxVec3(0.0f, -9.81f, 0.0f);

	out_dispatcher = PxDefaultCpuDispatcherCreate(2);
	sceneDesc.cpuDispatcher = out_dispatcher;
	sceneDesc.filterShader = contactReportFilterShader;
	sceneDesc.simulationEventCallback = &gContactReportCallback;

	out_scene = gPhysics->createScene(sceneDesc);
	return true;
}

size_t spawn_sample_particles() {
	constexpr objects::mass_f32 const min_mass = 0.1f;
	constexpr objects::mass_f32 const max_mass = 10.0f;

	{	
		constexpr size_t const particle_count = 100;
		for (size_t i = 0; i < particle_count; ++i) {
			float normalized_mass = std::uniform_real_distribution<float>(0.0f, 1.0f)(particle_system.generator.generator);
			objects::mass_f32 mass = min_mass + normalized_mass * (max_mass - min_mass);

			types::v3_f32 particle_position = {0.0f, 0.0f, 0.0f};
			size_t particle_id = particle_system.add_particle_random<objects::mass_particle>(
				[mass, &particle_position](objects::position3_f32 position, objects::velocity3_f32 velocity) {
					particle_position = position;
					return objects::mass_particle(objects::particle{position, {0.0f, 0.0f, 0.0f}}, mass);
				}, 0.5f, 1.0f
			);

			PxTransform *&particle_transform = std::get<PxTransform *&>(particle_system.set<PxTransform *>(
				particle_id,
				new PxTransform(particle_position)
			));

			static_spring_force_generator.add_anchor<8>(particle_system, particle_id, objects::position3_f32{
				0.0f, 100.0f, 1.0f
			});
		}
	}

	particle_system.iter_indexed<
		objects::mass_particle::deconstruct_inverse_mass const,
		PxTransform *
	>(
		[min_mass, max_mass](systems::particle_id id,
			objects::mass_particle::deconstruct_inverse_mass const &inverse_mass,
			PxTransform *&particle_transform) {
			float mass = 1.0f / inverse_mass.value;
			float normalized_mass = (mass - min_mass) / (max_mass - min_mass);

			types::v3_f32 low_mass_colour = { 0.15f, 0.05f, 0.95f };
			types::v3_f32 high_mass_colour = { 0.95f, 0.15f, 0.35f };
			types::v3_f32 colour = types::v3_f32::lerp(low_mass_colour, high_mass_colour, normalized_mass);
			RenderItem *particle_render_item = new RenderItem(CreateShape(PxSphereGeometry(1.0f)), particle_transform, {
				colour.x,
				colour.y,
				colour.z,
				1.0f
			});
			particle_system.set<RenderItem *>(id, std::move(particle_render_item));
		}
	);

	{
		constexpr size_t const solid_particle_count = 200;
		for (size_t i = 0; i < solid_particle_count; ++i) {
			float normalized_mass = std::uniform_real_distribution<float>(0.0f, 1.0f)(particle_system.generator.generator);
			objects::mass_f32 mass = min_mass + normalized_mass * (max_mass - min_mass);

			types::v3_f32 low_mass_colour = { 0.15f, 0.05f, 0.95f };
			types::v3_f32 high_mass_colour = { 0.95f, 0.15f, 0.35f };
			types::v3_f32 colour = types::v3_f32::lerp(low_mass_colour, high_mass_colour, normalized_mass);

			types::v3_f32 particle_position = {0.0f, 0.0f, 0.0f};
			size_t particle_id = particle_system.add_particle_random<objects::solid_dynamic_particle>(
				[mass, colour](objects::position3_f32 position, objects::velocity3_f32 velocity) {

					types::v3_f32 size = velocity.abs() * 0.5f;
					types::f32 mass_factor = mass / 12.0f;
					auto solid = objects::solid_dynamic_particle(
						*gPhysics,
						PxTransform(position),
						*gMaterial,
						PxBoxGeometry(size * 0.5f),
						PxVec4(colour, 1.0f),
						mass_factor * types::v3_f32{
							size.y * size.y + size.z * size.z,
							size.x * size.x + size.z * size.z,
							size.x * size.x + size.y * size.y
						}
					);
					gScene->addActor(*solid.rigid_dynamic);
					
					//disable gravity
					solid.rigid_dynamic->setActorFlag(PxActorFlag::eDISABLE_GRAVITY, true);
					
					return solid;
				}, 0.5f, 1.0f
			);
		}
	}

	return particle_system.alive_particle_count();
}

// Initialize physics engine
void initPhysics(bool interactive)
{
	PX_UNUSED(interactive);

	gFoundation = PxCreateFoundation(PX_FOUNDATION_VERSION, gAllocator, gErrorCallback);

	gPvd = PxCreatePvd(*gFoundation);
	PxPvdTransport* transport = PxDefaultPvdSocketTransportCreate(PVD_HOST, 5425, 10);
	gPvd->connect(*transport,PxPvdInstrumentationFlag::eALL);

	gPhysics = PxCreatePhysics(PX_PHYSICS_VERSION, *gFoundation, PxTolerancesScale(),true,gPvd);

	gMaterial = gPhysics->createMaterial(0.5f, 0.5f, 0.6f);

	assert(init_sample_physics(gScene, gDispatcher) && "fatal error: failed to initialize sample physics");
	
	// axis:
	using namespace types;
	//gScene.
	const f32 spacing = 10.0f;
	origin_transform = PxTransform(v3_f32(0.0f, 0.0f, 0.0f));
	positive_x_transform = PxTransform(v3_f32(spacing, 0.0f, 0.0f));
	positive_y_transform = PxTransform(v3_f32(0.0f, spacing, 0.0f));
	positive_z_transform = PxTransform(v3_f32(0.0f, 0.0f, spacing));

	const Vector4 color_origin = { v3_f32(1.0f, 1.0f, 1.0f), 1.0 };
	const Vector4 color_x = { v3_f32(1.0f, 0.0f, 0.0f), 1.0 };
	const Vector4 color_y = { v3_f32(0.0f, 1.0f, 0.0f), 1.0 };
	const Vector4 color_z = { v3_f32(0.0f, 0.0f, 1.0f), 1.0 };
	const Vector4 color_particle = { v3_f32(1.0f, 1.0f, 0.0f), 1.0f };

	const f32 size = 1.0f;
	origin_render_item = new RenderItem(CreateShape(PxSphereGeometry(size)), &origin_transform, color_origin);
	positive_x_render_item = new RenderItem(CreateShape(PxSphereGeometry(size)), &positive_x_transform, color_x);
	positive_y_render_item = new RenderItem(CreateShape(PxSphereGeometry(size)), &positive_y_transform, color_y);
	positive_z_render_item = new RenderItem(CreateShape(PxSphereGeometry(size)), &positive_z_transform, color_z);

	//std::cout << spawn_sample_particles() << " particles spawned" << std::endl;
	auto ground = objects::solid_static_particle(
		*gPhysics,
		PxTransform(PxVec3(0.0f, -10.0f, 0.0f)),
		*gMaterial,
		PxBoxGeometry(100.0f, 1.0f, 100.0f),
		PxVec4(0.5f, 0.5f, 0.5f, 1.0f)
	);
	gScene->addActor(*ground.rigid_static);
	particle_system.add_particle<objects::solid_static_particle>(ground);

	g_world = new world{ pan(*gPhysics, PxTransform(PxVec3(10.0f, 5.0f, 10.0f))) };
	gScene->addActor(*g_world->frying_pan.pan_solid.rigid_dynamic);
}


// Function to configure what happens in each step of physics
// interactive: true if the game is rendering, false if it offline
// t: time passed since last call in milliseconds
// ^^^^ the above comment is lying!! dt is in seconds!
void stepPhysics(bool interactive, double t)
{
	PX_UNUSED(interactive);

	objects::acceleration3_f32 g = { 0, -9.8f, 0 };
	for (size_t i = 0; i < projectiles.size(); ++i) {

		if (projectiles[i].particle.particle.position.y > 0) {
			projectiles[i].integrate(g, objects::acceleration3_f32(), 0.9875f, t);
			projectiles[i].particle.particle >> projectile_transforms[i];
		}
	}

	if (in_explosion_time) {
		auto force_composer = systems::force_composer<combined_generator>(combined_generator{
			gravity_generator,
			wind_generator,
			tornado_generator,
			explosion_generator,
			static_spring_force_generator,
			dynamic_spring_force_generator,
			buoyancy_generator
		});

		//update
		force_composer.apply_to_particles(particle_system, t);
		force_composer.compose_forces(particle_system, t);
	}

	particle_system.iter_indexed<
		objects::particle::deconstruct_position const,
		PxTransform *
	>(
		[](systems::particle_id id,
			objects::particle::deconstruct_position const &position, PxTransform *&particle_transform) {
			particle_transform->p = physx::PxVec3(position.x, position.y, position.z);
		}
	);

	gScene->simulate(t);
	PxU32 errorState = 0;
	gScene->fetchResults(true, &errorState);
	assert(errorState == 0 && "error: physics simulation failed");
}

// Function to clean data
// Add custom code to the begining of the function
void cleanupPhysics(bool interactive)
{
	PX_UNUSED(interactive);

	// axis:
	DeregisterRenderItem(positive_z_render_item);
	DeregisterRenderItem(positive_y_render_item);
	DeregisterRenderItem(positive_x_render_item);
	DeregisterRenderItem(origin_render_item);
	delete positive_z_render_item;
	delete positive_y_render_item;
	delete positive_x_render_item;
	delete origin_render_item;

	for (RenderItem *r : projectile_render_items) {
		DeregisterRenderItem(r);
		delete r;
	}
	
	particle_system.iter<
		RenderItem *
	>(
		[](RenderItem *r) {
			DeregisterRenderItem(r);
			delete r;
		}
	);

	for (size_t i = 0; i < particle_system.particles.particle_count(); ++i) {
		particle_system.remove_particle(i);
	}

	delete g_world;

	// Rigid Body ++++++++++++++++++++++++++++++++++++++++++
	gScene->release();
	gDispatcher->release();
	// -----------------------------------------------------
	gPhysics->release();	
	PxPvdTransport* transport = gPvd->getTransport();
	gPvd->release();
	transport->release();
	
	gFoundation->release();
}

// Function called when a key is pressed
void keyPress(unsigned char key, const PxTransform& camera)
{
	PX_UNUSED(camera);

	switch(toupper(key))
	{
	case 'B':
	{
		instantiate_projectile(camera, projectile_type::BULLET);
		break;
	}
	case 'F':
	{
		instantiate_projectile(camera, projectile_type::FIRE_BALL);
		break;
	}
	case 'G':
	{
		instantiate_projectile(camera, projectile_type::GAMING_MOUSE);
		break;
	}
	case 'P':
	{
		instantiate_projectile(camera, projectile_type::PINEAPPLE);
		break;
	}
	case 'E': {
		in_explosion_time = !in_explosion_time;
		break;
	}
	default:
		break;
	}
}

void onCollision(physx::PxActor* actor1, physx::PxActor* actor2)
{
	PX_UNUSED(actor1);
	PX_UNUSED(actor2);
}


int main(int, const char*const*)
{
//#define OFFLINE_EXECUTION
#ifndef OFFLINE_EXECUTION 
	extern void renderLoop();
	renderLoop();
#else
	static const PxU32 frameCount = 100;
	initPhysics(false);
	for(PxU32 i=0; i<frameCount; i++)
		stepPhysics(false);
	cleanupPhysics(false);
#endif

	return 0;
}