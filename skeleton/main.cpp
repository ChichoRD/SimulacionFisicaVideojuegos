#include <ctype.h>

#include <PxPhysicsAPI.h>

#include <vector>

#include "core.hpp"
#include "RenderUtils.hpp"
#include "callbacks.hpp"

#include <iostream>

#include <cassert>
#include "types/v3_f32.hpp"
#include "objects/particle.hpp"
#include "objects/mass_particle.hpp"
#include "objects/projectile.hpp"
#include "systems/particle_system.hpp"

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


	// axis:
	using namespace types;
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

	const f32 size = 2.0f;
	origin_render_item = new RenderItem(CreateShape(PxSphereGeometry(size)), &origin_transform, color_origin);
	positive_x_render_item = new RenderItem(CreateShape(PxSphereGeometry(size)), &positive_x_transform, color_x);
	positive_y_render_item = new RenderItem(CreateShape(PxSphereGeometry(size)), &positive_y_transform, color_y);
	positive_z_render_item = new RenderItem(CreateShape(PxSphereGeometry(size)), &positive_z_transform, color_z);

	RegisterRenderItem(origin_render_item);
	RegisterRenderItem(positive_x_render_item);
	RegisterRenderItem(positive_y_render_item);
	RegisterRenderItem(positive_z_render_item);

	systems::particle_system s = systems::particle_system();
	objects::acceleration3_f32 &stored =
		s.set_particle_attribute<objects::acceleration3_f32>(2, objects::acceleration3_f32(0, 1, 3));
	objects::acceleration3_f32 got = s.get_particle_attribute<objects::acceleration3_f32>(2);
	objects::acceleration3_f32 removed;
	std::cout << "watashi" << std::endl;
	//bool had = s.remove_particle_attribute<objects::acceleration3_f32>(2, removed);
	bool has = s.particle_has_attribute<objects::acceleration3_f32>(2);

	size_t count = s.iter<objects::acceleration3_f32 const>
		([](objects::acceleration3_f32 const &acc) {
		std::cout << acc.z << std::endl;
	});

	objects::mass_particle const p = { {{ 4, 2, 0 }, { 0, 6, 9 }}, 6.5 };
	auto att = s.set_particle_attributes_deconstruct(4, p);

	objects::particle r = s.get_particle_attributes_construct<objects::particle>(4);

	// For Solid Rigids +++++++++++++++++++++++++++++++++++++
	PxSceneDesc sceneDesc(gPhysics->getTolerancesScale());
	sceneDesc.gravity = PxVec3(0.0f, -9.8f, 0.0f);
	gDispatcher = PxDefaultCpuDispatcherCreate(2);
	sceneDesc.cpuDispatcher = gDispatcher;
	sceneDesc.filterShader = contactReportFilterShader;
	sceneDesc.simulationEventCallback = &gContactReportCallback;
	gScene = gPhysics->createScene(sceneDesc);
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

	gScene->simulate(t);
	gScene->fetchResults(true);
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

	for (RenderItem *r : projectile_render_items) {
		DeregisterRenderItem(r);
		delete r;
	}


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