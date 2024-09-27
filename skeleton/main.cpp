#include <ctype.h>

#include <PxPhysicsAPI.h>

#include <vector>

#include "core.hpp"
#include "RenderUtils.hpp"
#include "callbacks.hpp"

#include <iostream>

#include "types/v3_f32.hpp"
#include "objects/particle.hpp"

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

RenderItem* particle_render_item = NULL;
PxTransform particle_transform;

objects::particle particle;


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

	particle = objects::particle(v3_f32(spacing, spacing, spacing), -v3_f32(spacing, spacing, spacing) * 0.20f);
	particle_transform = PxTransform();

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
	particle_render_item = new RenderItem(CreateShape(PxSphereGeometry(size)), &particle_transform, color_particle);

	RegisterRenderItem(origin_render_item);
	RegisterRenderItem(positive_x_render_item);
	RegisterRenderItem(positive_y_render_item);
	RegisterRenderItem(positive_z_render_item);
	RegisterRenderItem(particle_render_item);


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

	// particle:
	objects::acceleration3_f32 a = objects::acceleration3_f32(0, -1, 0);
	particle.integrate_semi_implicit_euler(a, 0.9875f, t);
	particle >> particle_transform;
	//std::cout << "dt: " << t / 1000.0 << std::endl;
	std::cout << particle.position.x << ", " << particle.position.y << ", " << particle.position.z << std::endl;
	std::cout << particle.velocity.x << ", " << particle.velocity.y << ", " << particle.velocity.z << std::endl;
	std::cout << std::endl;

	gScene->simulate(t);
	gScene->fetchResults(true);
}

// Function to clean data
// Add custom code to the begining of the function
void cleanupPhysics(bool interactive)
{
	PX_UNUSED(interactive);

	// axis:
	DeregisterRenderItem(particle_render_item);
	DeregisterRenderItem(positive_z_render_item);
	DeregisterRenderItem(positive_y_render_item);
	DeregisterRenderItem(positive_x_render_item);
	DeregisterRenderItem(origin_render_item);


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
	//case 'B': break;
	//case ' ':	break;
	case ' ':
	{
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