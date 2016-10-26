#pragma once

#include <GL/glew.h>
#include "glm/glm.hpp"
#include "glm/gtc/matrix_transform.hpp"
#include <vector>

// SDL2 Headers
#include "SDL.h"
// Freetype
#include "include\SDL_ttf.h"

#define LENGTH_PROXIMAL_PHALANX			3.27f
#define LENGTH_INTERMEDIATE_PHALANX		1.81f
#define LENGTH_DISTAL_PHALANX			1.60f
#define LENGTH_SUM						LENGTH_PROXIMAL_PHALANX + LENGTH_INTERMEDIATE_PHALANX + LENGTH_DISTAL_PHALANX

#define DEMO_DOF						3
#define DEMO_THETA_INCREASE				M_PI * 0.06f
#define DEMO_CAMERA_DISTANCE			2
#define DEMO_SCALAR_JACOBIAN			0.1f
#define DEMO_TOLERANCE_EPSILON			0.011f
#define DEMO_ANIMATION_MIN_X			0.0f
#define DEMO_ANIMATION_MAX_X			6.0f
#define DEMO_ANIMATION_ITERATIONS		100

#define DEMO_ORTHO_TOP_LEFT				-6.0f * DEMO_CAMERA_DISTANCE
#define DEMO_ORTHO_TOP_RIGHT			6.0f * DEMO_CAMERA_DISTANCE
#define DEMO_ORTHO_BOTTOM_LEFT			-4.0f * DEMO_CAMERA_DISTANCE
#define DEMO_ORTHO_BOTTOM_RIGHT			4.0f * DEMO_CAMERA_DISTANCE

#define VIS_JOINT_RADIUS				0.1f
#define VIS_EXTENDED_LINE_LENGTH		1.00f


/* Axes display list */
static GLuint axes_list;

class Demo
{
public:
	enum DemoType { ForwardKinematics, ForwardKinematicsLimits, InverseKinematics, MaxDemos };

	Demo() {};
	Demo(SDL_GLContext *context, SDL_Window* window) 
	{
		// Set up projection matrix
		projection = glm::ortho(DEMO_ORTHO_TOP_LEFT, DEMO_ORTHO_TOP_RIGHT, DEMO_ORTHO_BOTTOM_LEFT, DEMO_ORTHO_BOTTOM_RIGHT);
			/*glm::perspective
			(
				45.0f,       // Field of view ( how far out to the sides we can see
				4.0f/3.0f, // Aspect ratio ( stretch image in widh or height )
				1.0f,        // Near plane ( anything close than this will be cut off )
				1000.0f       // Far plane ( anything further away than this will be cut off )
				);*/

		// Set up view matrix
		view = glm::lookAt
			(
				glm::vec3(0, 0, 1.0f), // Camera is at (0,0,-4), in World Space
				glm::vec3(0, 0, 0),  // And looks at the origin
				glm::vec3(0, 1, 0)  // Head is up ( set to 0,-1,0 to look upside-down )
				);

		//Initialize context
		mainContext = context;
		sdlWindow = window;
		sdlRenderer = SDL_CreateRenderer(window, -1, 0);

		//Initialize main font
		font = TTF_OpenFont("arial.ttf", 14);
		if (!font) {
			printf("TTF_OpenFont: %s\n", TTF_GetError());
		}

		//Initialize theta constraints

		thetaConstraints[0][0] = -M_PI / 3.0f;
		thetaConstraints[0][1] = M_PI / 3.0f;

		thetaConstraints[1][0] = -(2.0f * M_PI / 3);
		thetaConstraints[1][1] = 0.0f;

		thetaConstraints[2][0] = -(2.0f * M_PI / 3);
		thetaConstraints[2][1] = 0.0f;

		currentGuess = previousGuess = halfConstraints = glm::vec2(
			(thetaConstraints[0][0] + thetaConstraints[0][1]) / 2,
			(thetaConstraints[1][0] + thetaConstraints[1][1]) / 2);

		reachablePositions = std::vector<glm::vec2>();
	};

	~Demo() {};
	void Reset();
	void HandleInput();
	void Tick();
	void Draw();
	void SDLDraw();

	// Our opengl context handle
	SDL_GLContext *mainContext;
	SDL_Window *sdlWindow;
	SDL_Renderer *sdlRenderer;
	TTF_Font* font;

	// Demo settings
	DemoType type = DemoType::InverseKinematics;

	// Matricies
	glm::mat4 projection;
	glm::mat4 view;

	// forward kinematics 
	float theta[DEMO_DOF] = { 0.0f, 0.0f, 0.0f };
	float thetaConstraints[DEMO_DOF][2];

	// forward kinematic matrices
	glm::mat4 T1;
	glm::mat4 T2;
	glm::mat4 T3;
	glm::mat4 T4;

	// inverse kinematics
	glm::vec2 currentGuess, previousGuess, halfConstraints;
	glm::vec3 target = glm::vec3(0.0f, -2.0f, 0.0f);
	bool solved = false;
	int onObject = 100;

	std::vector<glm::vec2> reachablePositions;
	std::vector<glm::vec2> animation;

	glm::mat4 DenavitHartenbergMatrix(float a, float alpha, float d, float theta);
	void SolveFK(float theta1, float theta2, float theta3);
	void CalculateReachableTipPositions();
	void CalculateReachableTipPositionsLimits();

	glm::mat2 JacobiMatrix(float t1, float t2);
	void SolveIK(glm::vec3 target, glm::vec2 initialGuess);

	//initial guessing 
	glm::vec2 CircleCircleIntersection(glm::vec2 p1, float r1, glm::vec2 p2, float r2);

	glm::vec3 ScreenToWorld(glm::vec2 position);
	glm::vec3 ScreenToWorld2(glm::vec2 position);
	void DrawCross(float x, float y, float z, float size);
	void DrawLine(float x1, float y1, float z1, float x2, float y2, float z2);
	void DrawCircle(float x, float y, float radius);
	void DrawHollowCircle(float x, float y, float radius);
	void DrawArc(float cx, float cy, float r, float start_angle, float arc_angle, int num_segments);
	void DrawFilledArc(float cx, float cy, float r, float start_angle, float arc_angle, int num_segments);

	//SDL rendering
	void SDL_RenderText(const char *text, SDL_Color color, glm::vec2 position);
};