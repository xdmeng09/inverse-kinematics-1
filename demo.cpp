#include "stdafx.h"
#include "demo.h"
#include <algorithm>
#include "include/SDL_ttf.h"
#include "SDL.h"

static glm::vec3 backgroundColor = { 1.0f, 1.0f, 1.0f };
static glm::vec3 darkLineColor = { 0.0f, 0.0f, 0.0f };
static glm::vec3 halfDarkLineColor = { 0.15f, 0.15f, 0.15f };
static glm::vec3 lightLineColor = { 0.3f, 0.3f, 0.3f };
static glm::vec3 veryLightLineColor = { 0.9f, 0.9f, 0.9f };

static glm::vec3 objectColor = { 0.2f, 0.7f, 0.2f };
static glm::vec3 lRangeColor = { 0.5f, 0.5f, 1.0f };
static glm::vec3 rangeColor = { 0.2f, 0.2f, 0.7f };

static float lineThickness = 1.25f;
static float thickLineThickness = 3.5f;

static char* demoNames[Demo::DemoType::MaxDemos] = {"Forward kinematics without joint constraint", "Forward kinematics with joint constraint", "Inverse kinematics animation and mouse targeting"};

void Demo::Reset()
{
	theta[0] = 0.0f;
	theta[1] = 0.0f;
	theta[2] = 0.0f;

	if (type == DemoType::InverseKinematics)
	{
		//animation.clear();
		reachablePositions.clear();
		onObject = 0;
	}
	if (type == DemoType::ForwardKinematics)
	{
		onObject = 0;
		reachablePositions.clear();
		CalculateReachableTipPositions();
	}
	if (type == DemoType::ForwardKinematicsLimits)
	{
		onObject = 0;
		reachablePositions.clear();
		CalculateReachableTipPositionsLimits();
	}
};

void Demo::HandleInput()
{
	SDL_Event event;
	while (SDL_PollEvent(&event))
	{
		if (event.type == SDL_QUIT)
			break;

		if (event.type == SDL_MOUSEBUTTONDOWN)
		{
			switch (event.button.button)
			{
			case SDL_BUTTON_LEFT:
				int mouse_x, mouse_y;
				SDL_GetMouseState(&mouse_x, &mouse_y);
				glm::vec3 newTarget = glm::vec3(ScreenToWorld(glm::vec2(mouse_x,mouse_y)));
				newTarget.z = 0;
				if (target != newTarget)
				{
					target = newTarget;
					if (type == DemoType::InverseKinematics)
					{
						auto initialGuess = glm::vec2(theta[0], theta[1]);//glm::vec2((thetaConstraints[0][0] + thetaConstraints[0][1]) / 2, (thetaConstraints[1][0] + thetaConstraints[1][1]) / 2);
	
						SolveIK(target, initialGuess);
					}
				}
				break;
			}
		}

		if (event.type == SDL_KEYDOWN)
		{
			switch (event.key.keysym.sym)
			{
			case SDLK_UP:
				if (type != DemoType::InverseKinematics)
				{
					if (theta[0] + DEMO_THETA_INCREASE < thetaConstraints[0][1])
					{
						theta[0] += DEMO_THETA_INCREASE;
					}
					else
					{
						theta[0] = thetaConstraints[0][1];
					}
				}
				break;
			case SDLK_DOWN:
				if (type != DemoType::InverseKinematics)
				{
					if (theta[0] - DEMO_THETA_INCREASE > thetaConstraints[0][0])
					{
						theta[0] -= DEMO_THETA_INCREASE;
					}
					else
					{
						theta[0] = thetaConstraints[0][0];
					}
				}
				break;
			case SDLK_LEFT:
				if (type != DemoType::InverseKinematics)
				{
					if (theta[1] + DEMO_THETA_INCREASE < thetaConstraints[1][1])
					{
						theta[1] += DEMO_THETA_INCREASE;
					}
					else
					{
						theta[1] = thetaConstraints[1][1];
					}

					if (type == DemoType::ForwardKinematicsLimits)
					{
						theta[2] = 2 * theta[1] / 3;
					}

				}
				break;
			case SDLK_RIGHT:
				if (type != DemoType::InverseKinematics)
				{
					if (theta[1] - DEMO_THETA_INCREASE > thetaConstraints[1][0])
					{
						theta[1] -= DEMO_THETA_INCREASE;
					}
					else
					{
						theta[1] = thetaConstraints[1][0];
					}

					if (type == DemoType::ForwardKinematicsLimits)
					{
						theta[2] = 2 * theta[1] / 3;
					}
				}
				break;
			case SDLK_KP_PLUS:
				if (type == DemoType::ForwardKinematics)
				{
					if (theta[2] + DEMO_THETA_INCREASE < thetaConstraints[1][1])
					{
						theta[2] += DEMO_THETA_INCREASE;
					}
					else
					{
						theta[2] = thetaConstraints[1][1];
					}
				}
				break;
			case SDLK_KP_MINUS:
				if (type == DemoType::ForwardKinematics)
				{
					if (theta[2] - DEMO_THETA_INCREASE > thetaConstraints[1][0])
					{
						theta[2] -= DEMO_THETA_INCREASE;
					}
					else
					{
						theta[2] = thetaConstraints[1][0];
					}
				}
				break;
			case SDLK_1:
				type = DemoType::ForwardKinematics;
				Reset();
				break;
			case SDLK_2:
				type = DemoType::ForwardKinematicsLimits;
				Reset();
				break;
			case SDLK_3:
				type = DemoType::InverseKinematics;
				Reset();
				break;
			case SDLK_r:
				Reset();
				hideReachablePoints = false;
				break;
			case SDLK_F1:
				// visual bools
				hideReachablePoints = !hideReachablePoints;
				break;
			case SDLK_F2:
				hideAngles = !hideAngles;
				break;
			case SDLK_F3:
				hideMaxAngles = !hideMaxAngles;
				break;
			case SDLK_F4:
				hideMaxRange = !hideMaxRange;
				break;
			case SDLK_F5:
				hideObject = !hideObject;
				break;
			default:
				break;
			}
		}
	}
}

void Demo::Tick()
{
	if (type == DemoType::InverseKinematics)
	{
		SolveFK(theta[0], theta[1], 2 * theta[1] / 3);

		if (onObject < DEMO_ANIMATION_ITERATIONS)
		{
			float maxX = DEMO_ANIMATION_MAX_X;
			glm::vec3 newTarget = glm::vec3(DEMO_ANIMATION_MIN_X + (DEMO_ANIMATION_MAX_X / DEMO_ANIMATION_ITERATIONS) * (float)onObject, -2.0f, 0.0f);
			if (target != newTarget)
			{
				target = newTarget;
				SolveIK(target, glm::vec2(theta[0], theta[1]));
			//	animation.push_back(glm::vec2(theta[0], theta[1]));
			}
			onObject++;
		}
	}
	else
	{
		//Forward kinematics only mode
		SolveFK(theta[0], theta[1], theta[2]);
	}

};

void Demo::Draw()
{
	//Enable smooth line drawing
	glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
	glHint(GL_POLYGON_SMOOTH_HINT, GL_NICEST);

	//Clear BG color
	glClearColor(backgroundColor.r, backgroundColor.g, backgroundColor.b, 1.0f);
	glClear(GL_COLOR_BUFFER_BIT);

	//Initialize projection & view matrices
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glLoadMatrixf(&projection[0][0]);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glLoadMatrixf(&view[0][0]);

	glLineWidth(lineThickness);
	glColor3f(veryLightLineColor.r, veryLightLineColor.g, veryLightLineColor.b);

	//Draw grid
	float start = -8.0f; float end = 8.0f; float interval = 1.0f;
	glBegin(GL_LINES);
	while (start < end)
	{
		glVertex3f(-12.0f, start, 0); glVertex3f(12.0f, start, 0);
		start += interval;
	}
	start = -12.0f; end = 12.0f;
	while (start < end)
	{
		glVertex3f(start, -8.0f, 0); glVertex3f(start, 8.0f, 0);
		start += interval;
	}
	glEnd();

	// Draw a red x-axis, a green y-axis, and a blue z-axis.
	glBegin(GL_LINES);
	glColor3f(halfDarkLineColor.r, halfDarkLineColor.g, halfDarkLineColor.b);
	glVertex3f(-12.0f, 0, 0); glVertex3f(12.0f, 0, 0);
	glVertex3f(0, -8.0f, 0); glVertex3f(0, 8.0f, 0);
	//glColor3f(0, 0, 1); glVertex3f(0, 0, 0); glVertex3f(0, 0, 1);
	glEnd();

	//Draw reachable positions 
	glColor3f(lRangeColor.r, lRangeColor.g, lRangeColor.b);
	glPointSize(3.0);

	if (!reachablePositions.empty() && hideReachablePoints == false)
	{
		for (auto rp : reachablePositions)
		{
			glBegin(GL_POINTS); // render with points

			glVertex3f(rp.x, rp.y, 0.0f); //display a point
			glEnd();
		}
	}

	// Draw constrained object
	if (hideObject == false)
	{
		glLineWidth(5.0f);
		glBegin(GL_LINES);
		glColor3f(objectColor.r, objectColor.g, objectColor.b); glVertex3f(-100, -2.0f, 0); glVertex3f(100, -2.0f, 0);
		glEnd();
	}

	glLineWidth(lineThickness);

	//Draw target position
	glColor3f(rangeColor.r, rangeColor.g, rangeColor.b);
	DrawCross(target.x, target.y, target.z, VIS_JOINT_RADIUS);

	// Draw max range
	if (hideMaxRange == false)
	{
		glLineWidth(5.0f);
		DrawHollowCircle(0, 0, LENGTH_PROXIMAL_PHALANX + LENGTH_INTERMEDIATE_PHALANX + LENGTH_DISTAL_PHALANX);
	}

	auto L0 = glm::vec4(glm::vec3(0.0), 1.0);
	auto O1 = L0;
	auto O2 = O1 + L0 * T1;
	auto O3 = O2 + L0 * T2;
	auto O4 = O3 + L0 * T3;

	glLineWidth(lineThickness);

	//Draw theta 1 arc
	glColor3f(lightLineColor.r, lightLineColor.g, lightLineColor.b);
	if (hideAngles == false)
	{
		DrawArc(O1.x, O1.y, LENGTH_INTERMEDIATE_PHALANX * 0.5f, 0.0f, theta[0], 20);

	//Draw theta 2 arc

		auto extO2 = O2 + LENGTH_INTERMEDIATE_PHALANX * glm::normalize(O2 - O1);
		DrawLine(O2.x, O2.y, O2.z, extO2.x, extO2.y, extO2.z);
		DrawArc(O2.x, O2.y, LENGTH_INTERMEDIATE_PHALANX * 0.5f, theta[0], theta[1], 20);

	//Draw theta 3 arc

		auto extO3 = O3 + LENGTH_INTERMEDIATE_PHALANX * glm::normalize(O3 - O2);
		DrawLine(O3.x, O3.y, O3.z, extO3.x, extO3.y, extO3.z);
		if (type == DemoType::InverseKinematics || type == DemoType::ForwardKinematicsLimits)
		{
			DrawArc(O3.x, O3.y, LENGTH_INTERMEDIATE_PHALANX * 0.5f, theta[0] + theta[1], 2 * theta[1] / 3, 20);
		}
		else
		{
			DrawArc(O3.x, O3.y, LENGTH_INTERMEDIATE_PHALANX * 0.5f, theta[0] + theta[1], theta[2], 20);
		}
	}

	if (hideMaxAngles == false)
	{
		DrawFilledArc(O1.x, O1.y, LENGTH_PROXIMAL_PHALANX, thetaConstraints[0][0], thetaConstraints[0][1] * 2.0f, 20);
		DrawFilledArc(O2.x, O2.y, LENGTH_INTERMEDIATE_PHALANX, thetaConstraints[1][1] + theta[0], thetaConstraints[1][0], 20);
		if (type == DemoType::InverseKinematics || type == DemoType::ForwardKinematicsLimits)
		{
			DrawFilledArc(O3.x, O3.y, LENGTH_DISTAL_PHALANX, (2 * thetaConstraints[1][1] / 3) + theta[0] + theta[1], (2 * thetaConstraints[1][0] / 3), 20);
		}
		else
		{
			DrawFilledArc(O3.x, O3.y, LENGTH_DISTAL_PHALANX, thetaConstraints[2][1] + theta[0] + theta[1], thetaConstraints[2][0], 20);
		}
	}

	glColor4f(darkLineColor.r, darkLineColor.g, darkLineColor.b, 1.0f);

	//Draw links
	glLineWidth(thickLineThickness);
	DrawLine(O1.x, O1.y, O1.z, O2.x, O2.y, O2.z);
	DrawLine(O2.x, O2.y, O2.z, O3.x, O3.y, O3.z);
	DrawLine(O3.x, O3.y, O3.z, O4.x, O4.y, O4.z);
	glLineWidth(1.0f);

	//Draw joints
	DrawCircle(O1.x, O1.y, VIS_JOINT_RADIUS);
	DrawHollowCircle(O1.x, O1.y, VIS_JOINT_RADIUS);

	DrawCircle(O2.x, O2.y, VIS_JOINT_RADIUS);
	DrawHollowCircle(O2.x, O2.y, VIS_JOINT_RADIUS);

	DrawCircle(O3.x, O3.y, VIS_JOINT_RADIUS);
	DrawHollowCircle(O3.x, O3.y, VIS_JOINT_RADIUS);

	DrawCircle(O4.x, O4.y, VIS_JOINT_RADIUS);
	DrawHollowCircle(O4.x, O4.y, VIS_JOINT_RADIUS);


//Init text buffer
	char buffer[100];

	glLineWidth(lineThickness);
	glColor4f(darkLineColor.r, darkLineColor.g, darkLineColor.b, 1.0f);

	glEnable(GL_TEXTURE_2D);
	sprintf(buffer, "Demo %i - %s", type + 1, demoNames[type]);
	SDL_RenderText(buffer, { 255, 255, 255 }, glm::vec2(5.0f, 5.0f));

	sprintf(buffer, "Tip (x, y):  (%f, %f)", O4.x, O4.y);
	SDL_RenderText(buffer, { 255, 255, 255 }, glm::vec2(5.0f, 25.0f));

	sprintf(buffer, "Theta 1:  %f°", glm::degrees(theta[0]));
	SDL_RenderText(buffer, { 255, 255, 255 }, glm::vec2(5.0f, 45.0f));

	sprintf(buffer, "Theta 2:  %f°", glm::degrees(theta[1]));
	SDL_RenderText(buffer, { 255, 255, 255 }, glm::vec2(5.0f, 65.0f));

	if (type == DemoType::InverseKinematics || type == DemoType::ForwardKinematicsLimits)
	{
		sprintf(buffer, "Theta 3:  %f°", glm::degrees(2 * theta[1] / 3));
		SDL_RenderText(buffer, { 255, 255, 255 }, glm::vec2(5.0f, 85.0f));
	}
	else
	{
		sprintf(buffer, "Theta 3:  %f°", glm::degrees(theta[2]));
		SDL_RenderText(buffer, { 255, 255, 255 }, glm::vec2(5.0f, 85.0f));
	}

	SDL_RenderText("(0,0)", { 255, 255, 255 }, glm::vec2(600, 400.0f));




	glDisable(GL_TEXTURE_2D);

	glFlush();
};

void Demo::SDLDraw()
{

	//SDL_RenderPresent(sdlRenderer);
}

void Demo::SolveFK(float theta1, float theta2, float theta3)
{
	T1 = DenavitHartenbergMatrix(LENGTH_PROXIMAL_PHALANX, 0, 0, theta1);
	T2 = DenavitHartenbergMatrix(LENGTH_INTERMEDIATE_PHALANX, 0, 0, theta1 + theta2);
	T3 = DenavitHartenbergMatrix(LENGTH_DISTAL_PHALANX, 0, 0, theta1 + theta2 + theta3);
}

void Demo::SolveIK(glm::vec3 t, glm::vec2 initialGuess)
{
	//Edge case #1: Out of reach target
	if (glm::length(target) >= LENGTH_PROXIMAL_PHALANX + LENGTH_INTERMEDIATE_PHALANX + LENGTH_DISTAL_PHALANX)
	{
		target = glm::normalize(target) * (LENGTH_PROXIMAL_PHALANX + LENGTH_INTERMEDIATE_PHALANX + LENGTH_DISTAL_PHALANX);
		initialGuess = glm::vec2(atan(t.y / t.x), 0.0f);
	}

	//Initial guess
	currentGuess = previousGuess = initialGuess;

	//Update forward kinematics
	SolveFK(currentGuess.x,currentGuess.y, 2 * currentGuess.y / 3);

	//Update tip position
	auto L0 = glm::vec4(glm::vec3(0.0), 1.0);
	auto O1 = L0;
	auto O2 = O1 + L0 * T1;
	auto O3 = O2 + L0 * T2;
	auto T = O3 + L0 * T3; //Tip position

	int i = 0;

	while (glm::length(target - glm::vec3(T.x, T.y, T.z)) > DEMO_TOLERANCE_EPSILON)
	{
		//Only run for n iterations
		if (i > 500 - 1)
			break;

		//Jacobi-matrix of previous guess
		auto J = JacobiMatrix(
			previousGuess[0],
			previousGuess[1]
			);


		//Update forward kinematics
		SolveFK(previousGuess.x, previousGuess.y, 2 * previousGuess.y / 3);

		//Update tip position
		L0 = glm::vec4(glm::vec3(0.0), 1.0);
		O1 = L0;
		O2 = O1 + L0 * T1;
		O3 = O2 + L0 * T2;
		T = O3 + L0 * T3; //Tip position

		//Calculate psuedo-inverse of the JAcobian
		auto J_psuedoInverse = glm::inverse(glm::transpose(J) * J) * glm::transpose(J);

		//Calculate residue between target and tip position
		auto residue = target - glm::vec3(T.x, T.y, 0.0f);

		//Calculate currernt guess
		currentGuess = previousGuess + DEMO_SCALAR_JACOBIAN * J_psuedoInverse * residue;

		//Constrain current guess to joint angle limits
		currentGuess.x = std::max(thetaConstraints[0][0], std::min(currentGuess.x, thetaConstraints[0][1]));
		currentGuess.y = std::max(thetaConstraints[1][0], std::min(currentGuess.y, thetaConstraints[1][1]));

		previousGuess = currentGuess;
		i++;
	}

	printf("Iterations needed: %i \n", i);

	theta[0] = currentGuess.x;
	printf("Theta 1 - %f - %f - %f\n", thetaConstraints[0][0], theta[0], thetaConstraints[0][1]);
	theta[1] = currentGuess.y;
	printf("Theta 2 - %f - %f - %f\n", thetaConstraints[1][0], theta[1], thetaConstraints[1][1]);
	//theta[2] = currentGuess.z;
	//printf("Theta 3 - %f - %f - %f\n", thetaConstraints[2][0], theta[2], thetaConstraints[2][1]);
}

void Demo::CalculateReachableTipPositions()
{
	for (float t1 = thetaConstraints[0][0]; t1 < thetaConstraints[0][1]; t1 += DEMO_THETA_INCREASE)
	{
		for (float t2 = thetaConstraints[1][0]; t2 < thetaConstraints[1][1]; t2+= DEMO_THETA_INCREASE)
		{
			for (float t3 = thetaConstraints[2][0]; t3 < thetaConstraints[2][1]; t3+= DEMO_THETA_INCREASE)
			{
				SolveFK(t1, t2, t3);
				auto L0 = glm::vec4(glm::vec3(0.0), 1.0);

				auto O1 = L0;
				auto O2 = O1 + L0 * T1;
				auto O3 = O2 + L0 * T2;
				auto O4 = O3 + L0 * T3;
				reachablePositions.push_back(O4);
			}
		}
	}
}

void Demo::CalculateReachableTipPositionsLimits()
{
	for (float t1 = thetaConstraints[0][0]; t1 < thetaConstraints[0][1]; t1 += DEMO_THETA_INCREASE)
	{
		for (float t2 = thetaConstraints[1][0]; t2 < thetaConstraints[1][1]; t2 += DEMO_THETA_INCREASE)
		{
			SolveFK(t1, t2, 2 * t2 / 3);
			auto L0 = glm::vec4(glm::vec3(0.0), 1.0);

			auto O1 = L0;
			auto O2 = O1 + L0 * T1;
			auto O3 = O2 + L0 * T2;
			auto O4 = O3 + L0 * T3;
			reachablePositions.push_back(O4);
		}
	}
}


glm::mat4 Demo::DenavitHartenbergMatrix(float a, float alpha, float d, float theta)
{
	return glm::mat4(
		cosf(theta), -sinf(theta) * cosf(alpha), sinf(theta) * sinf(alpha), a * cosf(theta),
		sinf(theta), cosf(theta) * cosf(alpha), -cosf(theta) * sinf(alpha), a * sinf(theta),
		0, sinf(alpha), cosf(alpha), d,
		0, 0, 0, 1);
}

glm::mat2 Demo::JacobiMatrix(float t1, float t2)
{
	float l1 = LENGTH_PROXIMAL_PHALANX;
	float l2 = LENGTH_INTERMEDIATE_PHALANX;
	float l3 = LENGTH_DISTAL_PHALANX;

	float t1t2 = t1 + t2;

	return glm::mat2(
		-l1 * sinf(t1) - l2 * sinf(t1t2) - l3 * sinf(t1 + t2 + (2 / 3) * t2),
		-l2 * sinf(t1t2) - (l3 * 5 * cosf(t1 + t2 + (2/3) * t2)) * (1/3),
		l1 * cosf(t1) + l2 * cosf(t1t2) + l3 * cosf(t1 + t2 + (2 / 3) * t2),
		l2 * cosf(t1t2) + ((l3 * (5)) * cosf(t1 + t2 + (2 / 3) * t2)) * (1/3)
		);
}

glm::vec2 Demo::CircleCircleIntersection(glm::vec2 p1, float r1, glm::vec2 p2, float r2)
{
	glm::mat2 result;

	float distance = glm::distance(p1, p2);

	if (distance > r1 + r2)
	{
		return glm::vec2(0, 0);
	}
	if (distance < abs(r1 - r2))
	{
		return glm::vec2(0, 0);
	}

	float a = ((r1*r1) - (r2*r2) + (distance*distance)) / (2.0 * distance);
	
	glm::vec2 p3 = glm::vec2(p1.x + (p2.x - p1.x) * a / distance, p1.y + (p2.y - p1.y) * a / distance);
	float distanceToIntersection = sqrtf((r1*r1) - (a*a));

	glm::vec2  pOffset = glm::vec2(-(p2.y - p1.y) * (distanceToIntersection / distance), -(p2.x - p1.x) * (distanceToIntersection / distance));

	return glm::vec2(
		p3.x - pOffset.x,
		p3.y - pOffset.y
		);
}

glm::vec3 Demo::ScreenToWorld(glm::vec2 position)
{
	glm::vec2 temp = position;
	temp.x = 2.0 * position.x / 1200 - 1;
	temp.y = -2.0 * position.y / 800 + 1;

	glm::mat4 vpi = glm::inverse(view * projection);

	glm::vec3 posWorld = vpi * glm::vec4(temp, 1, 1) * glm::vec4(glm::vec2(1200 / 800, 1200 / 800), 1, 1);
	return posWorld;
}

glm::vec3 Demo::ScreenToWorld2(glm::vec2 position)
{
	glm::vec3 posWorld;
	posWorld.x = DEMO_ORTHO_TOP_LEFT + (2.0f * position.x / 1200.0f) * DEMO_ORTHO_TOP_RIGHT;
	posWorld.y = DEMO_ORTHO_BOTTOM_RIGHT + (2.0f * position.y / 800.0f) * DEMO_ORTHO_BOTTOM_LEFT;
	posWorld.z = 0.0f;
	return posWorld;
}

void Demo::DrawCross(float x, float y, float z, float size)
{
	glBegin(GL_LINES);
	glVertex3f(x - size, y, z); glVertex3f(x + size, y, z);
	glVertex3f(x, y - size, z); glVertex3f(x, y + size, z);
	glVertex3f(x, y, z - size); glVertex3f(x, y, z + size);
	glEnd();
}

void Demo::DrawLine(float x1, float y1, float z1, float x2, float y2, float z2)
{
	glBegin(GL_LINES);
	glVertex3f(x1, y1, z2); 
	glVertex3f(x2, y2, z2);
	glEnd();
}

void Demo::DrawCircle(float x, float y, float radius)
{

	int lineAmount = 100; //# of triangles used to draw circle

						  //GLfloat radius = 0.8f; //radius
	float twicePi = 2.0f * M_PI;

	glBegin(GL_POLYGON);
	//Change the 6 to 12 to increase the steps (number of drawn points) for a smoother circle
	//Note that anything above 24 will have little affect on the circles appearance
	//Play with the numbers till you find the result you are looking for
	//Value 1.5 - Draws Triangle
	//Value 2 - Draws Square
	//Value 3 - Draws Hexagon
	//Value 4 - Draws Octagon
	//Value 5 - Draws Decagon
	//Notice the correlation between the value and the number of sides
	//The number of sides is always twice the value given this range
	for (double i = 0; i <= 100; i ++) //<-- Change this Value
		glVertex2f(
			x + (radius * cos(i *  twicePi / lineAmount)),
			y + (radius* sin(i * twicePi / lineAmount))
			);
	glEnd();
}

void Demo::DrawHollowCircle(float x, float y, float radius) {
	int i;
	int lineAmount = 100; //# of triangles used to draw circle

						  //GLfloat radius = 0.8f; //radius
	float twicePi = 2.0f * M_PI;

	glBegin(GL_LINE_LOOP);
	for (i = 0; i <= lineAmount; i++) {
		glVertex2f(
			x + (radius * cos(i *  twicePi / lineAmount)),
			y + (radius* sin(i * twicePi / lineAmount))
			);
	}
	glEnd();
}

void Demo::DrawArc(float cx, float cy, float r, float start_angle, float arc_angle, int num_segments)
{
	float theta = arc_angle / float(num_segments - 1);//theta is now calculated from the arc angle instead, the - 1 bit comes from the fact that the arc is open

	float tangetial_factor = tanf(theta);

	float radial_factor = cosf(theta);


	float x = r * cosf(start_angle);//we now start at the start angle
	float y = r * sinf(start_angle);

	glBegin(GL_LINE_STRIP);//since the arc is not a closed curve, this is a strip now
	for (int ii = 0; ii < num_segments; ii++)
	{
		glVertex2f(x + cx, y + cy);

		float tx = -y;
		float ty = x;

		x += tx * tangetial_factor;
		y += ty * tangetial_factor;

		x *= radial_factor;
		y *= radial_factor;
	}
	glEnd();
}

void Demo::DrawFilledArc(float cx, float cy, float r, float start_angle, float arc_angle, int num_segments)
{
	float theta = arc_angle / float(num_segments - 1);//theta is now calculated from the arc angle instead, the - 1 bit comes from the fact that the arc is open

	float tangetial_factor = tanf(theta);

	float radial_factor = cosf(theta);



	float x = r * cosf(start_angle);//we now start at the start angle
	float y = r * sinf(start_angle);

	glBegin(GL_LINE_LOOP);//since the arc is not a closed curve, this is a strip now
	glVertex2f(cx, cy);
	for (int ii = 0; ii < num_segments; ii++)
	{
		glVertex2f(x + cx, y + cy);

		float tx = -y;
		float ty = x;

		x += tx * tangetial_factor;
		y += ty * tangetial_factor;

		x *= radial_factor;
		y *= radial_factor;
	}

	//glVertex2f(cx, cy);

	glEnd();
}

//RenderText( const char *text, SDL_Color color, int x, int y )
//Renders a const char array 'text' to the current renderer using SDL_Color color at position (x,y)
//Returns height of the drawn text.
void Demo::SDL_RenderText(const char *text, SDL_Color color, const glm::vec2 position)
{
	SDL_Surface* textSurface = TTF_RenderText_Blended(font, text,
		color);

	GLuint TextureID = 0;

	glGenTextures(1, &TextureID);
	glBindTexture(GL_TEXTURE_2D, TextureID);

	int Mode = GL_RGB;

	if (textSurface->format->BytesPerPixel == 4) {
		Mode = GL_RGBA;
	}

	glm::vec3 minRect = ScreenToWorld2(position);

	glm::vec3 maxRect = ScreenToWorld2( glm::vec3(position.x + textSurface->w, position.y + textSurface->h, 0.0f));
	//maxRect += test;
	//glm::vec2 maxRect = glm::vec2((minRect.x + textSurface->w) / DEMO_CAMERA_DISTANCE / 3.0f, (minRect.y - textSurface->h) / DEMO_CAMERA_DISTANCE / 3.0f);

	//printf("minrect x %f y %f \n", minRect.x, minRect.y);
	//printf("maxrect x %f y %f \n", maxRect.x, maxRect.y);

	glTexImage2D(GL_TEXTURE_2D, 0, Mode, textSurface->w, textSurface->h, 0, Mode, GL_UNSIGNED_BYTE, textSurface->pixels);

	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

	glBindTexture(GL_TEXTURE_2D, TextureID);

	glBegin(GL_QUADS);
	float Width = 1.0f;
	float Height = 1.0f;
	glTexCoord2f(0, 0); glVertex3f(minRect.x, minRect.y, 0);
	glTexCoord2f(1, 0); glVertex3f(maxRect.x, minRect.y, 0);
	glTexCoord2f(1, 1); glVertex3f(maxRect.x,  maxRect.y, 0);
	glTexCoord2f(0, 1); glVertex3f(minRect.x,  maxRect.y, 0);
	glEnd();

	SDL_FreeSurface(textSurface);
}