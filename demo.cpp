#include "stdafx.h"
#include "demo.h"
#include <algorithm>
#include "include/SDL_ttf.h"
#include "SDL.h"
void Demo::Reset()
{
	onObject = -10;
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
					SolveIK(target);
				}
				break;
			}
		}

		if (event.type == SDL_KEYDOWN)
		{
			switch (event.key.keysym.sym)
			{
			case SDLK_UP:
				theta[0] += DEMO_THETA_INCREASE;
				break;
			case SDLK_DOWN:
				theta[0] -= DEMO_THETA_INCREASE;
				break;
			case SDLK_LEFT:
				theta[1] += DEMO_THETA_INCREASE;
				break;
			case SDLK_RIGHT:
				theta[1] -= DEMO_THETA_INCREASE;
				break;
			case SDLK_KP_PLUS:
				theta[2] += DEMO_THETA_INCREASE;
				break;
			case SDLK_KP_MINUS:
				theta[2] -= DEMO_THETA_INCREASE;
				break;
			case SDLK_r:
				Reset();
				break;
			default:
				for (int j = 0; j < 4; j++)
				{
					for (int i = 0; i < 4; i++)
					{
						printf("T1 @ {%i,%i} %f\n", i, j, T1[i][j]);
					}
				}
				for (int j = 0; j < 4; j++)
				{
					for (int i = 0; i < 4; i++)
					{
						printf("T2 @ {%i,%i} %f\n", i, j, T2[i][j]);
					}
				}
				for (int j = 0; j < 4; j++)
				{
					for (int i = 0; i < 4; i++)
					{
						printf("T3 @ {%i,%i} %f\n", i, j, T3[i][j]);
					}
				}
				for (int j = 0; j < 4; j++)
				{
					for (int i = 0; i < 4; i++)
					{
						printf("T4 @ {%i,%i} %f\n", i, j, T4[i][j]);
					}
				}
				break;
			}
		}
	}
}

void Demo::Tick()
{
	T1 = DenavitHartenbergMatrix(LENGTH_PROXIMAL_PHALANX, 0, 0, theta[0]);
	T2 = DenavitHartenbergMatrix(LENGTH_INTERMEDIATE_PHALANX, 0, 0, theta[0] + theta[1]);
	T3 = DenavitHartenbergMatrix(LENGTH_DISTAL_PHALANX, 0, 0, theta[0] + theta[1] + (2*theta[1]/3));

	if (onObject < 100)
	{
		float maxX = LENGTH_SUM - 0.2f;
		glm::vec3 newTarget = glm::vec3(0.0f +(maxX / 100.0f) * (float)onObject,-2.0f, 0.0f);
		if (target != newTarget)
		{
			target = newTarget;
			SolveIK(target);
		}
		onObject++;
	}

	/*int mouse_x, mouse_y;
	SDL_GetMouseState(&mouse_x, &mouse_y);
	double x = 2.0 * mouse_x / 1200 - 1;
	double y = -2.0 * mouse_y / 800 + 1;

	glm::mat4 vpi = glm::inverse(view * projection);

	glm::vec3 mouseWorld = vpi * glm::vec4(glm::vec3(x, y, 0), 1) * glm::vec4(glm::vec2(1200 / 800 * 15, 1200 / 800 * 15), 1, 1);
	*/

	//if (!solved)
	//{
	//	SolveIK(target);
	//	solved = !solved;
	//}

};

void Demo::Draw()
{
	//Enable smooth line drawing
	glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
	glHint(GL_POLYGON_SMOOTH_HINT, GL_NICEST);

	//Clear BG color
	glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
	glClear(GL_COLOR_BUFFER_BIT);

	// Set the camera lens so that we have a perspective viewing volume whose
	// horizontal bounds at the near clipping plane are -2..2 and vertical
	// bounds are -1.5..1.5.  The near clipping plane is 1 unit from the camera
	// and the far clipping plane is 40 units away.
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glLoadMatrixf(&projection[0][0]);

	// Set up transforms so that the tetrahedron which is defined right at
	// the origin will be rotated and moved into the view volume.  First we
	// rotate 70 degrees around y so we can see a lot of the left side.
	// Then we rotate 50 degrees around x to "drop" the top of the pyramid
	// down a bit.  Then we move the object back 3 units "into the screen".
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glLoadMatrixf(&view[0][0]);


	// Draw the tetrahedron.  It is a four sided figure, so when defining it
	// with a triangle strip we have to repeat the last two vertices.
	/*glBegin(GL_TRIANGLE_STRIP);
	glColor3f(1, 1, 1); glVertex3f(0, 2, 0);
	glColor3f(1, 0, 0); glVertex3f(-1, 0, 1);
	glColor3f(0, 1, 0); glVertex3f(1, 0, 1);
	glColor3f(0, 0, 1); glVertex3f(0, 0, -1.4);
	glColor3f(1, 1, 1); glVertex3f(0, 2, 0);
	glColor3f(1, 0, 0); glVertex3f(-1, 0, 1);
	glEnd();*/

	// Draw a red x-axis, a green y-axis, and a blue z-axis.
	glBegin(GL_LINES);
	glColor3f(1, 0, 0); glVertex3f(-100, 0, 0); glVertex3f(100, 0, 0);
	glColor3f(0, 1, 0); glVertex3f(0, -100, 0); glVertex3f(0, 100, 0);
	glColor3f(0, 0, 1); glVertex3f(0, 0, 0); glVertex3f(0, 0, 1);
	glEnd();

	auto L0 = glm::vec4(glm::vec3(0.0), 1.0);

	auto O1 = L0;
	auto O2 = O1 + L0 * T1;
	auto O3 = O2 + L0 * T2;
	auto O4 = O3 + L0 * T3;

	//Draw theta 1 arc
	glColor3f(0.4f, 0.4f, 0.4f);
	DrawArc(O1.x, O1.y, LENGTH_INTERMEDIATE_PHALANX * 0.5f, 0.0f, theta[0], 20);

	//Draw theta 2 arc
	auto extO2 = O2 + LENGTH_INTERMEDIATE_PHALANX * glm::normalize(O2 - O1);
	DrawLine(O2.x, O2.y, O2.z, extO2.x, extO2.y, extO2.z);
	DrawArc(O2.x, O2.y, LENGTH_INTERMEDIATE_PHALANX * 0.5f, theta[0], theta[1], 20);

	//Draw theta 3 arc
	auto extO3 = O3 + LENGTH_INTERMEDIATE_PHALANX * glm::normalize(O3 - O2);
	DrawLine(O3.x, O3.y, O3.z, extO3.x, extO3.y, extO3.z);
	DrawArc(O3.x, O3.y, LENGTH_INTERMEDIATE_PHALANX * 0.5f, theta[0] + theta[1], 2 * theta[1] / 3, 20);


	DrawFilledArc(O1.x, O1.y, LENGTH_PROXIMAL_PHALANX, thetaConstraints[0][0], thetaConstraints[0][1] * 2.0f, 20);
	DrawFilledArc(O2.x, O2.y, LENGTH_INTERMEDIATE_PHALANX, thetaConstraints[1][1] + theta[0], thetaConstraints[1][0], 20);
	DrawFilledArc(O3.x, O3.y, LENGTH_DISTAL_PHALANX, (2 * thetaConstraints[1][1] / 3) + theta[0] + theta[1], (2 * thetaConstraints[1][0] / 3), 20);


	glColor3f(1, 1, 1);

	DrawLine(O1.x, O1.y, O1.z, O2.x, O2.y, O2.z);
	DrawLine(O2.x, O2.y, O2.z, O3.x, O3.y, O3.z);
	DrawLine(O3.x, O3.y, O3.z, O4.x, O4.y, O4.z);

	glColor3f(0.0f, 0.0f, 1.0f);
	DrawCross(O1.x, O1.y, O1.z, VIS_JOINT_RADIUS);
	DrawHollowCircle(O1.x, O1.y, VIS_JOINT_RADIUS);

	glColor3f(0.0f, 1.0f, 1.0f);
	DrawCross(O2.x, O2.y, O2.z, VIS_JOINT_RADIUS);
	DrawHollowCircle(O2.x, O2.y, VIS_JOINT_RADIUS);

	glColor3f(1.0f, 0.0f, 1.0f);
	DrawCross(O3.x, O3.y, O3.z, VIS_JOINT_RADIUS);
	DrawHollowCircle(O3.x, O3.y, VIS_JOINT_RADIUS);

	glColor3f(1.0f, 0.0f, 0.0f);
	DrawCross(O4.x, O4.y, O4.z, VIS_JOINT_RADIUS);
	DrawHollowCircle(O4.x, O4.y, VIS_JOINT_RADIUS);

	glBegin(GL_LINES);
	glColor3f(0, 0.8f, 0.8f); glVertex3f(-100, -2.0f, 0); glVertex3f(100, -2.0f, 0);
	glEnd();

	glColor3f(1.0f, 1.0f, 0.0f);
	DrawCross(target.x, target.y, target.z, VIS_JOINT_RADIUS);

	glColor3f(1.0f, 1.0f, 0.0f);

	DrawHollowCircle(0, 0, LENGTH_PROXIMAL_PHALANX + LENGTH_INTERMEDIATE_PHALANX + LENGTH_DISTAL_PHALANX);
//	DrawCross(mouseWorld.x, mouseWorld.y, 0.0f, VIS_JOINT_RADIUS);

//Init text buffer
	char buffer[100];

	glColor4f(1.0f, 1.0f, 1.0f, 1.0f);

	glEnable(GL_TEXTURE_2D);
	sprintf(buffer, "Theta 1:   %fk", theta[0]);
	SDL_RenderText("(0,0)", { 255, 255, 255 }, glm::vec2(600, 400.0f));
	glDisable(GL_TEXTURE_2D);

	glFlush();
};

void Demo::SDLDraw()
{

	//SDL_RenderPresent(sdlRenderer);
}

void Demo::SolveIK(glm::vec3 t)
{
	T1 = DenavitHartenbergMatrix(LENGTH_PROXIMAL_PHALANX, 0, 0, theta[0]);
	T2 = DenavitHartenbergMatrix(LENGTH_INTERMEDIATE_PHALANX, 0, 0, theta[0] + theta[1]);
	T3 = DenavitHartenbergMatrix(LENGTH_DISTAL_PHALANX, 0, 0, theta[0] + theta[1] + (2 * theta[1] / 3));

	//Calculate current positions 
	auto L0 = glm::vec4(glm::vec3(0.0), 1.0);
	auto O1 = L0;
	auto O2 = O1 + L0 * T1;
	auto O3 = O2 + L0 * T2;
	auto T = O3 + L0 * T3; //Tip position

	int i = 0;

	//Initial guess
	currentGuess = previousGuess = glm::vec2(
		(thetaConstraints[0][0] + thetaConstraints[0][1]) / 2,
		(thetaConstraints[1][0] + thetaConstraints[1][1]) / 2);

	//Edge case #2: Above x-axis
	if (target.y > -(LENGTH_INTERMEDIATE_PHALANX + LENGTH_DISTAL_PHALANX))
	{
	//	currentGuess.x = thetaConstraints[0][0] / 2;
	}

	if (target.y > 0.0f)
	{
	//	currentGuess.x = thetaConstraints[0][0] / 4;
	}

	//Edge case #3: Under x-axis
	if (target.y < -(LENGTH_INTERMEDIATE_PHALANX + LENGTH_DISTAL_PHALANX))
	{
	//	currentGuess.x = thetaConstraints[0][1] / 2;
	}

	//Edge case #1: Out of reach target
	if (glm::length(target) > LENGTH_PROXIMAL_PHALANX + LENGTH_INTERMEDIATE_PHALANX + LENGTH_DISTAL_PHALANX)
	{
		target = glm::normalize(target) * (LENGTH_PROXIMAL_PHALANX + LENGTH_INTERMEDIATE_PHALANX + LENGTH_DISTAL_PHALANX - 0.01f);
		currentGuess = previousGuess =
			glm::vec2(
				std::max(thetaConstraints[0][0], std::min(atan(target.y / target.x), thetaConstraints[0][1])) ,
				0.0f
				);
	}

	//Update forward kinematics
	T1 = DenavitHartenbergMatrix(LENGTH_PROXIMAL_PHALANX, 0, 0, currentGuess.x);
	T2 = DenavitHartenbergMatrix(LENGTH_INTERMEDIATE_PHALANX, 0, 0, currentGuess.x + currentGuess.y);
	T3 = DenavitHartenbergMatrix(LENGTH_DISTAL_PHALANX, 0, 0, currentGuess.x + currentGuess.y + 2 * currentGuess.y / 3);

	//Update tip position
	L0 = glm::vec4(glm::vec3(0.0), 1.0);
	O1 = L0;
	O2 = O1 + L0 * T1;
	O3 = O2 + L0 * T2;
	T = O3 + L0 * T3; //Tip position

	while (glm::length(target - glm::vec3(T.x, T.y, T.z)) > 0.011f)
	{
		if (i > 1000 - 1)
			break;

		auto J = JacobiMatrix(
			currentGuess[0],
			currentGuess[1]
			//std::max(thetaConstraints[0][0], std::min(previousGuess[0], thetaConstraints[0][1])),
			//std::max(thetaConstraints[1][0], std::min(previousGuess[1], thetaConstraints[1][1]))
			);

		//previousGuess = (previousGuess + halfConstraints) * 0.5f;


		float det = J[0][0] * J[1][1] - J[0][1] * J[1][0];
		auto J_psuedoInverse = glm::inverse(glm::transpose(J) * J) * glm::transpose(J);

		//(1 / det) * glm::mat2(J[1][1], -J[0][1], -J[1][0], J[0][0]);

		auto residue = target - glm::vec3(T.x, T.y, T.z);
		//glm::vec3 residue = goal - glm::vec3(tip.x, tip.y, tip.z)
		//glm::vec3 residue = glm::vec3(tip.x, tip.y, tip.z) - goal

	//	printf("T \n x: %f, y: %f, z: %f\n", T.x, T.y, T.z);
		//printf("Residue \n x: %f, y: %f, z: %f\n", residue.x, residue.y, residue.z);
		currentGuess = previousGuess + 0.1f * J_psuedoInverse * residue;
		currentGuess.x = std::max(thetaConstraints[0][0], std::min(currentGuess.x, thetaConstraints[0][1]));
		currentGuess.y = std::max(thetaConstraints[1][0], std::min(currentGuess.y, thetaConstraints[1][1]));
		//currentGuess = (currentGuess + halfConstraints) * 0.5f;


		//printf("Guesses \n theta1: %f, theta2: %f, theta3: %f\n", theta[0], theta[1], theta[2]);

		//Update forward kinematics
		T1 = DenavitHartenbergMatrix(LENGTH_PROXIMAL_PHALANX, 0, 0, currentGuess.x);
		T2 = DenavitHartenbergMatrix(LENGTH_INTERMEDIATE_PHALANX, 0, 0, currentGuess.x + currentGuess.y);
		T3 = DenavitHartenbergMatrix(LENGTH_DISTAL_PHALANX, 0, 0, currentGuess.x + currentGuess.y + 2 * currentGuess.y / 3);

		//Update tip position
		L0 = glm::vec4(glm::vec3(0.0), 1.0);
		O1 = L0;
		O2 = O1 + L0 * T1;
		O3 = O2 + L0 * T2;
		T = O3 + L0 * T3; //Tip position

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

//	float t3 = theta[2];

	float t1t2 = t1 + t2;
	//float t1t2t3 = t1 + t2 + t3;


	return glm::mat2(
		-l1 * sinf(t1) - l2 * sinf(t1t2) - l3 * sinf(t1 + 2 * t2 / 3),
		-l2 * sinf(t1t2) - l3 * (2.0f/3.0f) * sinf(t1 + 2 * t2 / 3),
		l1 * cosf(t1) + l2 * cosf(t1t2) + l3 * cosf(t1 + 2 * t2 / 3), 
		l2 * cosf(t1t2) + l3 * (2.0f/3.0f) * cosf(t1 + 2 * t2 / 3));
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