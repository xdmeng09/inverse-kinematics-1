#include "stdafx.h"
#include "demo.h"

void Demo::Reset()
{

};

void Demo::HandleInput()
{
	SDL_Event event;
	while (SDL_PollEvent(&event))
	{
		if (event.type == SDL_QUIT)
			break;

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
	T3 = DenavitHartenbergMatrix(LENGTH_DISTAL_PHALANX, 0, 0, theta[0] + theta[1] + theta[2]);

	//Calculate current positions 
	auto L0 = glm::vec4(glm::vec3(0.0), 1.0);
	auto O1 = L0;
	auto O2 = O1 + L0 * T1;
	auto O3 = O2 + L0 * T2;
	auto T = O3 + L0 * T3; //Tip position

	int i = 0;
	auto previousGuess = glm::vec3(0.0f, 0.0f, 0.0f);
	auto currentGuess = glm::vec3(theta[0], theta[1], theta[2]);

	while (i < 1000)
	{
		if (glm::length(glm::vec3(T.x, T.y, T.z) - target) < 0.001f) 
			break;

		auto J = JacobiMatrix();
		auto J_i = glm::transpose(J) * glm::inverse(J * glm::transpose(J));
		auto residue = glm::vec3(T.x, T.y, T.z) - target;

		//printf("T \n x: %f, y: %f, z: %f\n", T.x, T.y, T.z);
		//printf("Residue \n x: %f, y: %f, z: %f\n", residue.x, residue.y, residue.z);

		currentGuess = previousGuess + J_i * residue;

		//printf("Guesses \n theta1: %f, theta2: %f, theta3: %f\n", theta[0], theta[1], theta[2]);

		//Update forward kinematics
		T1 = DenavitHartenbergMatrix(LENGTH_PROXIMAL_PHALANX, 0, 0, currentGuess.x);
		T2 = DenavitHartenbergMatrix(LENGTH_INTERMEDIATE_PHALANX, 0, 0, currentGuess.x + currentGuess.y);
		T3 = DenavitHartenbergMatrix(LENGTH_DISTAL_PHALANX, 0, 0, currentGuess.x + currentGuess.y + currentGuess.z);

		//Update tip position
		L0 = glm::vec4(glm::vec3(0.0), 1.0);
		O1 = L0;
		O2 = O1 + L0 * T1;
		O3 = O2 + L0 * T2;
		T = O3 + L0 * T3; //Tip position

		previousGuess = currentGuess;
		i++;
	}

	theta[0] = currentGuess.x;
	theta[1] = currentGuess.y;
	theta[2] = currentGuess.z;

};

void Demo::Draw()
{
	//Clear BG color
	glClearColor(0, 0, 0, 1.0);
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
	DrawArc(O3.x, O3.y, LENGTH_INTERMEDIATE_PHALANX * 0.5f, theta[0] + theta[1], theta[2], 20);

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

	glColor3f(1.0f, 0.0f, 1.0f);
	DrawCross(target.x, target.y, target.z, VIS_JOINT_RADIUS);

	glFlush();
};

glm::mat4 Demo::DenavitHartenbergMatrix(float a, float alpha, float d, float theta)
{
	return glm::mat4(
		cosf(theta), -sinf(theta) * cosf(alpha), sinf(theta) * sinf(alpha), a * cosf(theta),
		sinf(theta), cosf(theta) * cosf(alpha), -cosf(theta) * sinf(alpha), a * sinf(theta),
		0, sinf(alpha), cosf(alpha), d,
		0, 0, 0, 1);
}

glm::mat3x2 Demo::JacobiMatrix()
{
	float l1 = LENGTH_PROXIMAL_PHALANX;
	float l2 = LENGTH_INTERMEDIATE_PHALANX;
	float l3 = LENGTH_DISTAL_PHALANX;

	return glm::mat3x2(
		-l1 * sinf(theta[0]) - l2 * sinf(theta[0] + theta[1]) - l3 * sinf(theta[0] + theta[1] + theta[2]), -l2 * sinf(theta[0] + theta[1]) - l3*sinf(theta[0] + theta[1] + theta[2]), l3*sinf(theta[0] + theta[1] + theta[2]),
		l1 * cosf(theta[0]) + l2 * cosf(theta[0] + theta[1]) + l3 * cosf(theta[0] + theta[1] + theta[2]), l2 * cosf(theta[0] + theta[1]) + l3 * cosf(theta[0] + theta[1] + theta[2]), l3 * cosf(theta[0] + theta[1] + theta[2]));
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