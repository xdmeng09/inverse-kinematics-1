// inverse-kinematics.cpp : Defines the entry point for the console application.
//
#include "stdafx.h"
// C++ Headers
#include <string>
#include <iostream>

// OpenGL / glew Headers
#define GL3_PROTOTYPES 1
#include <GL/glew.h>

// SDL2 Headers
#include "SDL.h"
#include "demo.h"

// Our SDL_Window ( just like with SDL2 wihout OpenGL)
SDL_Window *mainWindow;

// Our opengl context handle
SDL_GLContext mainContext;

Demo demo;

// Window definition
std::string programName = "Motion & Manipulation Solver";
#define SCREEN_WIDTH 1200
#define SCREEN_HEIGHT 800

bool SetOpenGLAttributes();
void PrintSDL_GL_Attributes();
void CheckSDLError(int line);
void RunGame();
void Cleanup();

bool Init()
{

	// Initialize SDL's Video subsystem
	if (SDL_Init(SDL_INIT_VIDEO) < 0)
	{
		std::cout << "Failed to init SDL\n";
		return false;
	}

	// Create our window centered at 512x512 resolution
	mainWindow = SDL_CreateWindow(programName.c_str(), SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
		SCREEN_WIDTH, SCREEN_HEIGHT, SDL_WINDOW_OPENGL);

	// Check that everything worked out okay
	if (!mainWindow)
	{
		std::cout << "Unable to create window\n";
		CheckSDLError(__LINE__);
		return false;
	}

	// Create our opengl context and attach it to our window
	mainContext = SDL_GL_CreateContext(mainWindow);

	TTF_Init();

	SetOpenGLAttributes();

	// This makes our buffer swap syncronized with the monitor's vertical refresh
	SDL_GL_SetSwapInterval(1);

	// Init GLEW
	// Apparently, this is needed for Apple. Thanks to Ross Vander for letting me know
#ifndef __APPLE__
	glewExperimental = GL_TRUE;
	glewInit();
#endif
	glEnable(GL_MULTISAMPLE);
	glEnable(GL_LINE_SMOOTH);
	glEnable(GL_POLYGON_SMOOTH);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	return true;
}

bool SetOpenGLAttributes()
{

	// Set our OpenGL version.
	// SDL_GL_CONTEXT_CORE gives us only the newer version, deprecated functions are disabled
	SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);

	// 3.2 is part of the modern versions of OpenGL, but most video cards whould be able to run it
	SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
	SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 2);

	// Turn on double buffering with a 24bit Z buffer.
	// You may need to change this to 16 or 32 for your system
	SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);

	return true;
}

int main(int argc, char *argv[])
{
	if (!Init())
		return -1;

	demo = Demo(&mainContext, mainWindow);

	RunGame();

	Cleanup();

	return 0;
}

void RunGame()
{
	bool loop = true;

	while (loop)
	{
		demo.HandleInput();
		demo.Tick();
		demo.Draw();
		SDL_GL_SwapWindow(mainWindow);
		demo.SDLDraw();
		// Swap our back buffer to the front
		// This is the same as :
		// 		SDL_RenderPresent(&renderer);
	}
}

void Cleanup()
{
	// Delete our OpengL context
	SDL_GL_DeleteContext(mainContext);

	// Destroy our window
	SDL_DestroyWindow(mainWindow);

	// Shutdown SDL 2
	SDL_Quit();
}

void CheckSDLError(int line = -1)
{
	std::string error = SDL_GetError();

	if (error != "")
	{
		std::cout << "SLD Error : " << error << std::endl;

		if (line != -1)
			std::cout << "\nLine : " << line << std::endl;

		SDL_ClearError();
	}
}

void PrintSDL_GL_Attributes()
{
	int value = 0;
	SDL_GL_GetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, &value);
	std::cout << "SDL_GL_CONTEXT_MAJOR_VERSION : " << value << std::endl;

	SDL_GL_GetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, &value);
	std::cout << "SDL_GL_CONTEXT_MINOR_VERSION: " << value << std::endl;
}