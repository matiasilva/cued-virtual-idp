#ifndef VISUALISER
#define VISUALISER

#include "header.h"
#include "database.h"

// handles creating and drawing to a window using the SDL library
class Visualiser {
public:
	Visualiser(int _width, int _height, DataBase *_dataBase);
	~Visualiser();
	
	// initialises the SDL window and renderer
	bool Init();
	
	// is called in the main loop every step
	void Loop();
	
private:
	
	DataBase *dataBase;
	
	int width, height;
	
	// calls render method in the database, passing it the renderer to render onto
	void Render();

	SDL_Window *window;
	SDL_Renderer *renderer;
	SDL_Event event;
};

#endif