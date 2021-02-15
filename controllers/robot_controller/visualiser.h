#ifndef VISUALISER
#define VISUALISER

#include "header.h"
#include "database.h"

class Visualiser {
public:
	Visualiser(int _width, int _height, DataBase *_dataBase);
	~Visualiser();
	bool Init();
	
	void Loop();
	
private:
	
	DataBase *dataBase;
	
	int width, height;
	
	void Render();

	SDL_Window *window;
	SDL_Renderer *renderer;
	SDL_Event event;
};

#endif