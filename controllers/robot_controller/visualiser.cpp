#include "visualiser.h"

Visualiser::Visualiser(int _width, int _height, DataBase *_dataBase){
	width = _width;
	height = _height;
	dataBase = _dataBase;
}
Visualiser::~Visualiser(){
	SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();
}

bool Visualiser::Init(){
	SDL_Init(SDL_INIT_VIDEO);
	
	window = SDL_CreateWindow("Virtual Integrated Design Project Database Visualiser", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, width, height, 0);
	
    if(window == NULL){
        printf("Could not create window: %s\n", SDL_GetError());
        return false;
    }
    
	renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_TARGETTEXTURE);
	
	return true;
}

void Visualiser::Loop(){
	SDL_PollEvent(&event);
		
	Render();
}

void Visualiser::Render(){
	SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
	SDL_RenderClear(renderer);
	
	dataBase->Render(renderer);
	
	SDL_RenderPresent(renderer);
}