// File:          	state.cpp
// Date: 			27/01/2021
// Description: 	
// Author: 			Edmund Prager, Richard Monteiro
// Modifications: 	None

#include "state.h"

InsertedState::InsertedState(Navigation *_nav, State *_returnTo) : State(_nav){
	returnTo = _returnTo;
}


DoNothingState::DoNothingState(Navigation *_nav) : TemporaryState(_nav){
	printf("%c: State changed to 'DoNothingState'\n", nav->GetC());
	RGB cam = nav->ReadCamera();
	printf("red = %d, green = %d, blue = %d\n", cam.r, cam.g, cam.b);
	if(cam.Red()) printf("I think this is red.\n");
	if(cam.Blue()) printf("I think this is blue.\n");
	if(!cam.Red() && !cam.Blue()) printf("I think this isn't a block.\n");
}
State *DoNothingState::Run(){
	nav->EndStep(0, 0);
	return this;
}


State *DefaultState::Run(){
	if(!nav->DBGetDestination()){ // no destination to be given
		nav->DBLogReading();
		nav->EndStep(2, -2);
		return this;
	}
	// can have a new destination
	//printf("%c: New destination: %f, %f\n", names[iAmRed], destination.z, destination.x);
	nav->EndStep(0, 0);
	return new MovingToState(nav);
}


WaitState::WaitState(Navigation *_nav, float _time) : State(_nav){
	printf("%c: State changed to 'WaitState'\n", nav->GetC());
	time = _time;
	stepsLeft = time*1000/nav->GetTS();
}
State *WaitState::Run(){
	stepsLeft--;
	if(stepsLeft <= 0){
		nav->EndStep(0, 0);
		return nullptr;
	}
	nav->EndStep(0, 0);
	return this;
}


DCheckingState::DCheckingState(Navigation *_nav, State *_returnTo) : InsertedState(_nav, _returnTo){
	printf("%c: State changed to 'DCheckingState'\n", nav->GetC());
	stepsLeft = time*1000/nav->GetTS();
	startPos = nav->GetPosition();
}
State *DCheckingState::Run(){
	stepsLeft--;
	if(stepsLeft <= 0){
		nav->SetBearing((nav->GetPosition() - startPos).Bearing());
		//printf("%c: Back to previous.\n", names[iAmRed]);
		nav->EndStep(0, 0);
		return returnTo;
	}
	nav->EndStep(4, 4);
	return this;
}


InitialScanState::InitialScanState(Navigation *_nav) : State(_nav){
	printf("%c: State changed to 'InitialScanState'\n", nav->GetC());
}
State *InitialScanState::Run(){
	nav->DBLogReading();
	if(nav->IAmRed()){
		if(nav->GetBearing() > -1.22 && nav->GetBearing() < 0){
			//printf("%c: Back to default.\n", names[iAmRed]);
			nav->EndStep(0, 0);
			return nullptr;
		}
		nav->EndStep(-1, 1);
		return this;
	} else {
		if(nav->GetBearing() < -1.92){
			//printf("%c: Back to default.\n", names[iAmRed]);
			nav->EndStep(0, 0);
			return nullptr;
		}
		nav->EndStep(1, -1);
		return this;
	}
}


MovingToState::MovingToState(Navigation *_nav) : State(_nav){
	printf("%c: State changed to 'MovingToState', destination %f, %f\n", nav->GetC(), nav->GetDestination().z, nav->GetDestination().x);
	counter = stepsPerRead;
}
State *MovingToState::Run(){
	if((nav->PositionInFront() - nav->GetDestination()).SqMag() < 0.0004){
		nav->EndStep(0, 0);
		printf("%c: Found.\n", nav->GetC());
		return new ColourDeterminingState(nav);
	}
	
	vec delta = nav->GetDestination() - nav->GetPosition();
	double targetBearing = delta.Bearing();
	double diff = MakePPMP(targetBearing - nav->GetBearing());
	if(fabs(diff) > 0.1){
		nav->EndStep(-1 + 2*(diff < 0), -1 + 2*(diff > 0));
		return this;
	}
	
	float expected = sqrt(delta.SqMag());
	float off = fabs(expected - nav->GetDistance(0));
	if(off > 0.07){
		printf("Lost! exp:%f  dist:%f  off:%f\n", expected, nav->GetDistance(0), off);
		nav->EndStep(0, 0);
		return new FindingLostState(nav);
	} else {
		if(--counter <= 0){
			counter = stepsPerRead;
			nav->DBLogReading();
		}
		nav->EndStep(4, 4);
		return this;
	}
}


FindingLostState::FindingLostState(Navigation *_nav) : State(_nav){
	printf("%c: State changed to 'FindingLostState'\n", nav->GetC());
	turningRight = false;
	turnTo = 0.2;
}
State *FindingLostState::Run(){
	vec delta = nav->GetDestination() - nav->GetPosition();
	float expected = sqrt(delta.SqMag());
	float off = fabs(expected - nav->GetDistance(0));
	double expectedBearing = delta.Bearing();
	if(off > 0.05){
		if(fabs(turnTo) > 1.6) return new DCheckingState(nav, this);
		if(turningRight){
			if(MakePPMP(nav->GetBearing() - (expectedBearing - turnTo)) < 0){
				turningRight = false;
				turnTo += 0.3;
				nav->EndStep(0, 0);
			} else {
				nav->EndStep(1, -1);
			}
		} else {
			if(MakePPMP(nav->GetBearing() - (expectedBearing + turnTo)) > 0){
				turningRight = true;
				nav->EndStep(0, 0);
			} else {
				nav->EndStep(-1, 1);
			}
		}
		return this;
	} else {
		nav->SetBearing(expectedBearing);
		nav->EndStep(0, 0);
		return new MovingToState(nav);
	}
}

ColourDeterminingState::ColourDeterminingState(Navigation *_nav) : State(_nav) {
	printf("%c: State changed to 'ColourDeterminingState'\n", nav->GetC());
}
State *ColourDeterminingState::Run(){
	RGB cam = nav->ReadCamera();
	bool blockIsRed;
	if(cam.Blue()){
		if(cam.Red()){
			printf("%c: I think it's both.\n", nav->GetC());
			return new DoNothingState(nav);
		}
		blockIsRed = false;
	} else if(cam.Red()){
		blockIsRed = true;
	} else {
		printf("%c: I think it's neither.\n", nav->GetC());
		return new DoNothingState(nav);
	}
	nav->DBLogReading(blockIsRed ? red : blue);
	return nullptr;
}