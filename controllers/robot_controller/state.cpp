// File:          	state.cpp
// Date: 			27/01/2021
// Description: 	
// Author: 			Edmund Prager, Richard Monteiro
// Modifications: 	None

#include "state.h"

DoNothingState::DoNothingState(Navigation *_nav) : TemporaryState(_nav){
	printf("%c: State changed to 'DoNothingState'\n", nav->GetC());
}
State *DoNothingState::Run(){
	nav->EndStep(0, 0);
	return this;
}


State *DefaultState::Run(){
	if(!nav->DBGetDestination()){ // no destination to be given
		if(!nav->DBLogReading(true)){
			nav->EndStep(0, 0);
			return new DCheckingState(nav);
		}
		nav->EndStep(-1, 1); // rotate slowly to scan for blocks
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


DCheckingState::DCheckingState(Navigation *_nav) : State(_nav){
	printf("%c: State changed to 'DCheckingState'\n", nav->GetC());
	stepsLeft = time*1000/nav->GetTS();
	startPos = nav->GetPosition();
}
State *DCheckingState::Run(){
	stepsLeft--;
	if(stepsLeft <= 0){
		nav->SetBearing((nav->GetPosition() - startPos).Bearing());
		//printf("%c: Back to default.\n", names[iAmRed]);
		nav->EndStep(0, 0);
		return nullptr;
	}
	nav->EndStep(4, 4);
	return this;
}


InitialScanState::InitialScanState(Navigation *_nav) : State(_nav){
	printf("%c: State changed to 'InitialScanState'\n", nav->GetC());
}
State *InitialScanState::Run(){
	nav->DBLogReading(false);
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
	printf("%c: State changed to 'MovingToState'\n", nav->GetC());
}
State *MovingToState::Run(){
	if((nav->PositionInFront() - nav->GetDestination()).SqMag() < 0.0004){
		nav->EndStep(0, 0);
		//printf("%c: Found.\n", names[iAmRed]);
		return new DoNothingState(nav);
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
	if(off > 0.05){
		nav->EndStep(0, 0);
		return new FindingLostState(nav);
	} else {
		nav->DBLogReading(false);
		nav->SetBearing(targetBearing);
		nav->EndStep(4, 4);
		return this;
	}
}


FindingLostState::FindingLostState(Navigation *_nav) : State(_nav){
	printf("%c: State changed to 'FindingLostState'\n", nav->GetC());
	turningRight = false;
	turnTo = 0.08;
}
State *FindingLostState::Run(){
	vec delta = nav->GetDestination() - nav->GetPosition();
	float expected = sqrt(delta.SqMag());
	float off = fabs(expected - nav->GetDistance(0));
	double expectedBearing = delta.Bearing();
	if(off > 0.05){
		if(turningRight){
			if(MakePPMP(nav->GetBearing() - (expectedBearing - turnTo)) < 0){
				turningRight = false;
				turnTo += 0.08;
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