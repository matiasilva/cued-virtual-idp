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


InitialScanState::InitialScanState(Navigation *_nav) : InputState(_nav){
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
	if((nav->PositionInFront() - nav->GetDestination()).SqMag() < 0.0002){
		nav->EndStep(0, 0);
		//printf("%c: Found.\n", names[iAmRed]);
		return new GrabbingState(nav);
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
	if(off > 0.1){
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

GrabbingState::GrabbingState(Navigation *_nav) : State(_nav){
	printf("%c: State changed to 'GrabbingState'\n", nav->GetC());
	mPerStep = closedWidth / (time*1000/nav->GetTS());
	grabbing = false;
}
State *GrabbingState::Run(){
	if(grabbing){
		nav->SetArmAngle(0);
		float newDist = 0.14f - (mPerStep * ++count);
		bool done = false;
		if(newDist <= closedWidth){
			newDist = closedWidth;
			done = true;
		}
		nav->SetClawWidth(newDist);
		nav->EndStep(0, 0);
		if(done) return new PickingUpState(nav);
		return this;
	} else {
		if(nav->GetDistance(0) < 0.08){
			count = 0;
			grabbing = true;
			nav->EndStep(0, 0);
		} else if(nav->GetDistance(0) > 0.15){
			nav->EndStep(0, 0);
			return new FindingCloseState(nav);
		} else {
			nav->EndStep(1, 1);
		}
		return this;
	}
}

FindingCloseState::FindingCloseState(Navigation *_nav) : State(_nav){
	printf("%c: State changed to 'FindingCloseState'\n", nav->GetC());
	turningRight = false;
	turnTo = 0.08;
	expectedBearing = nav->GetBearing();
	toTarget = false;
}
State *FindingCloseState::Run(){
	if(toTarget){
		double delta = MakePPMP(target - nav->GetBearing());
		if(fabs(delta) < 0.05){
			nav->EndStep(0, 0);
			return new GrabbingState(nav);
		}
		int d = (delta > 0) - (delta < 0);
		nav->EndStep(-d, d);
		return this;
	}
	if(nav->GetDistance(0) > 0.15){
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
		target = nav->GetBearing() - (turningRight*2 - 1)*0.24f;
		toTarget = true;
		nav->EndStep(0, 0);
		return this;
	}
}

PickingUpState::PickingUpState(Navigation *_nav) : State(_nav){
	printf("%c: State changed to 'PickingUpState'\n", nav->GetC());
	radPerStep = upperAngle / (time*1000/nav->GetTS());
	count = 0;
}
State *PickingUpState::Run(){
	double newAngle = radPerStep * ++count;
	bool done = false;
	if(newAngle >= upperAngle){
		newAngle = upperAngle;
		done = true;
	}
	nav->SetArmAngle(newAngle);
	nav->EndStep(0, 0);
	if(done) return new ReturningState(nav);
	return this;
}

ReturningState::ReturningState(Navigation *_nav) : State(_nav){
	printf("%c: State changed to 'ReturningState'\n", nav->GetC());
	
}
State *ReturningState::Run(){
	if((nav->PositionInFront() - nav->GetInitialPosition()).SqMag() < 0.0004){
		nav->EndStep(0, 0);
		//printf("%c: Found.\n", names[iAmRed]);
		return new DroppingState(nav);
	}
	
	vec delta = nav->GetInitialPosition() - nav->GetPosition();
	double targetBearing = delta.Bearing();
	double diff = MakePPMP(targetBearing - nav->GetBearing());
	if(fabs(diff) > 0.1){
		nav->EndStep(-3 + 6*(diff < 0), -3 + 6*(diff > 0));
		return this;
	}
	
	nav->EndStep(4, 4);
	return this;
}

DroppingState::DroppingState(Navigation *_nav) : State(_nav){
	printf("%c: State changed to 'DroppingState'\n", nav->GetC());
	mPerStep = closedWidth / (time*1000/nav->GetTS());
}
State *DroppingState::Run(){
	float newDist = closedWidth + mPerStep * ++count;
	bool done = false;
	if(newDist >= 0.14f){
		newDist = 0.14f;
		done = true;
	}
	nav->SetClawWidth(newDist);
	nav->EndStep(0, 0);
	if(done) return new WaitState(nav, 2);
	return this;
}