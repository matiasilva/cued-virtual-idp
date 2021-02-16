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
	printf("%c: New destination: %f, %f\n", nav->GetC(), nav->GetDestination().z, nav->GetDestination().x);
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
	// have we reached our destination?
	if((nav->PositionInFront() - nav->GetDestination()).SqMag() < 0.0002){
		nav->EndStep(0, 0);
		//printf("%c: Found.\n", names[iAmRed]);
		return new GrabbingState(nav);
	}
	
	// are we close to the other robot - if that is the case, back away for a few steps
	static int steps = 0;
	if ((nav->GetPosition() - nav->GetOtherPos()).SqMag() <= BLOCK_POS_UNCERTAINTY*1.6 || steps != 0) {
		// Set robot to back away while turning - hence getting path away from other robot
		nav->EndStep(-5, -3);
		if (steps == 100) steps = 0;
		else steps++;
		return this;

	}
	
	// are we looking in the right direction?
	vec delta = nav->GetDestination() - nav->GetPosition();
	double targetBearing = delta.Bearing();
	double diff = MakePPMP(targetBearing - nav->GetBearing());
	if(fabs(diff) > 0.1){
		nav->EndStep(-1 + 2*(diff < 0), -1 + 2*(diff > 0));
		return this;
	}
	
	// are our distance sensors looking at the block?
	float dLeft = nav->GetDistance(0);
	float dRight = nav->GetDistance(1);
	float expected = sqrt(delta.SqMag());
	bool leftCons = fabs(expected - dLeft) < 0.1f;
	bool rightCons = fabs(expected - dRight) < 0.1f;
	int cons = (leftCons + rightCons)*(fabs(dLeft - dRight) < 0.05f);
	if(cons){
		if(cons == 2){
			// we are looking at the block
			nav->DBLogReading(true, true);
			nav->EndStep(5, 5);
			return this;
		}
		// only one side is looking at the block
		nav->EndStep(2 + 2*rightCons, 2 + 2*leftCons);
		return this;
	}
	// we aren't looking at the block
	if(0.5*(dLeft + dRight) < 0.13){ // close to something
		return new BackingState(nav);
	}
	nav->EndStep(0, 0);
	return new FindingLostState(nav);
}


FindingLostState::FindingLostState(Navigation *_nav) : State(_nav){
	printf("%c: State changed to 'FindingLostState'\n", nav->GetC());
	turningRight = false;
	turnTo = 0.08;
}
State *FindingLostState::Run(){
	nav->DBLogReading(false);
	vec delta = nav->GetDestination() - nav->GetPosition();
	float expected = sqrt(delta.SqMag());
	float dLeft = nav->GetDistance(0);
	float dRight = nav->GetDistance(1);
	bool leftCons = fabs(expected - dLeft) < 0.07f;
	bool rightCons = fabs(expected - dRight) < 0.07f;
	bool cons = (leftCons + rightCons)*(fabs(dLeft - dRight) < 0.05f);
	double expectedBearing = delta.Bearing();
	if(!cons){
		if(turnTo >= limit){
			nav->DestinationInvalid();
			nav->EndStep(0, 0);
			return nullptr;
		}
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
		nav->DBLogReading(true, true);
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
			Colour blockColour = nav->ReadCamera();
			if(nav->IAmRed() + 1 != (int)blockColour) return new BackingState(nav);
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
	if(0.5*(nav->GetDistance(0) + nav->GetDistance(1)) > 0.15){
		if(turnTo >= limit){
			nav->DestinationInvalid();
			nav->EndStep(0, 0);
			return nullptr;
		}
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
	if(done){
		if(nav->GetDistance(0) > 0.08 && nav->GetDistance(1) > 0.08){
			nav->Got();
			return new ReturningState(nav);
		}
		return nullptr;
	}
	return this;
}

LoweringState::LoweringState(Navigation *_nav) : State(_nav){
	printf("%c: State changed to 'LoweringState'\n", nav->GetC());
	radPerStep = upperAngle / (time*1000/nav->GetTS());
	count = 0;
}
State *LoweringState::Run(){
	double newAngle = upperAngle - radPerStep * ++count;
	bool done = false;
	if(newAngle <= 0){
		newAngle = 0;
		done = true;
	}
	nav->SetArmAngle(newAngle);
	nav->EndStep(0, 0);
	if(done) return nullptr;
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
	
	nav->EndStep(5, 5);
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
	if(done){
		nav->GetStateManager()->SetNextState(new LoweringState(nav));
		return new BackingState(nav);
	}
	return this;
}

BackingState::BackingState(Navigation *_nav) : State(_nav){
	printf("%c: State changed to 'BackingState'\n", nav->GetC());
	steps = time*1000/nav->GetTS();
	count = 0;
}
State *BackingState::Run(){
	nav->EndStep(-3, -3);
	if(count++ > steps) return nullptr;
	return this;
}