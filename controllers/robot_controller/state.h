// File:          	state.h
// Date: 			27/01/2021
// Author: 			Edmund Prager, Richard Monteiro
// Modifications: 	

#ifndef STATE
#define STATE

#include "header.h"
#include "navigation.h"

// state's run function is called
// it executes code for this step
// returns pointer to next state, either itself, another or 'nullptr' to go back to the default state

// Current state system states
// 'DefaultState':			=Temporary State= Asking the database for a destination. Will switch to 'movingTo' upon receiving one.
//							Otherwise will turn slowly, scanning information for the database
// 'DCheckingState':		Moving straight forwards from a remembered initial position. Re-calibrates bearing
//							after 'dCTime' seconds, then goes back to 'def'
// 'InitialScanState':		Scans a bit of 90 degrees downwards, logging questions into the database
//							('canConfirm' = 'false'). Switches to 'def' when finished
// 'MovingToState':			Turns towards, then moves towards destination. Will switch to 'findingLost' if
//							distances sensor reading becomes unexpected
// 'FindingLostState':		Wiggles left and right to increasing angles until distance sensor reads distance
//							to destination !!! does not have a way of getting out of this is if it never reads
//							the right thing
// 'DoNothingState':		=Temporary State= Does nothing.
// 'WaitState(float time)':	Waits for 'time' seconds

// ----- States -----

// parent class
class State {
public:
	State(Navigation *_nav){
		nav = _nav;
	}
	virtual ~State(){}
	
	virtual State *Run(){
		return nullptr;
	}
	
protected:
	Navigation *nav;
};

// parent class for temporary states
class TemporaryState : public State {
public:
	TemporaryState(Navigation *_nav) : State(_nav) {}
};

// special default state - only created once, and is reverted back to when has no other state to go to
class DefaultState : public TemporaryState {
public:
	DefaultState(Navigation *_nav) : TemporaryState(_nav){}
	
	State *Run() override;
};

// type of state where robot is acquiring new block data from its sensors - hence receiving communication data and modification is locked.
class InputState : public State {
public:
	InputState(Navigation* _nav) : State(_nav) {}
};


// ----- Other temporary states -----

class DoNothingState : public TemporaryState {
public:
	DoNothingState(Navigation *_nav);
	
	State *Run() override;
};


// ----- Normal states -----

class WaitState : public State {
public:
	WaitState(Navigation *_nav, float _time);
	
	State *Run() override;
	
private:
	float time;
	int stepsLeft;
};

class DCheckingState : public State {
public:
	DCheckingState(Navigation *_nav);
	
	State *Run() override;
	
private:
	int stepsLeft; // time steps left
	const float time = 0.5; // seconds, how long double checking takes
	vec startPos; // position at start of double check
};

class InitialScanState : public InputState {
public:
	InitialScanState(Navigation *_nav);
	
	State *Run() override;
};

class MovingToState : public State {
public:
	MovingToState(Navigation *_nav);
	
	State *Run() override;
};

class FindingLostState : public State {
public:
	FindingLostState(Navigation *_nav);
	
	State *Run() override;
	
private:
	bool turningRight; // which way are we wiggling
	double turnTo; // how far out are we wiggling to
};

class GrabbingState : public State {
public:
	GrabbingState(Navigation *_nav);
	
	State *Run() override;
	
private:
	const double closedWidth = 0.04; // width of closed claw
	const float time = 1; // seconds to close
	double mPerStep;
	unsigned int count;
	bool grabbing;
};

class FindingCloseState : public State {
public:
	FindingCloseState(Navigation *_nav);
	
	State *Run() override;
	
private:
	double expectedBearing;
	bool turningRight; // which way are we wiggling
	double turnTo; // how far out are we wiggling to
	double target;
	bool toTarget;
};

class PickingUpState : public State {
public:
	PickingUpState(Navigation *_nav);
	
	State *Run() override;
	
private:
	const double upperAngle = 0.8; // radians above horizontal
	const float time = 1; // seconds to go from horizontal to upper angle
	double radPerStep;
	unsigned int count;
};

class ReturningState : public State {
public:
	ReturningState(Navigation *_nav);
	
	State *Run() override;
};

class DroppingState : public State {
public:
	DroppingState(Navigation *_nav);
	
	State *Run() override;
	
private:
	const double closedWidth = 0.04; // width of closed claw
	const float time = 1; // seconds to open
	double mPerStep;
	unsigned int count;
};

// ----- State Manager -----

class StateManager {
public:
	StateManager(Navigation *_nav){
		nav = _nav;
		defaultState = new DefaultState(nav);
		state = defaultState;
		nextState = nullptr;
	}
	~StateManager(){}
	
	void Run(){
		if(!state){
			state = defaultState;
		}
		if(dynamic_cast<TemporaryState*>(state)){
			if(nextState){
				if(state != defaultState) delete state;
				state = nextState;
				nextState = nullptr;
			}
		}
		State *nextStepState = state->Run();
		if(nextStepState != state){
			if(state != defaultState) delete state;
			state = nextStepState;
		}
	}
	
	// Forces change of state to '_state'.
	// Note: if '_state' is a temporary state and 'nextState' isn't a 'nullptr', the state will be immediately switched to 'nextState'.
	//		This means, of course, that if you want to immediately skip to whatever state is stored in 'nextState', this can be
	//		achieved by calling this function with any temporary state as the parameter.
	void ForceStateChange(State *_state){
		state = _state;
	}
	
	// Sets state that will be switched to once current state series has finished (state goes back to 'nullptr' or a temporary state)
	void SetNextState(State *_nextState){
		nextState = _nextState;
	}

	// Get state
	State* GetState() {
		return state;
	}
	
private:
	Navigation *nav;
	State *state;
	State *nextState;
	State *defaultState;
};

#endif