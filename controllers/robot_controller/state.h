// File:          	state.h
// Date: 			27/01/2021
// Author: 			Edmund Prager, Richard Monteiro
// Modifications: 	

#ifndef STATE
#define STATE

#include "header.h"
#include "navigation.h"

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

// parent class for temporary states, these remain active until there is a state in the queue to switch to
class TemporaryState : public State {
public:
	TemporaryState(Navigation *_nav) : State(_nav) {}
};

// special default state - only created once, and is reverted back to when has no other state to go to
class DefaultState : public TemporaryState {
public:
	DefaultState(Navigation *_nav) : TemporaryState(_nav){
		printf("Default state created.\n");
	}
	
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

// waits a number of seconds given as an initialiser parameter
class WaitState : public State {
public:
	WaitState(Navigation *_nav, float _time);
	
	State *Run() override;
	
private:
	float time;
	int stepsLeft;
};

// double checks bearing reading by moving forward and comparing GPS reading before and after
class DCheckingState : public State {
public:
	DCheckingState(Navigation *_nav);
	
	State *Run() override;
	
private:
	int stepsLeft; // time steps left
	const float time = 0.5; // seconds, how long double checking takes
	vec startPos; // position at start of double check
};

// does an initial scan of the arena
class InitialScanState : public InputState {
public:
	InitialScanState(Navigation *_nav);
	
	State *Run() override;
};

// moves toward destination location as long as it can see the block at the location, feeds readings back to database to update destination position
class MovingToState : public State {
public:
	MovingToState(Navigation *_nav);
	
	State *Run() override;
};

// wiggles back and forth, looking for a block at an expected distance away
class FindingLostState : public State {
public:
	FindingLostState(Navigation *_nav);
	
	State *Run() override;
	
private:
	bool turningRight; // which way are we wiggling
	double turnTo; // how far out are we wiggling to
	const double limit = 0.16f;
};

// evaluates colour of block in front, if there is one, and closes claw around the block
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

// wiggles back and forth looking for a block right in front of it
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
	const double limit = 0.16f;
};

// raises arm once block is grabbed in claw
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

// lowers arm
class LoweringState : public State {
public:
	LoweringState(Navigation *_nav);
	
	State *Run() override;
	
private:
	const double upperAngle = 0.8; // radians above horizontal
	const float time = 1; // seconds to go from upper angle to horizontal
	double radPerStep;
	unsigned int count;
};

// moves back to the starting square to deposit a collected block
class ReturningState : public State {
public:
	ReturningState(Navigation *_nav);
	
	State *Run() override;
};

// releases claw
class DroppingState : public State {
public:
	DroppingState(Navigation *_nav);
	
	State *Run() override;
	
private:
	const double closedWidth = 0.04; // width of closed claw
	const float time = 0.2f; // seconds to open
	double mPerStep;
	unsigned int count;
};

// reverses a short distance
class BackingState : public State {
public:
	BackingState(Navigation *_nav);
	
	State *Run() override;
	
private:
	const float time = 0.75; // seconds to reverse for
	int steps;
	unsigned int count;
};


// ----- State Manager -----

// manages state objects, the state queue and progression between states
class StateManager {
public:
	StateManager(Navigation *_nav){
		nav = _nav;
		defaultState = new DefaultState(nav);
		state = defaultState;
		nextState = nullptr;
	}
	~StateManager(){}
	
	// runs the current state saves returned state object for next step
	void Run();
	
	// Forces change of state to '_state'.
	// Note: if '_state' is a temporary state and 'nextState' isn't a 'nullptr', the state will be immediately switched to 'nextState'.
	//		This means, of course, that if you want to immediately skip to whatever state is stored in 'nextState', this can be
	//		achieved by calling this function with any temporary state as the parameter.
	void ForceStateChange(State *_state){
		if(state && state != defaultState) delete state;
		state = _state;
	}
	
	// Sets state that will be switched to once current state series has finished (state goes back to 'nullptr' or a temporary state)
	void SetNextState(State *_nextState){
		if(nextState && nextState != defaultState) delete nextState;
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