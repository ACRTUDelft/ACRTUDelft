#ifndef State_Approaching_H
#define State_Approaching_H

#include "state_navigating.hpp"

/* This state is part of the Navigating superbehaviour
 * State approaching is the state that entered when the robot sees a person while navigating
 * When the person is in front of you, start interacting with it
 */
class State_Approaching : public State_Navigating {	
  public:  
	State_Approaching();

	State* update() override;

	void switchTo() override;

	void switchFrom() override;
};
#endif
