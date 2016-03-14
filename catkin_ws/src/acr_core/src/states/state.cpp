#ifndef State_H
#define State_H

#include "ros/ros.h"

/* Class representing an abstact state.
 * The methods print an error message by default.
 */
class State {
  public:
  
	/* Update method called every cycle.
	 * Checks transition conditions.
	 * Returns the new state (can be 'this')
	 */
    virtual State* update() {ROS_WARN("Update() method not implemented!");};
    
    /* Method called when this state becomes the current state.
     * This method is executed after the constructor of this class.
     * Should be preferred over the constructor for init code,
     * as this results in cleaner code.
     */
    virtual void switchTo() {ROS_WARN("SwitchTo() method not implemented!");};
    
    /* Method called when this state is no longer the current state.
     * 
     * NOTE: 'delete this' should not be used here;
     * this is done automaticaly right after this method returns.
     */
    virtual void switchFrom() {ROS_WARN("SwitchFrom() method not implemented!");};   
};

#endif
