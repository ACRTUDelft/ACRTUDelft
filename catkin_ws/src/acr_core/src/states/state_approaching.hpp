#ifndef State_Approaching_H
#define State_Approaching_H

class State_Approaching : public State {	
  public:  
	State_Approaching();
	
	State* update() override;
    
    void switchTo() override;
    
    void switchFrom() override;
};
#endif
