#ifndef ControlMotor_h
#define ControlMotor_h

struct PonteH
{
int pin_motor_left_1; 
int pin_motor_left_2; 

int pin_motor_right_1; 
int pin_motor_right_2; 

int STBY;

int pin_speed_motor_left;
int pin_speed_motor_right;
};

class ControlMotor{
  private:
  int pin_motor_left_1, pin_motor_right_1, pin_motor_left_2, pin_motor_right_2;
  int pin_motor_left_pwm, pin_motor_right_pwm;
  int pin_STBY;

  public:
  ControlMotor(PonteH _pins); //constructor
  void setPWM(int _pwm_left, int _pwm_right);
  void goForward();
  void TurnLeft();
  void TurnRight();
  void Off();

};


#endif