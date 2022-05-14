#ifndef LEDRGB_h
#define LEDRGB_h

class LedRGB{
  private:
  int pin_r, pin_g, pin_b;
  public:
  LedRGB(int _pin_r, int _pin_g, int _pin_b);
  void setR();
  void setG();
  void setB();
  void Off();
};

#endif

