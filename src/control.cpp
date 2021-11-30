#include "control.h"
#include "chasis.h"

using namespace vex;

//Voids for auton 

void setBar(double degs){
  float kP = 1;
  while(true){
    float error = degs - BarRot.position(deg);
    float mtrpwr = error*kP;

    Bar.spin(fwd, mtrpwr, pct);
    if (error <=1){
      Bar.stop(hold);
      break;
    }
  }
}

void barThread(double degs){
  thread([](void *state) {
    double degs = *((double*)state);
    
    void (*bar_ptr)(double) = &setBar;
    (*bar_ptr)(degs);
  }, (void*)(&degs)).detach();
}

void barT(double t){
  if(t>0){
    Bar.spin(fwd,100,pct);
    vex::task::sleep(t*1000);
    Bar.stop(hold);\
  } else{    
    Bar.spin(reverse,100,pct);
    vex::task::sleep(-t*1000);
    Bar.stop(hold);
    
  }
}


void setMogo(double degs){
float kP = 1;
  while(true){
    float error = degs - MogoRot.position(deg);
    float mtrpwr = error*kP;

    RearMogo.spin(fwd, mtrpwr, pct);
    if (error <=1){
      RearMogo.stop(hold);
      break;
    }

  }
}

void mogoThread(double degs){
  thread([](void *state) {
    double degs = *((double*)state);
    
    void (*mogo_ptr)(double) = &setMogo;
    (*mogo_ptr)(degs);
  }, (void*)(&degs)).detach();
}

void mogoPos(int pos, bool daemon){
  float posUp = 188; //All the way up
  float posRing = 102; //position of ring
  if (daemon){ //Backround 
    switch (pos){
      case 1: //All the way up
        mogoThread(posUp);
        break;
      case 2: //Ring height
        thread([]() {
          setMogo(102);
        }).detach();
        break;
      case 3: //All the way down
        mogoThread(0);
        break;
    }
  }else{ //Not backround
    switch (pos){
      case 1: //All the way up
        setMogo(posUp);
        break;
      case 2: //Ring height
        setMogo(posRing);
        break;
      case 3: //All the way down
        setMogo(0);
        break;
    }
  }
}

void spinIntake(){
  Intake.spin(fwd, 100, pct);
}

void stopIntake(){
  Intake.stop(coast);
}

void openClaw(){
  clawSpinT(-.6);
}

void closeClaw(){
  clawSpinT(.6);
}

void clawSpinT(float t){
    if(t>0){
    Claw.spin(fwd,80,pct);
    vex::task::sleep(t*1000);
    Claw.stop(hold);
  } else{    
    Claw.spin(reverse,80,pct);
    vex::task::sleep(-t*1000);
    Claw.stop(hold);
  }
}
