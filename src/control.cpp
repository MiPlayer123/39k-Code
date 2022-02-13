#include "control.h"
#include "chasis.h"

using namespace vex;

//Voids for auton 

void setBar(double degs){
  float kP = 8;
  float error;
  float mtrpwr;
  while(true){
    if(BarRot.position(deg)>degs){
      error = BarRot.position(deg) - degs; 
      mtrpwr = error*kP;
      Bar.spin(reverse, mtrpwr, pct);
    }
    else {
      error = degs - BarRot.position(deg);
      mtrpwr = error*kP;
      Bar.spin(fwd, mtrpwr, pct);
    }
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

float posRing = 102; //position of ring (103.2)
int mogoToggle;

void setMogo(double degs){
float kP = 2.4;
  float error;
  float mtrpwr;
  while(true){
    if(MogoRot.position(deg)>degs){
      error = MogoRot.position(deg) - degs; 
      mtrpwr = error*kP;
      RearMogo.spin(reverse, mtrpwr, pct);
    }
    else {
      error = degs - MogoRot.position(deg);
      mtrpwr = error*(kP+.6);
      RearMogo.spin(fwd, mtrpwr, pct);
    }
    if (error <=1){
      RearMogo.stop(hold);
      break;
    }
  }
}

int mogoHeight(){
  float kP = 1;
  float error;
  float mtrpwr;
  mogoToggle=1;
  while(true){
    while(mogoToggle==1){
      if(MogoRot.position(deg)>posRing){
        error = MogoRot.position(deg) - posRing; 
        mtrpwr = error*kP;
        RearMogo.spin(reverse, mtrpwr, pct);
      }
      else {
        error = posRing - MogoRot.position(deg);
        mtrpwr = error*kP;
        RearMogo.spin(fwd, mtrpwr, pct);
      }
      
    }
    if (error <=1){
      RearMogo.stop(hold);
      break;
    }
    break;
  }
  return 1;
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
  //float timeOut = 2;
  //float startTime=Brain.timer(timeUnits::msec);
  if (daemon){ //Backround 
    switch (pos){
      case 1: //All the way up
        mogoThread(posUp);
        break;
      case 2: //Ring height
        /*while((std::abs(MogoRot.position(deg) - posRing)>.5)){ // && ((Brain.timer(timeUnits::msec)-startTime) < timeOut*1000)
          mogoToggle=1;
          vex::task::sleep(10);
          mogoToggle=0;
        }
        RearMogo.stop(hold); 
        mogoToggle=0;*/
        thread([]() {
          setMogo(posRing);
          if(std::abs(MogoRot.position(deg) - posRing)>.5){
            std::terminate();
          }
        }).detach();
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
        setMogo(.8);
        break;
    }
  }
}

void startBar(float speed){
  Bar.spin(fwd,speed, pct);
}

void stopBar(){
  Bar.stop(hold);
}

void mogoRotation(float rot){
  RearMogo.rotateFor(rot, rotationUnits::rev, 100, velocityUnits::pct, true);
}

void spinIntake(){
  Intake.spin(fwd, 100, pct);
}

void stopIntake(){
  Intake.stop(coast);
}

void openClaw(){
  clawSpinT(-.4);
}

void closeClaw(){
  clawSpinT(.5);
}

void clawSpinT(float t){
    if(t>0){
    Claw.spin(fwd,100,pct);
    vex::task::sleep(t*1000);
    Claw.stop(hold);
  } else{    
    Claw.spin(reverse,100,pct);
    vex::task::sleep(-t*1000);
    Claw.stop(hold);
  }
}
