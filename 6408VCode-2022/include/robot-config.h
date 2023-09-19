using namespace vex;

extern brain Brain;

// VEXcode devices
extern controller Controller1;
extern motor MogoLift;
extern motor RightLift;
extern motor LeftLift;
extern motor RightFront;
extern motor LeftFront;
extern motor RightBack;
extern motor LeftBack;
extern motor Intake;
extern pneumatics Clamp;
/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );