void Steppers_Init();

void Servo_Init();

void penMoveUp(bool);

void updateValues();

void limitStop_X();

void limitStop_Y();

void calibrationToOrigin();

void coreXYMoveTo(double X_des, double Y_des);

void coreXYMove(double dx, double dy);

void processCommand(bool noSerialResponse = false);