

void updateValues();

void Steppers_Init();

void limitStop_X();

void limitStop_Y();

void calibrationToOrigin();

void coreXYMoveTo(double X_des, double Y_des);

void coreXYMove(double dx, double dy);

void processCommand(bool noSerialResponse = false);