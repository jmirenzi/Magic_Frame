



void Steppers_Init();

void limitStop();

void calibrationToOrigin();

void coreXYMoveTo(double X_des, double Y_des);

void coreXYMove(double dx, double dy);

void processCommand(bool noSerialResponse = false);