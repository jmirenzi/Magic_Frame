



void initializeSteppers();

void calibrateGlobals(float, float);

void updateGlobals();

void moveR(float, float, float, float);

void moveR_paired(float, float, float, float);

float mag(float, float);

float distTo(float,float);

void move(float,float);

void move_paired(float,float);

void processCommand(bool noSerialResponse = false);