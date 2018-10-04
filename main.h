
enum state {IDLE, DRIVE_SIDE, ROTATE};

struct Settings_s{

    int driveDirection;
    int rotateDirection;
    int speed;
    int rotate;
    int drive;
    int rotateCounter;

    int movementComplete;

};

typedef struct Settings_s Settings_t;

void stateMachine();
void evalSettings();

