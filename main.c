#include "main.h"
#include "RP6RobotBaseLib.h"


enum state RP6State;
Settings_t RP6Settings;

int main(void)
{
    RP6State = DRIVE_SIDE;

    RP6Settings.driveDirection = FWD;
    RP6Settings.rotateDirection = LEFT;
    RP6Settings.speed = 0;
    RP6Settings.drive = FALSE;
    RP6Settings.rotate = FALSE;
    RP6Settings.movementComplete = TRUE;
    RP6Settings.rotateCounter = 0;

    initRobotBase();
    powerON();

    while(1)
    {
        stateMachine();
        evalSettings();

        task_RP6System();
    }

    return 0;

}

void stateMachine()
{
    switch(RP6State)
    {
    case IDLE:
        if(RP6Settings.movementComplete)
        {

        }
        break;

    case DRIVE_SIDE:

        writeString("STATE: DRIVE SIDE\n");

        if(RP6Settings.movementComplete)
        {
            RP6Settings.speed = 60;
            RP6Settings.drive = TRUE;
        }

        if(getBumperLeft())
        {
            writeString("BUMPER HIT LEFT\n");
            stop();
            RP6Settings.drive = FALSE;
            RP6Settings.rotate = TRUE;
            RP6Settings.rotateDirection = RIGHT;

            RP6State = ROTATE;
        }
        else if(getBumperRight())
        {
            writeString("BUMPER HIT RIGHT\n");
            stop();
            RP6Settings.drive = FALSE;
            RP6Settings.rotate = TRUE;
            RP6Settings.rotateDirection = LEFT;

            RP6State = ROTATE;
        }
        break;

    case ROTATE:
        writeString("STATE: ROTATE\n");
        if(RP6Settings.movementComplete)
        {
            writeString("ROTATE COMPLETE\n");
            RP6Settings.rotate = FALSE;
            RP6Settings.drive = TRUE;
            RP6State = DRIVE_SIDE;

            RP6Settings.rotateCounter++;

        }

        break;
    }
}

void evalSettings()
{

    if(RP6Settings.drive && RP6Settings.rotateCounter < 5)
    {
        writeString("moving now\n");
        moveAtSpeedDirection(RP6Settings.speed, RP6Settings.speed);
    }
    else if(RP6Settings.rotate)
    {
        writeString("rotating now\n");
        rotate(RP6Settings.speed, RP6Settings.rotateDirection, 90, BLOCKING);
    }


    if(isMovementComplete())
    {

        writeString("Movement complete\n");
        RP6Settings.movementComplete = TRUE;
    }

}
