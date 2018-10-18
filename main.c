#include "main.h"
#include "RP6RobotBaseLib.h"
#include <math.h>

#define RIGHT_ANGLE_CORRECTED 112
#define STEPS_TO_CM(STEPS) ((double)STEPS *(0.24/10))

#define FIRST_COMPLETE_CROSS 1
#define LAST_COMPLETE_CROSS 2

enum state RP6State;
Settings_t RP6Settings;

Track_t firstTracks[20];
Track_t secondTracks[20];
int firstSide = 0;
int secondSide = 0;
double tanAlpha;

int onStartPointSecondRun = FALSE;

int main(void)
{
    RP6State = DRIVE_SIDE;

    RP6Settings.driveDirection = FWD;
    RP6Settings.rotateDirection = LEFT;
    RP6Settings.speed = 0;
    RP6Settings.drive = FALSE;
    RP6Settings.rotate = FALSE;
    RP6Settings.movementComplete = TRUE;
    RP6Settings.drivenDistance = 0;
    RP6Settings.currentTrackCounter = 0;
    RP6Settings.totalTrackCounter = 0;

    RP6Settings.lastRotateDirection = 0;


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
            RP6Settings.drivenDistance = getLeftDistance();

            RP6State = ROTATE;
        }
        else if(getBumperRight())
        {
            writeString("BUMPER HIT RIGHT\n");
            stop();
            RP6Settings.drive = FALSE;
            RP6Settings.rotate = TRUE;
            RP6Settings.rotateDirection = LEFT;
            RP6Settings.drivenDistance = getLeftDistance();

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

            mleft_dist = 0;
            mright_dist = 0;

        }

        break;
    }
}

void evalSettings()
{

    if(RP6Settings.drive)
    {
        moveAtSpeedDirection(RP6Settings.speed, RP6Settings.speed);
    }
    else if(RP6Settings.rotate)
    {
        if(firstSide == 0)
        {
            RP6Settings.totalTrackCounter++;

            if(tryCalcFirstSide())
            {
                RP6Settings.currentTrackCounter = 0;
            }

            writeString("First Side: ");
            writeInteger(STEPS_TO_CM(firstSide), DEC);
            writeChar('\n');
        }
        else
        {
            Track_t *t = newTrack(RP6Settings.drivenDistance, RP6Settings.lastRotateDirection, RP6Settings.rotateDirection);
            RP6Settings.totalTrackCounter++;

            if(!t->betweenParallelSides && !onStartPointSecondRun)
            {
                onStartPointSecondRun = TRUE;
            }
            if(onStartPointSecondRun)
            {
                if(tryCalcSecondSide())
                {
                    writeString("Second Side: ");
                    writeInteger(STEPS_TO_CM(secondSide), DEC);
                    writeChar('\n');
                }
            }
            free(t);
        }


        writeString("rotating now\n");
        rotate(RP6Settings.speed, RP6Settings.rotateDirection, RIGHT_ANGLE_CORRECTED, BLOCKING);

        RP6Settings.lastRotateDirection = RP6Settings.rotateDirection;


    }
    if(isMovementComplete())
    {
        RP6Settings.movementComplete = TRUE;
    }

}

int tryCalcFirstSide()
{
    if(RP6Settings.lastRotateDirection != 0)
    {
        firstTracks[RP6Settings.currentTrackCounter] = *newTrack(RP6Settings.drivenDistance, RP6Settings.lastRotateDirection, RP6Settings.rotateDirection);

        RP6Settings.currentTrackCounter++;

        Track_t *a;
        Track_t *b;
        Track_t *c;
        Track_t *d;

        if(RP6Settings.currentTrackCounter >= 2)
        {
            a = &firstTracks[RP6Settings.currentTrackCounter - 2];
            b = &firstTracks[RP6Settings.currentTrackCounter - 1];

            /*check if last two firstTracks are both between parallel sides*/
            if(checkTwoTracks(a, b))
            {
                calcWithTwoTracks(a,b);

                return TRUE;
            }
            else
            {
                if(RP6Settings.currentTrackCounter >= 3)
                {
                    a = &firstTracks[RP6Settings.currentTrackCounter - 3];
                    b = &firstTracks[RP6Settings.currentTrackCounter - 2];
                    c = &firstTracks[RP6Settings.currentTrackCounter - 1];

                    if(checkThreeTracks(a, b, c))
                    {
                        calcWithThreeTracks(a, b, c);
                        return TRUE;
                    }
                    else
                    {
                        if(RP6Settings.currentTrackCounter >= 4)
                        {
                            a = &firstTracks[RP6Settings.currentTrackCounter - 4];
                            b = &firstTracks[RP6Settings.currentTrackCounter - 3];
                            c = &firstTracks[RP6Settings.currentTrackCounter - 2];
                            d = &firstTracks[RP6Settings.currentTrackCounter - 1];

                            calcWithFourTracks(a,b,c,d);

                            return TRUE;

                        }
                    }
                }
            }
        }
    }
    return FALSE;
}

int tryCalcSecondSide()
{

    secondTracks[RP6Settings.currentTrackCounter] = *newTrack(RP6Settings.drivenDistance, RP6Settings.lastRotateDirection, RP6Settings.rotateDirection);

    RP6Settings.currentTrackCounter++;

    writeString("TRACK ADDED \n");


    secondSide = 0;

    if(hasCrossTracks())
    {
        writeString("Has cross tracks ");

        int indexFirstCornerTrack = checkTwoNotAdjacentCornerTracks();
        if(indexFirstCornerTrack >= 0)
        {
            writeString("Has enough cross tracks ");
            int indexLastCornerTrack = getNextCornerTrack(indexFirstCornerTrack);
            int i;

            writeString("index first Corner track: ");
            writeInteger(indexFirstCornerTrack,DEC);
            writeChar('\n');
            writeString("index last Corner track: ");
            writeInteger(indexLastCornerTrack,DEC);
            writeChar('\n');

            for(i = indexFirstCornerTrack; i <= indexLastCornerTrack; i++ )
            {
                if(((RP6Settings.totalTrackCounter - RP6Settings.currentTrackCounter + i)%2))
                {

                    if(firstTrackIsCrossTrack())
                    {
                        //use cosinus
                       secondSide = secondSide + secondTracks[i].distance* (1 / sqrt(1 + pow(tanAlpha,2)));
                       writeString("COS \n");
                    }
                    else
                    {
                        //use sinus
                        secondSide = secondSide + secondTracks[i].distance*(tanAlpha / (sqrt(1 + pow(tanAlpha,2))));
                        writeString("SIN \n");
                    }

                }
                else
                {
                     if(!firstTrackIsCrossTrack())
                    {
                        //use cosinus
                       secondSide = secondSide + secondTracks[i].distance* (1 / sqrt(1 + pow(tanAlpha,2)));
                       writeString("COS \n");
                    }
                    else
                    {
                        //use sinus
                        secondSide = secondSide + secondTracks[i].distance*(tanAlpha / (sqrt(1 + pow(tanAlpha,2))));
                        writeString("SIN \n");
                    }
                }
            }
            int j;
            for(j = 0; j < RP6Settings.currentTrackCounter; j++)
            {
                writeInteger(secondTracks[j].distance,DEC);
                writeChar('\n');
            }

            return TRUE;

        }
        else
        {
            return FALSE;
        }
    }
    else
    {
        if(RP6Settings.currentTrackCounter == 3 && onlyCornerTracks())
        {
            secondSide = secondTracks[RP6Settings.currentTrackCounter -2].distance * (1 / sqrt(1 + pow(tanAlpha,2))) +
            secondTracks[RP6Settings.currentTrackCounter -1].distance * (tanAlpha / (sqrt(1 + pow(tanAlpha,2))));

            return TRUE;
        }

        return FALSE;

    }
}

int getNextCornerTrack(int current)
{
    int next = current + 1;
    while(secondTracks[next].betweenParallelSides)
    {
        next++;
    }

    return next;
}

int firstTrackIsCrossTrack()
{
    if(firstTracks[0].betweenParallelSides)
    {
        return TRUE;
    }

    return FALSE;
}

int onlyCornerTracks()
{
    int i;

    for(i = 0; i < RP6Settings.currentTrackCounter; i++)
    {
        if(secondTracks[i].betweenParallelSides)
        {
            return FALSE;
        }
    }

    return TRUE;
}

int hasCrossTracks()
{
    int i;
    int hasCrossTracks = FALSE;

    for(i = 0; i < RP6Settings.currentTrackCounter; i++)
    {
        if(secondTracks[i].betweenParallelSides)
        {
            hasCrossTracks = TRUE;
        }
    }

    return hasCrossTracks;
}

int checkTwoNotAdjacentCornerTracks()
{
    int i;

    int firstCrossLineIndex = -1;

    for(i = 0; i < (RP6Settings.currentTrackCounter -1); i++)
    {
        if(!secondTracks[i].betweenParallelSides && secondTracks[i + 1].betweenParallelSides)
        {
            firstCrossLineIndex = i;
        }
        else if(secondTracks[i].betweenParallelSides && !secondTracks[i + 1].betweenParallelSides)
        {
            return firstCrossLineIndex;
        }
    }

    return -1;

}

void calcWithTwoTracks(Track_t *a, Track_t *b)
{
    if(b->distance < a->distance)
    {
        tanAlpha = (double) b->distance / (double) a->distance;
        firstSide = b->distance * (1 / sqrt(1 + pow(tanAlpha,2)));
    }
    else
    {
        tanAlpha =  (double) a->distance /  (double) b->distance;
        firstSide = a->distance * (1 / sqrt(1 + pow(tanAlpha,2)));
    }
}


void calcWithThreeTracks(Track_t *a, Track_t *b, Track_t *c)
{
    int test = checkThreeTracks(a, b, c);
    if(test == FIRST_COMPLETE_CROSS)
    {
        tanAlpha = ((double)a->distance - (double) c->distance) / (double) b->distance;
        firstSide = a->distance * (1 / sqrt(1 + pow(tanAlpha,2)));

    }
    else if(test == LAST_COMPLETE_CROSS)
    {
        tanAlpha = ((double)c->distance - (double) a->distance) / (double) b->distance;
        firstSide = c->distance * (1 / sqrt(1 + pow(tanAlpha,2)));
    }
}


void calcWithFourTracks(Track_t *a, Track_t *b, Track_t *c, Track_t *d)
{
    tanAlpha = ((double) c->distance - (double) a->distance)/ ((double) b->distance - (double) d->distance);

    firstSide = a->distance * (1 / sqrt(1 + pow(tanAlpha,2))) + b->distance * (tanAlpha / (sqrt(1 + pow(tanAlpha,2))));
}


Track_t *newTrack(int distance, int lastRotate, int currentRotate)
{
    Track_t *dummy = (Track_t*) malloc(sizeof(Track_t));

    dummy->distance = distance + DIST_CM(18);

    if(lastRotate == currentRotate)
    {
        dummy->betweenParallelSides = FALSE;
    }
    else
    {
        dummy->betweenParallelSides = TRUE;
    }

    return dummy;

}

int checkTwoTracks(Track_t* t1, Track_t* t2)
{

    if(t1->betweenParallelSides && t2->betweenParallelSides)
    {
        return TRUE;
    }

    return FALSE;
}

int checkThreeTracks( Track_t* t1, Track_t* t2, Track_t* t3)
{
    if(t1->betweenParallelSides && !t2->betweenParallelSides && !t3->betweenParallelSides)
    {
        return FIRST_COMPLETE_CROSS;
    }
    else if(!t1->betweenParallelSides && !t2->betweenParallelSides && t3->betweenParallelSides)
    {
        return LAST_COMPLETE_CROSS;
    }

    return FALSE;
}

