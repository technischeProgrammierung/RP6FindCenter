
enum state {IDLE, DRIVE_SIDE, ROTATE};

struct Settings_s
{

    int driveDirection;
    int rotateDirection;
    int lastRotateDirection;
    int speed;
    int rotate;
    int drive;
    int drivenDistance;
    int currentTrackCounter;
    int totalTrackCounter;

    int movementComplete;

};

typedef struct Settings_s Settings_t;

struct Track_s
{
    int distance;
    int betweenParallelSides;
};

typedef struct Track_s Track_t;


Track_t *newTrack(int, int, int);

void stateMachine();
void evalSettings();

int checkTwoTracks(Track_t*, Track_t*);
int checkThreeTracks( Track_t*, Track_t*, Track_t*);
int checkFourTracks( Track_t*, Track_t*, Track_t*, Track_t*);

int tryCalcFirstSide();
int tryCalcSecondSide();

int hasCrossTracks();
int onlyCornerTracks();
int checkTwoNotAdjacentCornerTracks();
int getNextCornerTrack(int);

int firstTrackIsCrossTrack();

void calcWithTwoTracks(Track_t*, Track_t*);
void calcWithThreeTracks(Track_t*, Track_t*, Track_t*);
void calcWithFourTracks(Track_t*, Track_t*, Track_t*, Track_t*);
