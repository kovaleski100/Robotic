#include "Robot.h"

#include <unistd.h>
#include <GL/glut.h>
#include <cmath>
#include <iostream>


//////////////////////////////////////
///// CONSTRUCTORS & DESTRUCTORS /////
//////////////////////////////////////

Robot::Robot()
{
    ready_ = false;
    running_ = true;
    firstIteration = true;

    grid = new Grid();
    mcl = NULL;

    plan = new Planning();
    plan->setGrid(grid);
    plan->setMaxUpdateRange(base.getMaxLaserRange());

    // variables used for navigation
    isFollowingLeftWall_=false;

    // variables used for visualization
    viewMode=0;
    numViewModes=5;
    motionMode_=MANUAL_SIMPLE;

}

Robot::~Robot()
{
    base.closeARIAConnection();
    if(grid!=NULL)
        delete grid;
}

////////////////////////////////////
///// INITIALIZE & RUN METHODS /////
////////////////////////////////////

void Robot::initialize(ConnectionMode cmode, LogMode lmode, std::string fname, std::string mapName)
{
    logMode_ = lmode;

    mcl = new MCL(base.getMaxLaserRange(),mapName,grid->mutex);

    // initialize ARIA
    if(logMode_!=PLAYBACK){
        bool success = base.initialize(cmode,lmode,fname);
        if(!success){
            printf("Could not connect to robot... exiting\n");
            exit(0);
        }
    }

    ready_ = true;
    controlTimer.startLap();
}

void Robot::run()
{
    controlTimer.waitTime(0.2);

    if(logMode_==PLAYBACK){
        bool hasEnded = base.readFromLog();
        if(hasEnded){
            std::cout << "PROCESS COMPLETE. CLOSING PROGRAM." << std::endl;
            exit(0);
        }
    }else{
        bool success = base.readOdometryAndSensors();
        if(!success){
            usleep(50000);
            return;
        }

        if(logMode_==RECORDING)
            base.writeOnLog();
    }

    Pose odometry = base.getOdometry();
    if(firstIteration){
        prevLocalizationPose_ = odometry;
        firstIteration = false;
    }

    Action u;
    u.rot1 = atan2(odometry.y-prevLocalizationPose_.y,odometry.x-prevLocalizationPose_.x)-DEG2RAD(odometry.theta);
    u.trans = sqrt(pow(odometry.x-prevLocalizationPose_.x,2)+pow(odometry.y-prevLocalizationPose_.y,2));
    u.rot2 = DEG2RAD(odometry.theta)-DEG2RAD(prevLocalizationPose_.theta)-u.rot1;

    // check if there is enough robot motion
    if(u.trans > 0.1 || fabs(u.rot1) > DEG2RAD(30) || fabs(u.rot2) > DEG2RAD(30))
    {
        std::cout << currentPose_ << std::endl;
        mcl->run(u,base.getLaserReadings());
        prevLocalizationPose_ = odometry;
    }

    currentPose_ = odometry;

    pthread_mutex_lock(grid->mutex);

    // Mapping
    mappingWithHIMMUsingLaser();
    mappingWithLogOddsUsingLaser();
    mappingUsingSonar();

    pthread_mutex_unlock(grid->mutex);

    plan->setNewRobotPose(currentPose_);

    // Save path traversed by the robot
    if(base.isMoving() || logMode_==PLAYBACK){
        path_.push_back(currentPose_);
    }

    // Navigation
    switch(motionMode_){
        case WANDER:
            wanderAvoidingCollisions();
            break;
        case WALLFOLLOW:
            wallFollow();
            break;
        case POTFIELD_0:
            followPotentialField(0);
            break;
        case POTFIELD_1:
            followPotentialField(1);
            break;
        case POTFIELD_2:
            followPotentialField(2);
            break;
        case ENDING:
            running_=false;
            break;
        default:
            break;
    }

    base.resumeMovement();

    usleep(50000);
}

//////////////////////////////
///// NAVIGATION METHODS /////
//////////////////////////////

void Robot::move(MovingDirection dir)
{
    switch(dir){
        case FRONT:
            std::cout << "moving front" << std::endl;
            break;
        case BACK:
            std::cout << "moving back" << std::endl;
            break;
        case LEFT:
            std::cout << "turning left" << std::endl;
            break;
        case RIGHT:
            std::cout << "turning right" << std::endl;
            break;
        case STOP:
            std::cout << "stopping robot" << std::endl;
    }

    if(motionMode_==MANUAL_SIMPLE)
        base.setMovementSimple(dir);
    else if(motionMode_==MANUAL_VEL)
        base.setMovementVel(dir);
    else if(motionMode_==WALLFOLLOW)
        if(dir==LEFT)
            isFollowingLeftWall_=true;
        else if(dir==RIGHT)
            isFollowingLeftWall_=false;



}

void Robot::wanderAvoidingCollisions()
{
    float linVel=0;
    float angVel=0;

    //TODO - implement obstacle avoidance




    base.setWheelsVelocity_fromLinAngVelocity(linVel, angVel);
}

void Robot::wallFollow()
{
    float linVel=0;
    float angVel=0;

    if(isFollowingLeftWall_)
        std::cout << "Following LEFT wall" << std::endl;
    else
        std::cout << "Following RIGHT wall" << std::endl;

    //TODO - implementar wall following usando PID



    base.setWheelsVelocity_fromLinAngVelocity(linVel, angVel);
}

void Robot::followPotentialField(int t)
{
    int scale = grid->getMapScale();
    int robotX=currentPose_.x*scale;
    int robotY=currentPose_.y*scale;
    float robotAngle = currentPose_.theta;
    float Tp= 0.01;
    float alpha = 40;

    // how to access the grid cell associated to the robot position
    Cell* c=grid->getCell(robotX,robotY);

    float phi=0;

 /*   for(int j = robotY-1; j<=robotY+1; j++)
    {
        for(int i = robotX-1; i <= robotX+1; i++)
        {
            Cell* c=grid->getCell(i,j);
            //printf("xxxxxxxxxxxxxxxxxxxxxxx");
            phi = phi + RAD2DEG(atan2(c->dirY[t], c->dirX[t])) - robotAngle;
        }
    }

    phi = phi/9;*/


    phi = RAD2DEG(atan2(c->dirY[t], c->dirX[t])) - robotAngle;
    phi = normalizeAngleDEG(phi);


    float linVel, angVel;

    angVel = Tp*phi;

    //printf("angVel = %f\n", angVel);
    if ((angVel > 0.7))
        angVel = 0.7;
    else if ((angVel < -0.7))
        angVel = -0.7;


    if ((phi > alpha) || (phi < -alpha))
        linVel = 0;
    else
        linVel = 0.3;

    // TODO: define the robot velocities using a control strategy
    //       based on the direction of the gradient of c given by c->dirX[t] and c->dirY[t]




    base.setWheelsVelocity_fromLinAngVelocity(linVel,angVel);
}


///////////////////////////
///// MAPPING METHODS /////
///////////////////////////

float Robot::getOccupancyFromLogOdds(float logodds)
{
    return 1.0 - 1.0/(1.0+exp(logodds));
}

float Robot::getLogOddsFromOccupancy(float occupancy)
{
    return log(occupancy/(1.0-occupancy));
}

void Robot::mappingWithLogOddsUsingLaser()
{
    float lambda_r = 0.1; //  10 cm
    float lambda_phi = 1.0;  // 1 degree

    int scale = grid->getMapScale();
    float maxRange = base.getMaxLaserRange();
    int maxRangeInt = maxRange*scale;

    int robotX=currentPose_.x*scale;
    int robotY=currentPose_.y*scale;
    float robotAngle = currentPose_.theta;

    // how to access a grid cell:
//    Cell* c=grid->getCell(robotX,robotY);

    // access log-odds value of variable in c->logodds
    // how to convert logodds to occupancy values:
//    c->occupancy = getOccupancyFromLogOdds(c->logodds);

    // TODO: define fixed values of occupancy
    float locc, lfree;



    // TODO: update cells in the sensors' field-of-view
    // ============================================================================
    // you only need to check the cells at most maxRangeInt from the robot position
    // that is, in the following square region:
    //
    //  (robotX-maxRangeInt,robotY+maxRangeInt)  -------  (robotX+maxRangeInt,robotY+maxRangeInt)
    //                     |                       \                         |
    //                     |                        \                        |
    //                     |                         \                       |
    //  (robotX-maxRangeInt,robotY-maxRangeInt)  -------  (robotX+maxRangeInt,robotY-maxRangeInt)




}

void Robot::mappingUsingSonar()
{
    float lambda_r = 0.5; //  50 cm
    float lambda_phi = 30.0;  // 30 degrees

    // TODO: update cells in the sensors' field-of-view
    // Follow the example in mappingWithLogOddsUsingLaser()



}

void Robot::mappingWithHIMMUsingLaser()
{
    float lambda_r = 0.2; //  20 cm
    float lambda_phi = 1.0;  // 1 degree
    int scale = grid->getMapScale();
    float maxRange = base.getMaxLaserRange();
    int maxRangeInt = maxRange*scale;

    int robotX=currentPose_.x*scale;
    int robotY=currentPose_.y*scale;
    float robotAngle = currentPose_.theta;

    // TODO: update cells in the sensors' field-of-view
    // Follow the example in mappingWithLogOddsUsingLaser()
    int limiteUL[2],limiteUR[2],limiteDL[2],limiteDR[2];
    limiteUL[0] = robotX-maxRangeInt;
    limiteUL[1] = robotY+maxRangeInt;
    limiteUR[0] = robotX+maxRangeInt;
    limiteUR[1] = robotY+maxRangeInt;
    limiteDL[0] = robotX-maxRangeInt;
    limiteDL[1] = robotY-maxRangeInt;
    limiteDR[0] = robotX+maxRangeInt;
    limiteDR[1] = robotY-maxRangeInt;

    //Cell* c=grid->getCell(x,y);
   // printf("ULx = %d, URx = %d\n", limiteUL[0],limiteUR[0]);
    int free = -1;
    int occ = 3;



    for (int y = limiteUL[1]; y >= limiteDL[1]; y--)
    {
        float result=0; //mudar para iniciar com valor deconhecido
        for (int x = limiteUL[0]; x <= limiteUR[0]; x++)
        {
             //printf("Cel x = %d, y = %d\n", x,y);
            /*Cell *d = grid->getCell(x,y);
            d->planType = FRONTIER;*/

             float powX = pow(x-robotX,2);
             float powY = pow(y-robotY,2);
             float r = sqrt(powX+powY)/scale;
             float phi = normalizeAngleDEG(57.2958*atan2(y-robotY,x-robotX)-robotAngle);
             int nearestLaser = base.getNearestLaserBeam(phi);
             float k = base.getAngleOfLaserBeam(nearestLaser);
             float nlReading = base.getKthLaserReading(nearestLaser);

             if ((r>maxRange || r>(nlReading+(lambda_r/2))) || (fabs(phi-k)>(lambda_phi/2)))
                 result = 0;
             else if (nlReading<maxRange && (fabs(r-nlReading)<(lambda_r/2)))
                 result = occ;
             else if (r<=nlReading)
                 result = free;


             if (result != 0)
             {
                 Cell* c=grid->getCell(x,y);
                 int chance = c->himm + result;
                 if (chance <0)
                     chance = 0;
                 else if (chance > 15)
                     chance = 15;
                 c->himm = chance;
             }

        }
    }


}

/////////////////////////////////////////////////////
////// METHODS FOR READING & WRITING ON LOGFILE /////
/////////////////////////////////////////////////////

// Prints to file the data that we would normally be getting from sensors, such as the laser and the odometry.
// This allows us to later play back the exact run.
void Robot::writeOnLog()
{
    logFile_->writePose("Odometry",currentPose_);
    logFile_->writeSensors("Sonar",base.getSonarReadings());
    logFile_->writeSensors("Laser",base.getLaserReadings());
}

// Reads back into the sensor data structures the raw readings that were stored to file
// While there is still information in the file, it will return 0. When it reaches the end of the file, it will return 1.
bool Robot::readFromLog() {

    if(logFile_->hasEnded())
        return true;

    base.setOdometry(logFile_->readPose("Odometry"));
    base.setSonarReadings(logFile_->readSensors("Sonar"));
    base.setLaserReadings(logFile_->readSensors("Laser"));

    return false;
}

Pose Robot::readInitialPose()
{
    Pose p = logFile_->readPose("Start");
    p.theta = DEG2RAD(p.theta);
    return p;
}

////////////////////////
///// DRAW METHODS /////
////////////////////////

void Robot::draw(float xRobot, float yRobot, float angRobot)
{
    float scale = grid->getMapScale();
    glTranslatef(xRobot,yRobot,0.0);
    glRotatef(angRobot,0.0,0.0,1.0);
    glScalef(1.0/scale,1.0/scale,1.0/scale);

    // sonars and lasers draw in cm
    if(viewMode==1)
        base.drawSonars(true);
    else if(viewMode==2)
        base.drawSonars(false);
    else if(viewMode==3)
        base.drawLasers(true);
    else if(viewMode==4)
        base.drawLasers(false);

    // robot draw in cm
    base.drawBase();

    glScalef(scale,scale,scale);
    glRotatef(-angRobot,0.0,0.0,1.0);
    glTranslatef(-xRobot,-yRobot,0.0);
}

/////////////////////////
///// OTHER METHODS /////
/////////////////////////

bool Robot::isReady()
{
    return ready_;
}

bool Robot::isRunning()
{
    return running_;
}

const Pose& Robot::getCurrentPose()
{
    return currentPose_;
}

void Robot::drawMCL()
{
    mcl->draw(grid->viewMode-6);
}

void Robot::drawPath()
{
    float scale = grid->getMapScale();

    if(path_.size() > 1){
        glScalef(scale,scale,scale);
        glLineWidth(3);
        glBegin( GL_LINE_STRIP );
        {
            for(unsigned int i=0;i<path_.size()-1; i++){
                glColor3f(1.0,0.0,1.0);

                glVertex2f(path_[i].x, path_[i].y);
                glVertex2f(path_[i+1].x, path_[i+1].y);
            }
        }
        glEnd();
        glLineWidth(1);
        glScalef(1.0/scale,1.0/scale,1.0/scale);

    }
}

void Robot::waitTime(float t){
    float l;
    do{
        usleep(1000);
        l = controlTimer.getLapTime();
    }while(l < t);
    controlTimer.startLap();
}
