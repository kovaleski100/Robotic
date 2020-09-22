#include "Robot.h"

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

    grid = new Grid();

    // variables used for navigation
    isFollowingLeftWall_=false;

    // variables used for visualization
    viewMode=1;
    numViewModes=5;
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

void Robot::initialize(ConnectionMode cmode, LogMode lmode, std::string fname)
{
    logMode_ = lmode;
//    logFile_ = new LogFile(logMode_,fname);
    ready_ = true;

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
    controlTimer.waitTime(0.1);

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

    currentPose_ = base.getOdometry();

    // Save path traversed by the robot
    if(base.isMoving() || logMode_==PLAYBACK){
        path_.push_back(base.getOdometry());
    }

    // Navigation
    switch(motionMode_){
        case WANDER:
            wanderAvoidingCollisions();
            break;
        case WALLFOLLOW:
            wallFollow();
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
    else if(motionMode_=WALLFOLLOW)
        if(dir==LEFT)
            isFollowingLeftWall_=true;
        else if(dir==RIGHT)
            isFollowingLeftWall_=false;
}

void Robot::wanderAvoidingCollisions()
{
    float minLeftSonar  = base.getMinSonarValueInRange(0,2);
    float minFrontSonar = base.getMinSonarValueInRange(3,4);
    float minRightSonar = base.getMinSonarValueInRange(5,7);

    float minLeftLaser  = base.getMinLaserValueInRange(0,74);
    float minFrontLaser = base.getMinLaserValueInRange(75,105);
    float minRightLaser = base.getMinLaserValueInRange(106,180);

    float linVel=0.2;
    float angVel=0;
    float l = 0.4;
    //float Rl =  minLeftSonar + l;
    //float Rr =  minRightSonar + l;
    float lw=0;
    float rw=0;

    if (minLeftSonar < 1.0)
    {
        rw = (1.0+l-minLeftSonar); // girar pro lado contrario
       /* if (minLeftSonar < (minRightSonar))
            linVel = linVel/(2.0*(1-minLeftSonar));*/
    }
    if (minRightSonar < 1.0)
    {
        lw = (1.0+l-minRightSonar);
     /*   if (minRightSonar < (minLeftSonar))
            linVel = linVel/(2.0*(1-minRightSonar));*/
    }
    if (minFrontSonar < 1.2)
    {
      if (minFrontSonar < 0.7) // muito perto
          linVel = 0.0;
      else
       linVel = linVel/(3-(minFrontSonar));  // desacelera
    }
    if (abs(minLeftSonar-minRightSonar) < 0.1)  //evita ficar parado em cantos internos
        rw = rw+0.1;



    angVel = (lw-rw)/2;
    //angVel = -2.0;

    if (linVel == 0.0 && abs(angVel) < 0.1) //evita ficar parado em cantos externos
        angVel= angVel+0.1;


    printf("l,f,r = %f,%f,%f/n", minLeftSonar,minFrontSonar,minRightSonar);

    //angVel = -2.0;

    //TODO - implementar desvio de obstaculos
    //if ((minLeftSonar || minRightSonar || minRightSonar) < 0.7);
   /* if (minLeftSonar < 0.7)
    {
        if (minFrontSonar < 2.0)
        {
            rw=0;
            lw=60;
        }
        else
        {
            rw=200;
            lw=200;
        }
    }
    else if (minRightSonar < 0.7)
    {
        if (minFrontSonar < 2.0)
        {
            rw=60;
            lw=0;
        }
        else
        {
            rw=200;
            lw=200;
        }
    }
    else if (minFrontSonar < 0.7)
    {
        if (minLeftSonar < 0.7)
        {
            rw=0;
            lw=60;
        }
        else if (minRightSonar < 0.7)
        {
            rw=60;
            lw=0;
        }
        else
        {
            rw=-200;
            lw=-150;
        }
    }
    else
    {
        rw=300;
        lw=300;
    }*/





    base.setWheelsVelocity_fromLinAngVelocity(linVel, angVel);
    //base.setWheelsVelocity(lw,rw);
}


float last_error = 0;
float CTE_sum = 0;
//int chosenWall = 0;

void Robot::wallFollow()
{
    float minLeftSonar  = base.getMinSonarValueInRange(0,2);
    float minFrontSonar = base.getMinSonarValueInRange(3,4);
    float minRightSonar = base.getMinSonarValueInRange(5,7);

    float minLeftLaser  = base.getMinLaserValueInRange(0,74);
    float minFrontLaser = base.getMinLaserValueInRange(75,105);
    float minRightLaser = base.getMinLaserValueInRange(106,180);


    float angVel=0;
    float linVel=0.2;
    float CTE = 0;
    float Tp = 0.5;
    float Td = 0.05;
    float Ti = 0.000;
    float sp = 1.4; // vai ser a distancia pra ele ficar da parede

    float result_sensor = 0;

    if (minFrontSonar < 1) // muito perto
    {
        if (minFrontSonar < 0.6) // pra ter certeza que nao vai bater
            linVel = 0;
        else
            linVel = linVel/2;
    }
   /* if(minLeftSonar<=minRightSonar)
    {
        result_sensor = minLeftSonar;
    }
    else
    {
        result_sensor = minRightSonar;
    }*/

       // result_sensor = minLeftLaser - minRightLaser;// se <0, esquerda está mais perto

    /*if(result_sensor < 0.7) //serve como threshould pra mudar de parede dps de escolher uma
        chosenWall = 0;*/

    if (chosenWall == true)
     {
        result_sensor = minLeftSonar;
        isFollowingLeftWall_ = true;
        //chosenWall = -1;
     }
      else
     {
        result_sensor = minRightSonar;
        isFollowingLeftWall_ = false;
        //chosenWall = 1;
     }


   /* result_sensor = minLeftSonar;
    isFollowingLeftWall_ = true;*/

    CTE = sp - result_sensor; //
    if (CTE > 0.5)
        CTE = 0.5;
    else if (CTE < -0.5)
        CTE = -0.5;



    CTE_sum += CTE;
    float New_CTE = (CTE) - last_error; // vai dar a diferença erro atual com o erri anterior
    last_error = New_CTE; //fica salvo pra proxima iteração


    angVel = -(Tp*(CTE)) - (Td*New_CTE) - (Ti* CTE_sum);

    /*if ((angVel) > 0.9)
       angVel = 0.9;
    else if ((angVel) < -0.9)
       angVel = -0.9;*/

   // angVel*=(-1);
    /*-0.75 -  -(0.3 - 0) < 0
    -0.75 -  0.3 - 0*/
    //-0.5 - ()
    printf("l,f,r = %f,%f,%f\n", minLeftSonar,minFrontSonar,minRightSonar);
    //printf("CTE,Td*newCTE,newCTE = %f,%f,%f/n", (CTE),(-Td*New_CTE),(New_CTE));
        //printf("angVel= %f/n", (angVel));
    printf("CTE = %f\n", (CTE));

    if (isFollowingLeftWall_ == false) // inverte os resultados para seguir a parede da direita
        angVel*=(-1);

    if(isFollowingLeftWall_)
        std::cout << "Following LEFT wall" << std::endl;
    else
        std::cout << "Following RIGHT wall" << std::endl;

    //TODO - implementar wall following usando PID


    base.setWheelsVelocity_fromLinAngVelocity(linVel, angVel);
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

