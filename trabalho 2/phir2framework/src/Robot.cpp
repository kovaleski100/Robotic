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

    grid = new Grid();
    printf("criando grid\n");

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
    controlTimer.waitTime(0.2);
    //std::cout << "jejeje" << std::endl;

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

    pthread_mutex_lock(grid->mutex);

    // Mapping
    mappingWithHIMMUsingLaser();
    mappingWithLogOddsUsingLaser();
    mappingUsingSonar();

    pthread_mutex_unlock(grid->mutex);

    plan->setNewRobotPose(currentPose_);

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
}

float last_error = 0;
float CTE_sum = 0;


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
    // base.getMaxLaserRange()*scale = numero de células

    // how to access a grid cell:
    //Cell* d=grid->getCell(robotX,robotY+20);
    //printf("robot x = %d, y = %d, angle = %f, scale = %d\n", robotX,robotY,robotAngle,scale);
    //d->occupancy = 0.1;

    //logodds = (c->logodds) + getLogOddsFromOccupanc

    // access log-odds value of variable in c->logodds
    // how to convert logodds to occupancy values:
//    c->occupancy = getOccupancyFromLogOdds(c->logodds);

    // TODO: define fixed values of occupancy
    float locc = 0.75;
    float lfree = 0.25;


 //determinando campo de visão
    //int numCells = maxRange*scale;
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


    for (int y = limiteUL[1]; y != limiteDL[1]; y--)
    {
        float result=0; //mudar para iniciar com valor deconhecido
        for (int x = limiteUL[0]; x != limiteUR[0]; x++)
        {
             float powX = pow(x-robotX,2);
             float powY = pow(y-robotY,2);
             float r = sqrt(powX+powY)/scale;

             float arcTan2 = x-robotX;
             if (arcTan2 == 0)
                 arcTan2=0.0001;

             float phi = ((57.2958*atan2(y-robotY,arcTan2))-robotAngle);
             phi = normalizeAngleDEG(phi);
             int nearestLaser = base.getNearestLaserBeam(phi);
             float k = base.getAngleOfLaserBeam(nearestLaser);
             float nlReading = base.getKthLaserReading(nearestLaser);

             if (r>std::min(maxRange, nlReading+(lambda_r/2)) || ((fabs(phi-k))>(lambda_phi/2)))
                 result = 0.0;
             else if (nlReading<maxRange && (fabs(r-nlReading)<(lambda_r/2)))
                 result = locc;
             else if (r<=nlReading)
                 result = lfree;

             if (result != 0.0)
             {
                 Cell* c=grid->getCell(x,y);
                 float logodds = (c->logodds) + getLogOddsFromOccupancy(result);
                 c->logodds = logodds;
                 c->occupancy = getOccupancyFromLogOdds(logodds);
             }
        }
    }


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
    int scale = grid->getMapScale();
    float maxRange = base.getMaxSonarRange();
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
    int free = 2;
    int occ = 1;
    int result=0;




    for (int y = limiteUL[1]; y != limiteDL[1]; y--)
    {
        for (int x = limiteUL[0]; x != limiteUR[0]; x++)
        {
             //printf("Cel x = %d, y = %d\n", x,y);
            Cell* c=grid->getCell(x,y);
            if (c->occupancySonar < 0.01)
                c->occupancySonar = 0.01;
             else if (c->occupancySonar > 0.99)
                c->occupancySonar = 0.99;

             float powX = pow(x-robotX,2);
             float powY = pow(y-robotY,2);
             float r = sqrt(powX+powY)/scale;
             float phi = normalizeAngleDEG(57.2958*atan2(y-robotY,x-robotX)-robotAngle);
             int nearestSonar = base.getNearestSonarBeam(phi);
             float k = base.getAngleOfSonarBeam(nearestSonar);
             float nsReading = base.getKthSonarReading(nearestSonar);

             if ((r>maxRange || r>(nsReading+(lambda_r/2))) || (fabs(phi-k)>(lambda_phi/2)))
                 result = 0;
             else if (nsReading<maxRange && (fabs(r-nsReading)<(lambda_r/2)))
                 result = occ;
             else if (r<=nsReading)
                 result = free;


//printf("occ = %d\n", c->occupancySonar);
             if (result != 0)
             {
                 //Cell* c=grid->getCell(x,y);
                 float occUpdate;
                 float beta = lambda_phi/2;
                 float readingTerm = ((maxRange-r)/maxRange);
                 float angleTerm = ((beta)-k)/(beta);
                 float angleReading = 0.5*(readingTerm+angleTerm);

                 if (result == occ)
                    occUpdate = 0.5*(angleReading)+0.5;
                 else if (result == free)
                    occUpdate = 0.5*(1-angleReading);

                float occTerm = occUpdate*(c->occupancySonar);
                float occ = (occTerm)/(occTerm + ((1.0 - occUpdate) * (1.0 - c->occupancySonar)));
               // printf("occ = %d\n", c->occupancySonar);
                c->occupancySonar = occ;
             }

        }
    }

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



    for (int y = limiteUL[1]; y != limiteDL[1]; y--)
    {
        float result=0; //mudar para iniciar com valor deconhecido
        for (int x = limiteUL[0]; x != limiteUR[0]; x++)
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
