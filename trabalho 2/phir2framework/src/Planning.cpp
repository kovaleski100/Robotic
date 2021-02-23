#include "Planning.h"

#include <queue>
#include <float.h> //DBL_MAX

////////////////////////
///                  ///
/// Métodos Públicos ///
///                  ///
////////////////////////

Planning::Planning()
{
    newRobotPosition.x = 0;
    newRobotPosition.y = 0;

    newGridLimits.minX = newGridLimits.minY = 1000;
    newGridLimits.maxX = newGridLimits.maxY = -1000;
}

Planning::~Planning()
{}

void Planning::setGrid(Grid *g)
{
    grid = g;
}

void Planning::setMaxUpdateRange(int r)
{
    maxUpdateRange = 1.2*r*grid->getMapScale();
}

void Planning::setNewRobotPose(Pose p)
{
    newRobotPosition.x = (int)(p.x*grid->getMapScale());
    newRobotPosition.y = (int)(p.y*grid->getMapScale());

    newGridLimits.minX = std::min(newGridLimits.minX,newRobotPosition.x-maxUpdateRange);
    newGridLimits.maxX = std::max(newGridLimits.maxX,newRobotPosition.x+maxUpdateRange);
    newGridLimits.minY = std::min(newGridLimits.minY,newRobotPosition.y-maxUpdateRange);
    newGridLimits.maxY = std::max(newGridLimits.maxY,newRobotPosition.y+maxUpdateRange);
}

void Planning::run()
{
    pthread_mutex_lock(grid->mutex);

    // resetCellsTypes();

    // update robot position and grid limits using last position informed by the robot
    robotPosition = newRobotPosition;
    gridLimits = newGridLimits;

    updateCellsTypes();

    pthread_mutex_unlock(grid->mutex);
}

/////////////////////////////////////////////
///                                       ///
/// Métodos para classificacao de celulas ///
///                                       ///
/////////////////////////////////////////////

void Planning::resetCellsTypes()
{
    for(int i=gridLimits.minX;i<=gridLimits.maxX;i++){
        for(int j=gridLimits.minY;j<=gridLimits.maxY;j++){

            Cell* c = grid->getCell(i,j);

            c->occType = UNEXPLORED;
            c->planType = REGULAR;
        }
    }
}


void Planning::updateCellsTypes()
{
   // Cell* c;=grid->getCell(x,y);
    int limiteUL[2],limiteUR[2],limiteDL[2];
    int limiarOCCtoFREE = 3;
    int limiarFREEtoOCC = 8;
    int xL,xR,yU,yD;


    limiteUL[0] = robotPosition.x-maxUpdateRange;
    limiteUL[1] = robotPosition.y+maxUpdateRange;
    limiteUR[0] = robotPosition.x+maxUpdateRange;
    limiteDL[1] = robotPosition.y-maxUpdateRange;

    //usando HIMM como base
//c->planType = FRONTIER;
   // printf("y=%f\n",maxUpdateRange);

    for (int y = limiteUL[1]; y != limiteDL[1]; y--)
    {
        for (int x = limiteUL[0]; x != limiteUR[0]; x++)
        {
           // printf("x=%d\n",x);
            Cell* c  = grid->getCell(x,y);

           /* if (c->planType == FRONTIER)
                printf("esta e front \n");*/

           if (c->occType == UNEXPLORED)
           {
               if (c->himm <= limiarOCCtoFREE)
                   c->occType = FREE;
               else if (c->himm >= limiarFREEtoOCC)
                   c->occType = OCCUPIED;

              if (c->occType == UNEXPLORED) //se ainda é unexplored
              {
               xL = x-1;
               if (xL < limiteUL[0]) //pra n sair da área de análise
                   xL = limiteUL[0];
               xR = x+1;
               if (xR > limiteUR[0])
                   xR = limiteUR[0];
               yU = y+1;
               if (yU > limiteUL[1])
                   yU = limiteUL[1];
               yD = y-1;
               if (yD < limiteDL[1])
                   yD = limiteDL[1];

               int frontier = 0;
               Cell* d =grid->getCell(x,y);
               //->planType = FRONTIER;
              // printf("x=%d\n",x);


               for (int y2=yU; y2!=yD; y2--)
               {
                  for (int x2=xL; x2!=xR; x2++)
                  {
                     d = grid->getCell(x2,y2);
                      //printf("x2=%d\n",x2);
                     if (d->occType == FREE)
                     {
                         frontier = 1;
                         //printf("achei frontier\n");
                         break;
                     }
                  }
                  if (frontier == 1)
                      break;
               }
               if (frontier == 1)
                 c->planType = FRONTIER;
              }


           }

           if (c->occType == FREE)
           {
             if(c->himm > limiarFREEtoOCC)
               c->occType = OCCUPIED;

             if (c->occType == FREE)
             {
                 xL = x-3;
                 if (xL < limiteUL[0]) //pra n sair da área de análise
                     xL = limiteUL[0];
                 xR = x+3;
                 if (xR > limiteUR[0])
                     xR = limiteUR[0];
                 yU = y+3;
                 if (yU > limiteUL[1])
                     yU = limiteUL[1];
                 yD = y-3;
                 if (yD < limiteDL[1])
                     yD = limiteDL[1];

                 int danger = 0;
                 Cell* d =grid->getCell(x,y);
                 //->planType = FRONTIER;
                // printf("x=%d\n",x);


                 for (int y2=yU; y2!=yD; y2--)
                 {
                    for (int x2=xL; x2!=xR; x2++)
                    {
                       d = grid->getCell(x2,y2);
                        //printf("x2=%d\n",x2);
                       if (d->occType == OCCUPIED)
                       {
                           danger = 1;
                           //printf("achei frontier\n");
                           break;
                       }
                    }
                    if (danger == 1)
                        break;
                 }
                 if (danger == 1)
                   c->planType = DANGER;
                 else
                   c->planType = REGULAR;
             }
           }
           else if (c->occType == OCCUPIED && c->himm < limiarOCCtoFREE)
           {
              c->occType = FREE;
           }
        }
    }

    // If you want to access the current cells surrounding the robot, use this range
    //
    //  (robotPosition.x-maxUpdateRange, robotPosition.y+maxUpdateRange)  -------  (robotPosition.x+maxUpdateRange, robotPosition.y+maxUpdateRange)
    //                                 |                                    \                                     |
    //                                 |                                     \                                    |
    //                                 |                                      \                                   |
    //  (robotPosition.x-maxUpdateRange, robotPosition.y-maxUpdateRange)  -------  (robotPosition.y+maxUpdateRange, robotPosition.y-maxUpdateRange)


    // If you want to access all observed cells (since the start), use this range
    //
    //  (gridLimits.minX, gridLimits.maxY)  -------  (gridLimits.maxX, gridLimits.maxY)
    //                  |                     \                      |
    //                  |                      \                     |
    //                  |                       \                    |
    //  (gridLimits.minX, gridLimits.minY)  -------  (gridLimits.maxX, gridLimits.minY)

    // TODO: classify cells

    // the occupancy type of a cell can be defined as:
    // c->occType = UNEXPLORED
    // c->occType = OCCUPIED
    // c->occType = FREE

    // the planning type of a cell can be defined as:
    // c->planType = REGULAR
    // c->planType = FRONTIER
    // c->planType = DANGER





}

