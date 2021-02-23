#include "Planning.h"

#include <queue>
#include <float.h> //DBL_MAX
#include <GL/glut.h>

////////////////////////
///                  ///
/// Métodos Públicos ///
///                  ///
////////////////////////

Planning::Planning()
{
    newRobotPosition.x = 0;
    newRobotPosition.y = 0;

    robotPosition = newRobotPosition;

    newGridLimits.minX = newGridLimits.minY = 1000;
    newGridLimits.maxX = newGridLimits.maxY = -1000;

    gridLimits = newGridLimits;
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

    resetCellsTypes();

    // update robot position and grid limits using last position informed by the robot
    robotPosition = newRobotPosition;
    gridLimits = newGridLimits;

    updateCellsTypes();

    pthread_mutex_unlock(grid->mutex);

    initializePotentials();

    for(int i=0; i<100; i++){
        iteratePotentials();
    }

    updateGradient();
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
            c->planType = REGULAR;
        }
    }
}

void Planning::updateCellsTypes()
{
   // Cell* c;
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

    for (int y = limiteUL[1]; y >= limiteDL[1]; y--)
    {
        for (int x = limiteUL[0]; x <= limiteUR[0]; x++)
        {
           // printf("x=%d\n",x);
            Cell* c  = grid->getCell(x,y);
            int danger = 0;
            int x3,y3;

           /* if (c->planType == FRONTIER)
                printf("esta e front \n");*/

           if (c->occType == UNEXPLORED)
           {


              if (c->occType == UNEXPLORED) //se ainda é unexplored
              {
               xL = x-1;
             //  if (xL < limiteUL[0]) //pra n sair da área de análise
             //      xL = limiteUL[0];
               xR = x+1;
             //  if (xR > limiteUR[0])
             //      xR = limiteUR[0];
               yU = y+1;
             //  if (yU > limiteUL[1])
             //      yU = limiteUL[1];
               yD = y-1;
             //  if (yD < limiteDL[1])
             //      yD = limiteDL[1];

               int frontier = 0;

               Cell* d =grid->getCell(x,y);
               //->planType = FRONTIER;
              // printf("x=%d\n",x);


               for (int y2=yU; y2>=yD; y2--)
               {
                  for (int x2=xL; x2<=xR; x2++)
                  {
                     d = grid->getCell(x2,y2);
                      //printf("x2=%d\n",x2);

                     if (d->planType == REGULAR && d->occType == FREE)
                     {
                         frontier = 1;
                         c->planType = FRONTIER;
                         //printf("achei frontier\n");
                        // break;
                     }

                     if (d->planType == NEAR_WALLS)
                     {
                         frontier = 1;
                         c->planType = FRONTIER_NEAR_WALL;
                         //printf("achei frontier\n");
                         break;
                     }
                  }
                  if (frontier == 1)
                      break;
               }

              }

              if (c->himm <= limiarOCCtoFREE)
              {
                  c->occType = FREE;
                  c->planType = REGULAR;
              }
              else if (c->himm >= limiarFREEtoOCC)
              {
                  c->occType = OCCUPIED;
                  c->planType = REGULAR;
              }
           }

           if (c->occType == FREE)
           {
             if(c->himm > limiarFREEtoOCC)
             {
               c->occType = OCCUPIED;
               c->planType = REGULAR;
             }

             if (c->occType == FREE)
             {
                 /*if (c->occType != NEAR_WALLS)
                 c->planType = REGULAR;*/
                // c->planType = REGULAR;
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


                 Cell* d =grid->getCell(x,y);
                 //->planType = FRONTIER;
                // printf("x=%d\n",x);




                 for (y3=yU; y3>=yD; y3--)
                 {
                    for (x3=xL; x3<=xR; x3++)
                    {
                       d = grid->getCell(x3,y3);

                       if (d->occType == OCCUPIED)
                       {
                           danger = 1;
                           c->planType = DANGER;
                          //  printf("x3=%d, y3=%d\n",x3,y3);
                           //printf("achei frontier\n");
                          // c->planType = DANGER;
                         //  break;

                       }
                       if (danger == 1)
                           break;
                    }

                    if (danger == 1)
                        break;
                 }
                 if (danger == 0 && c->planType == DANGER)
                     c->planType = REGULAR;

              //   else
              //     c->planType = REGULAR;

             }
           }
           else if (c->occType == OCCUPIED && c->himm <= limiarOCCtoFREE)
           {
              c->occType = FREE;
              c->planType = REGULAR;
           }


           int xL2 = x-4;
            if (xL2 < limiteUL[0]) //pra n sair da área de análise
                xL2 = limiteUL[0];
           int xR2 = x+4;
            if (xR2 > limiteUR[0])
                xR2 = limiteUR[0];
           int yU2 = y+4;
            if (yU2 > limiteUL[1])
                yU2 = limiteUL[1];
           int yD2 = y-4;
            if (yD2 < limiteDL[1])
                yD2 = limiteDL[1];
            //printf("yU2=%d, yD2=%d\n",yU2,yD2);


           if (danger == 1)
           {  //printf("x3=%d, y3=%d\n",x3,y3); exit(0);
            for (int y4=yU2; y4>=yD2; y4--)
            {
               for (int x4=xL2; x4<=xR2; x4++)
               {
                   Cell* e = grid->getCell(x4,y4);
                  // e->planType = DANGER;
                 /*  if (d->occType == FREE)
                  d->planType = FRONTIER_NEAR_WALL;*/
                   //printf("x2=%d\n",x2);
                  // printf("x4=%d, y4=%d\n",x4,y4); //exit(0);
                  if (e->occType == FREE &&  e->planType != DANGER)
                   {

                  /*  if (e->planType == REGULAR)
                       e->planType = NEAR_WALLS;*/
                  //  if (danger == 1)
                    {
                     if (e->planType == FRONTIER)
                       e->planType = FRONTIER_NEAR_WALL;
                    else
                       e->planType = NEAR_WALLS;
                    }
                  /*  else
                    {
                     if (e->planType != FRONTIER && e->planType != DANGER)
                        e->planType = REGULAR;
                    }*/
                   }

               }

            }
           }
           /*if (danger == 1)
           {
            exit(0);
           }*/
           int regular = 0;
         if (c->planType == NEAR_WALLS)
         {
           for (int y4=yU2; y4>=yD2; y4--)
           {
              for (int x4=xL2; x4<=xR2; x4++)
              {
                  Cell* e = grid->getCell(x4,y4);
                 // e->planType = DANGER;
                /*  if (d->occType == FREE)
                 d->planType = FRONTIER_NEAR_WALL;*/
                  //printf("x2=%d\n",x2);
                 // printf("x4=%d, y4=%d\n",x4,y4); //exit(0);
                   {
                    if (e->planType == DANGER)
                    {
                        regular =1;
                        break;
                    }
                   }
                 /*  else
                   {
                    if (e->planType != FRONTIER && e->planType != DANGER)
                       e->planType = REGULAR;
                   }*/


              }
              if (regular == 1)
                  break;
           }
           if (regular == 0)
               c->planType = REGULAR;
         }



        }
    }

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
    // c->planType = NEAR_WALLS
    // c->planType = FRONTIER_NEAR_WALL





}

void Planning::initializePotentials()
{
    Cell *c;

    int minX = gridLimits.minX; //+ robotPosition.x;
    int minY = gridLimits.minY; //+ robotPosition.y;
    int maxX = gridLimits.maxX; //- robotPosition.x;
    int maxY = gridLimits.maxY; //- robotPosition.y;

    for(int j = minY; j<maxY; j++)
    {
        for(int i = minX; i<maxX; i++)
        {
            c = grid->getCell(i,j);
            if(c->occType == OCCUPIED || c->planType == DANGER)
            {
                c->pot[0] = 1;
                c->pot[1] = 1;
                c->pot[2] = 1;
            }
            else if(c->planType == FRONTIER)
            {
                c->pot[0] = 0;
                c->pot[1] = 0;
                c->pot[2] = 1;
            }
            else if(c->planType == FRONTIER_NEAR_WALL)
            {
                c->pot[0] = 0;
                c->pot[1] = 0;
                c->pot[2] = 0;
            }

            if(c->occType == FREE)
            {
             if(c->planType == NEAR_WALLS)
                 c->pref = 0.1;
             else
                 c->pref = -0.1;
            }
           /* c->pot[0] = 0;
            c->pot[1] = 1;
            c->pot[2] = 0.5;*/
        }
    }

    // the potential of a cell is stored in:
    // c->pot[i]
    // the preference of a cell is stored in:
    // c->pref

    // TODO: initialize the potential field in the known map
    //
    //  (gridLimits.minX, gridLimits.maxY)  -------  (gridLimits.maxX, gridLimits.maxY)
    //                  |                     \                      |
    //                  |                      \                     |
    //                  |                       \                    |
    //  (gridLimits.minX, gridLimits.minY)  -------  (gridLimits.maxX, gridLimits.minY)




}

void Planning::iteratePotentials()
{
    Cell* c;
    Cell* n;
    float left,right,up,down;

    int minX = gridLimits.minX; //+ robotPosition.x;
    int minY = gridLimits.minY; //+ robotPosition.y;
    int maxX = gridLimits.maxX; //- robotPosition.x;
    int maxY = gridLimits.maxY; //- robotPosition.y;


    // the update of a FREE cell in position (i,j) will use the potential of the four adjacent cells
    // where, for example:
    //     left  = grid->getCell(i-1,j);


    // TODO: iterate the potential field in the known map
    //
    //  (gridLimits.minX, gridLimits.maxY)  -------  (gridLimits.maxX, gridLimits.maxY)
    //                  |                     \                      |
    //                  |                      \                     |
    //                  |                       \                    |
    //  (gridLimits.minX, gridLimits.minY)  -------  (gridLimits.maxX, gridLimits.minY)

    for(int j = minY; j<=maxY; j++)
    {
        for(int i = minX; i<=maxX; i++)
        {
            c = grid->getCell(i,j);

           for(int k =0; k<3; k++)
           {

            if(c->occType == FREE)
            {

                    n = grid->getCell(i+1,j);
                    right = n->pot[k];
                    n = grid->getCell(i-1,j);
                    left = n->pot[k];
                    n = grid->getCell(i,j+1);
                    up = n->pot[k];
                    n = grid->getCell(i,j-1);
                    down = n->pot[k];

                    float result = (up+down+left+right)/4;

                    if(k == 1)
                    {
                        float d = fabs((right - left)/2) + fabs((up-down)/2);
                        result = result- (c->pref/4)*d;
                    }
                    c->pot[k] = result;
                }

            }

        }
    }

}

void Planning::updateGradient()
{

    // the components of the descent gradient of a cell are stored in:
    // c->dirX[i] and c->dirY[i], for pot[i]

    // the gradient of a FREE cell in position (i,j) is computed using the potential of the four adjacent cells
    // where, for example:
    //     left  = grid->getCell(i-1,j);


    // TODO: compute the gradient of the FREE cells in the known map
    //
    //  (gridLimits.minX, gridLimits.maxY)  -------  (gridLimits.maxX, gridLimits.maxY)
    //                  |                     \                      |
    //                  |                      \                     |
    //                  |                       \                    |
    //  (gridLimits.minX, gridLimits.minY)  -------  (gridLimits.maxX, gridLimits.minY)
    Cell* c;
    Cell* n;
    float left,right,up,down;

    int minX = gridLimits.minX;// + robotPosition.x;
    int minY = gridLimits.minY;// + robotPosition.y;
    int maxX = gridLimits.maxX;// - robotPosition.x;
    int maxY = gridLimits.maxY;// - robotPosition.y;


    // the update of a FREE cell in position (i,j) will use the potential of the four adjacent cells
    // where, for example:
    //     left  = grid->getCell(i-1,j);


    // TODO: iterate the potential field in the known map
    //
    //  (gridLimits.minX, gridLimits.maxY)  -------  (gridLimits.maxX, gridLimits.maxY)
    //                  |                     \                      |
    //                  |                      \                     |
    //                  |                       \                    |
    //  (gridLimits.minX, gridLimits.minY)  -------  (gridLimits.maxX, gridLimits.minY)



      for(int j = minY; j<maxY; j++)
      {
          for(int i = minX; i<maxX; i++)
          {
              for(int k =0; k<3; k++)
              {
                c = grid->getCell(i,j);

                if(c->occType == FREE)
                {

                        n = grid->getCell(i+1,j);
                        right = n->pot[k];
                        n = grid->getCell(i-1,j);
                        left = n->pot[k];
                        n = grid->getCell(i,j+1);
                        up = n->pot[k];
                        n = grid->getCell(i,j-1);
                        down = n->pot[k];

                        float resultY = -(up-down)/2;
                        float resultX = -(right-left)/2;

                        float norma = sqrt(pow(resultX,2)+pow(resultY,2));


                        if (norma != 0)
                        {
                         c->dirX[k] = resultX/norma;
                         c->dirY[k] = resultY/norma;
                        }
                        else
                        {
                         c->dirX[k] = resultX;
                         c->dirY[k] = resultY;
                        }
                }
                else
                {
                 c->dirX[k] = 0;
                 c->dirY[k] = 0;
                }


            }

        }
    }

}

