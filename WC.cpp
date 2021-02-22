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


    printf("l,f,r = %f,%f,%f/n", minLeftLaser,minFrontLaser,minRightLaser);

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