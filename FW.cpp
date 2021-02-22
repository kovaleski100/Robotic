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
    float Td = 0.1;
    float Ti = 0.000;
    float sp = 1; // vai ser a distancia pra ele ficar da parede

    float result_sensor = 0;

    if (minFrontSonar < 1) // muito perto
    {
        if (minFrontSonar < 0.6) // pra ter certeza que nao vai bater
            linVel = 0;
        else
            linVel = linVel/2;
    }
    if(minLeftSonar<=minRightSonar)
    {
        result_sensor = minLeftSonar;
    }
    else
    {
        result_sensor = minRightSonar;
    }

       // result_sensor = minLeftLaser - minRightLaser;// se <0, esquerda está mais perto

    if(result_sensor < 1*0.7) //serve como threshould pra mudar de parede dps de escolher uma
        chosenWall = 0;

    if (chosenWall == 0)
     {
     if(minLeftSonar<=minRightSonar)
     {
        //result_sensor = minLeftSonar;
        isFollowingLeftWall_ = true;
        chosenWall = -1;
     }
      else
     {
        //result_sensor = minRightSonar;
        isFollowingLeftWall_ = false;
        chosenWall = 1;
     }
    }
    /*result_sensor = minRightSonar;
    isFollowingLeftWall_ = false;*/

    CTE = sp - result_sensor; // sp sempre é zero aqui

    CTE_sum += CTE;
    float New_CTE = (CTE) - last_error; // vai dar a diferença erro atual com o erri anterior
    last_error = New_CTE; //fica salvo pra proxima iteração


    angVel = -(Tp*(CTE)) - (Td*New_CTE) - (Ti* CTE_sum);
   // angVel*=(-1);
    /*-0.75 -  -(0.3 - 0) < 0
    -0.75 -  0.3 - 0*/
    //-0.5 - ()
    printf("l,f,r = %f,%f,%f/n", minLeftSonar,minFrontSonar,minRightSonar);
   // printf("Tp,Td,newCTE = %f,%f,%f/n", (-Tp*CTE),(-Td*New_CTE),(New_CTE));

    if (isFollowingLeftWall_ == false) // inverte os resultados para seguir a parede da direita
        angVel*=(-1);

    if(isFollowingLeftWall_)
        std::cout << "Following LEFT wall" << std::endl;
    else
        std::cout << "Following RIGHT wall" << std::endl;

    //TODO - implementar wall following usando PID


    base.setWheelsVelocity_fromLinAngVelocity(linVel, angVel);
}