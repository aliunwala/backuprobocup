#include <localization/LocalizationModule.h>
#include <vision/ObjectDetector.h>
#include <common/Field.h>

void LocalizationModule::specifyMemoryDependency() {
  requiresMemoryBlock("world_objects");
  requiresMemoryBlock("localization");
  requiresMemoryBlock("team_packets");
  requiresMemoryBlock("vision_frame_info");
  requiresMemoryBlock("vision_odometry");
  requiresMemoryBlock("robot_state");
  requiresMemoryBlock("game_state");
  requiresMemoryBlock("vision_joint_angles");
  requiresMemoryBlock("behavior");
  requiresMemoryBlock("vision_processed_sonar");
  requiresMemoryBlock("delayed_localization");
}

void LocalizationModule::specifyMemoryBlocks() {
  getOrAddMemoryBlock(worldObjects,"world_objects");
  getOrAddMemoryBlock(localizationMem,"localization");
  getOrAddMemoryBlock(teamPacketsMem,"team_packets");
  getOrAddMemoryBlock(frameInfo,"vision_frame_info");
  getOrAddMemoryBlock(odometry,"vision_odometry");
  getOrAddMemoryBlock(robotState,"robot_state");
  getOrAddMemoryBlock(gameState,"game_state");
  getOrAddMemoryBlock(jointAngles,"vision_joint_angles");
  getOrAddMemoryBlock(behaviorMem,"behavior");
  getOrAddMemoryBlock(processedSonar,"vision_processed_sonar");
  getOrAddMemoryBlock(delayedLocalization,"delayed_localization");
}


void LocalizationModule::initSpecificModule(){
  for(int i = 0; i < NUM_PARTICLES; i++) {
    Particle& p = particles_[i];
    p.loc.x = 0;
    p.loc.y = 0;
    p.theta = 0;
    p.prob = 1.0;
  }
  resetParticles();
  copyParticles();
  sampleRate = 0;
  sampleMod = 3;
  globalSumX = 0;
  globalSumY = 0;
  globalTheta = 0;
}

void LocalizationModule::processFrame() {
  int frameID = frameInfo->frame_id;

  // 1. Update particles from observations
    // Circle based weighting
        float beaconx ; 
        float beacony ;
        float perceiveddist; //=300;


        // for each beacon
        // WorldObject* beacon_blue_yellow = &vblocks_.world_object->objects_[WO_BEACON_BLUE_YELLOW]; 
        // WorldObject* beacon = world_object->objects_[WO_BEACON_PINK_BLUE];
        int beaconArray[6] = {WO_BEACON_YELLOW_BLUE, WO_BEACON_BLUE_YELLOW, 
                             WO_BEACON_PINK_BLUE, WO_BEACON_BLUE_PINK, 
                             WO_BEACON_YELLOW_PINK, WO_BEACON_PINK_YELLOW};


            // set landmark locations
        int numSeen = 0;
        for (int i = 0; i < NUM_LANDMARKS; i++) {
          WorldObject *wo = &worldObjects->objects_[i + LANDMARK_OFFSET];
          wo->loc = landmarkLocation[i];
          if(wo->seen)
          {
            numSeen++;
          }
        }

        int countDegraded = 0;
        for(int i = 0; i < 6 ; i++) {
        WorldObject* beacon = &worldObjects->objects_[ beaconArray[i] ];
        if (beacon->seen){
          perceiveddist = beacon->distance;
          beaconx = beacon->loc.x;
          beacony = beacon->loc.y;
          // printf("x: %f y: %f\n",beaconx, beacony );
          // printf("x: %f y: %f\n",beacon->loc.x, beacon->loc.y );

            // for each paricle
            for(int j = 0; j < NUM_PARTICLES; j++) {
              // look at the dist betwenthe particle and the becaon
              // if the dist is close update
              bool degradedThisIteration = false;
              float x = particles_[j].loc.x;
              float y = particles_[j].loc.y;
              float thet = particles_[j].theta;
              float distToBeaconFromParticle =  sqrt((x-beaconx)*(x-beaconx)    +  (y-beacony)*(y-beacony) );  
              if (  abs(perceiveddist - distToBeaconFromParticle) > 100  ){
                particles_[j].degradeProbability(.3);
                degradedThisIteration =  true;
              }

              Pose2D bPose(0, beaconx, beacony);
              Pose2D pPose(thet, x, y);
              Pose2D bPoseRel = bPose.globalToRelative(pPose);
              AngRad relBearing = atan2f(bPoseRel.translation.y, bPoseRel.translation.x);
              
              if((relBearing > M_PI/6.0 || relBearing < -M_PI/6.0) && numSeen > 1)
              {
                particles_[j].degradeProbability(.5);
                degradedThisIteration =  true;
              }
              else if((relBearing > M_PI/12.0 || relBearing < -M_PI/12.0) && numSeen > 1)
              {
                particles_[j].degradeProbability(.7);
                degradedThisIteration =  true;
              }

              if(degradedThisIteration ==  true){
                countDegraded ++;
              }

            }
          }
        }
        // printf("degradednum: %d\n", countDegraded );
        // if( countDegraded == (NUM_PARTICLES*numSeen)  && numSeen >= 1 ) {  // We are very uncertain of what is going on try again
        //   resetParticles();
        // }


              // float temptheta = atanf(  (beacon->loc.y - particles_[j].loc.y)/(beacon->loc.x - particles_[j].loc.x)   );
              
              // float thetadiff = abs(temptheta - particles_[j].theta );
              // if (thetadiff < .5){
              // printf("temptheta %f\n", temptheta*(180/3.14) );
              // printf("realtheta %f\n", particles_[j].theta*(180/3.14) );
              // printf("tempdiff %f\n", thetadiff*(180/3.14) );

              //   }
              // if (!(thetadiff > 0.78645)){
              //   particles_[j].degradeProbability(.7);
              // }



              // float offsetBX = cosf(beacon->visionBearing + particles_[j].theta) * beacon->visionDistance;
              // float offsetBY = sinf(beacon->visionBearing + particles_[j].theta) * beacon->visionDistance;
              // float projBX = offsetBX + particles_[j].loc.x;
              // float projBY = offsetBY + particles_[j].loc.y;

              // // printf("PROJBX %f   PROJBY %f     ACTUAL(%f, %f)\n", projBX, projBY, beaconx, beacony);

              // // if(abs(beaconx-projBX) > 250 || abs(beacony-projBY) > 250)
              // float realBeaconTheta = atanf(  (beacon->loc.y - particles_[j].loc.y)/(beacon->loc.x - particles_[j].loc.x)   );
              // float projBeaconTheta = atanf(  (projBY - particles_[j].loc.y)/(projBX - particles_[j].loc.x)   );
              // // printf("REALBEACON: %f    FAKEBEACON: %f\n", realBeaconTheta, projBeaconTheta);
              // if(abs(realBeaconTheta-projBeaconTheta) > M_PI/12.0)
              // {
              //   particles_[j].degradeProbability(.3);
              // }
              // else if(abs(realBeaconTheta-projBeaconTheta) > M_PI/8.0)
              // {
              //   particles_[j].degradeProbability(.5);
              // }
              // else if(abs(realBeaconTheta-projBeaconTheta) > M_PI/4.0)
              // {
              //   particles_[j].degradeProbability(.7);
              // }






              // if (!( beacon->loc.y>0  && particles_[j].theta>0) 
              //   || !( beacon->loc.y<0  && particles_[j].theta<0) ){
              //   particles_[j].degradeProbability(.5);
              // }
  // 4. If this is a random walk frame, random walk
    // Initally all particles are unifromly distributed
    // Spread each of the particles out based on the random methods provided
    // randomWalkParticles();
    // randomWalkParticles();
    // randomWalkParticles();
  // 2. If this is a resampling frame, resample
    // Some points are not the same weight redisribute the points based on weight
    // weights should all be the same/low
      
      sampleRate++;
      // if (sampleRate%sampleMod == 0){
        resample();
      // }
      updateParticlesFromOdometry();
  // 3. Update the robot's pose
    // Iterate overall the points and find the average x,y by mass
      // This may be an bad idea if you have two large points of mass... Two circles intersecting 
  // if (sampleRate%2 == 0){
    WorldObject& self = worldObjects->objects_[robotState->WO_SELF];
      double sumprob = 0;
    // if(sampleRate%sampleMod==0)
    // {
      double sumx = 0;
      double sumy = 0;
      AngRad sumtheta =0;
      for (int i = 0;  i < NUM_PARTICLES ; i++){
        sumprob = sumprob + particles_[i].prob;
        sumx += particles_[i].loc.x * particles_[i].prob;
        sumy += particles_[i].loc.y * particles_[i].prob;
        sumtheta += particles_[i].theta * particles_[i].prob;
      }

      globalSumX = sumx;///NUM_PARTICLES;
      globalSumY = sumy;///NUM_PARTICLES;
      // if(sumtheta > 0)
      // {
      //     globalTheta =  fmod(sumtheta + M_PI, M_PI*2) - M_PI;
      // }
      // else
      // {
      //     globalTheta =  fmod(sumtheta - M_PI, -M_PI*2) + M_PI;
      // }

      globalTheta = ((sumtheta >=0) ? 1 : -1)  * (fmod(abs(sumtheta),(M_PI)));
      // printf("GLOBAL THETA: %f -> %f\n", sumtheta, globalTheta);
      ///NUM_PARTICLES;
    // }

    self.loc.x = globalSumX;
    self.loc.y = globalSumY;
    self.orientation = globalTheta;
    // self.loc.x = 10;
    // self.loc.y = 10;
    // self.orientation = 3;
    // printf("sumprob: %f \n", sumprob );
  // }

  // self.orientation = 1.7;
  // double actualDegrees = (self.orientation * 180.0) / (3.1459);
  // printf("x: %f y: %f orientation: %f degrees: %f \n", self.loc.x, self.loc.y, self.orientation, actualDegrees);


  // 5. Copy particles to localization memory:
  copyParticles();
}

void LocalizationModule::copyParticles() {
  memcpy(localizationMem->particles, particles_, NUM_PARTICLES * sizeof(Particle));
}

void LocalizationModule::updateParticlesFromOdometry() {
  Pose2D disp = odometry->displacement;
  for(int i = 0; i < NUM_PARTICLES; i++) {
    Particle& p = particles_[i];
    p.moveRelative(disp);
    p.degradeProbability(DEGRADE_FACTOR);
  }
}

void LocalizationModule::resetParticles(){
  for (int i = 0; i < NUM_PARTICLES; i++){
    Particle& p = particles_[i];
    p.prob = 1.0f/NUM_PARTICLES;
    p.placeRandomly();
  }
}

void LocalizationModule::setParticleProbabilities(float newProb){
  for (int i = 0; i < NUM_PARTICLES; i++){
    Particle& p = particles_[i];
    p.prob = newProb;
  }
}

void LocalizationModule::randomWalkParticles() {
  // loop through half, moving each pair of particles opposite directions

  for ( int i = 0; i < NUM_PARTICLES/2; i++ ) {
    Particle& part1 = particles_[i];
    Particle& part2 = particles_[i+NUM_PARTICLES/2];

    Vector2D dPos(DELTA_DIST * (2.0 * drand48() - 1),
                  DELTA_DIST * (2.0 * drand48() - 1));
    AngRad dAng = DELTA_ANG  * (2.0 * drand48() - 1);

    // move them in opposite directions on this vector, based on their prob
    float p1Ratio = 1.0 - part1.prob;
    float p2Ratio = 1.0 - part2.prob;

    float p1AngleRatio = p1Ratio;
    float p2AngleRatio = p2Ratio;

    part1.move(dPos*p1Ratio, p1AngleRatio*dAng);
    part2.move(-dPos*p2Ratio,p2AngleRatio*-dAng);

  }
}

void LocalizationModule::updatePose() {
  WorldObject& self = worldObjects->objects_[robotState->WO_SELF];
  // Compute a weighted average of the particles to fill in your location
}



void LocalizationModule::resample() {
    //Sum all weights

    float sum = 0;
    for(int i = 0 ; i < NUM_PARTICLES ;  i ++ ){
      sum += particles_[i].prob; 
    }  

    std::vector<Point2D> listofpoints;
    std::vector<float> listofprob; // Corresponding indexes for the point
    std::vector<float> listofprob2; // Corresponding indexes for the point
    std::vector<AngRad> listoftheta;

    for (int i = 0 ; i < NUM_PARTICLES;  i++){
        bool inListOfPoints = false;
          // printf("Before %d\n", inListOfPoints);
        for(int j = 0 ;  j < listofpoints.size() ; j++ ){
          // if equal to another point in the list increment prob
          if (listofpoints[j].x == particles_[i].loc.x  && 
              listofpoints[j].y == particles_[i].loc.y && listoftheta[j] == particles_[i].theta){ //////////////////////////////////////////////FIX THETA
            listofprob[j] += particles_[i].prob; 
            listofprob2[j] = particles_[i].prob; 
            inListOfPoints = true;
          }
        }
        // printf("After %d\n", inListOfPoints);

        // not part of the prob or list
        if(!inListOfPoints){
          listofpoints.push_back( particles_[i].loc);
          listofprob.push_back( particles_[i].prob); 
          listofprob2.push_back( particles_[i].prob); 
          listoftheta.push_back(particles_[i].theta);
        }
    }

    if ((listoftheta.size() != listofprob.size())){
        printf("ERROR 1--------------------------------------------------------------------------------------\n");
    }
    if ((listoftheta.size() != listofpoints.size())){
        // printf("listoftheta:%d \n",listoftheta.size());
        // printf("listofpoints:%d \n",listofpoints.size());
        printf("ERROR 2--------------------------------------------------------------------------------------\n");
    }

    for (int i  = 0 ; i < listofprob.size(); i++ ){
      listofprob[i] = listofprob[i]/sum;
      // listofprob[i] = NUM_PARTICLES * listofprob[i];  // May work... # of vectors to have at that point for next iteration
    }
    // float mysum = 0 ; 
    // for (int i  = 0 ; i < listofprob.size(); i++ ){
    //   mysum += listofprob[i]; 
    // }
    // for (int i  = listofprob.size()-1 ; i >= 0 ; i-- ){
    //   printf("%f\n", listofprob[i]);
    // }


    // std::vector<float> listofprobSummed; // Corresponding indexes for the point
    for (int i  = listofprob.size()-1 ; i >= 0 ; i-- ){
      float tempsum=0;
      for (int j  = i  ; j >= 0 ; j-- ){
        tempsum += listofprob[j];    
      }
      // listofprobSummed[i] += tempsum;
      listofprob[i] += tempsum;
    }

    // printf("bottom\n");
    // for (int i  = listofprob.size()-1 ; i >= 0 ; i-- ){
    //   printf("%f\n", listofprob[i]);
    // }
    // printf("\n\n\n\n");
    float totalProb = 0.0;
    int totalCountPart = 0;
    for (int i = 0 ; i < NUM_PARTICLES ; i++ ){
      float myrand = (float)rand()/(float)RAND_MAX;
      // printf("myrand %f\n", myrand);
      for (int j  = 0 ; j < listofprob.size(); j++ ){
        // if (myrand > listofprob[j]  &&  myrand < listofprob[j+1]){
            // printf("j: %f j+1: %f\n", listofprob[j], listofprob[j+1]);
          // if( abs(myrand-listofprob[j])  <=  abs(myrand-listofprob[j+1]) ){
          if (myrand<=listofprob[j]){
            // printf("if\n");
            Particle p ;
            p.loc.x = listofpoints[j].x;
            p.loc.y = listofpoints[j].y;
            p.theta = listoftheta[j];
            p.prob = 1.0/NUM_PARTICLES;
            // printf("prob:%f\n", p.prob );
            // p.prob = listofprob2[j+1];
            totalProb += p.prob;
            totalCountPart++;

            particles_[i] = p;
            break;
          }
          // else{ // PROBLEM??? for inital postition?
          //   Particle p ;
          //   p.loc.x = listofpoints[1].x;
          //   p.loc.y = listofpoints[1].y;
          //   p.theta = listoftheta[1];
          //   p.prob = 1.0;
          //   particles_[i] = p; 
          // }
       }
      }
      printf("CountPart %d\n", totalCountPart);
    //   float sumprob = 0;
    // for (int i = 0;  i < NUM_PARTICLES ; i++){
    //     sumprob = sumprob + particles_[i].prob;
    //   }
    // printf("sumprob: %f \n", sumprob );
      // for (int i = 0 ; i < NUM_PARTICLES ; i++ ){
      //   particles_[i].prob /= totalProb;
      //  }
 }     
