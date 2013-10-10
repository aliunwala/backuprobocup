#!/usr/bin/env python

import core
from task import Task, HeadBodyTask, MachineTask
import pose, head, kicks
import math
import commands, cfgstiff
import testFSM
import percepts
#from numpy import matrix
#from numpy import linalg


def areDistinct(state1, state2):
  if state1 == core.INITIAL and state2 == core.FINISHED: return False
  if state1 == core.FINISHED and state2 == core.INITIAL: return False
  if state1 == state2: return False
  return True

def createStateTask(state):
  if state == core.INITIAL: return Initial()
  if state == core.READY: return Ready()
  if state == core.PLAYING: return Playing()
  if state == core.FINISHED: return Finished()
  if state == core.TESTING: return Testing()
  if state == core.PENALISED: return Penalised()
  return None

def inDuration(beginTime, endTime, self):
  if self.getTime() >= beginTime and endTime==-1.0:
    return True
  elif self.getTime() >= beginTime and self.getTime() < endTime:
    return True
  else:
    return False


Penalised = Initial = Finished = pose.Sit

# class KalmanFilter:
#   dt = 1.0/30.0
#   A = matrix([[1,2,3],[1,2,3]])
#   B = matrix([[1,2,3],[1,2,3]])
#   C = matrix([[1,2,3],[1,2,3]])

class Ready(Task):
  # def __init__(self):
  #   HeadBodyTask.__init__(self,
  #     head.Scan(period = 3.0, maxPan = 105.0 * core.DEG_T_RAD, numSweeps = 4),
  #     pose.Stand()
  #   )

  # def run(self):
  #   commands.setStiffness()
  #   HeadBodyTask.run(self)
  sampleSum = 0
  errorSamples = []
  rotateCounter = 0
  kickStart = -1
  seenPercentage = True
  percInARow = 0


  def run(self):
    ball = core.world_objects.getObjPtr(core.WO_BALL)
    goal = core.world_objects.getObjPtr(core.WO_OPP_GOAL)
    goalline = core.world_objects.getObjPtr(core.WO_OPP_GOAL_LINE)
    imageWidth = 320.0
    imageHeight = 240.0
    xc = imageWidth/2
    ballLinedUpForKick = ball.imageCenterX>= (imageWidth*.35) and ball.imageCenterX<= (imageWidth*.65) and not (ball.imageCenterX<= (imageWidth*.55) and ball.imageCenterX>= (imageWidth*.45))
    ballLinedUpForDribble = ball.imageCenterX<= (imageWidth*.65) and ball.imageCenterX>= (imageWidth*.35) and ball.imageCenterY >(imageHeight*.8)
    canKick  = ball.seen and ball.imageCenterY >(imageHeight*.6) and ballLinedUpForKick
    
    #print("PERCINAROW: " + str(self.percInARow) + "\n")
    if not self.seenPercentage and goal.seen and goal.radius >= 0.10:
      self.percInARow+= 1
      if(self.percInARow >= 4):
        self.seenPercentage = True
    elif not self.seenPercentage:
      self.percInARow = 0

    if inDuration(0.0, 2.0, self): ##START UP
      commands.setStiffness()
      commands.stand()
      commands.setHeadTilt(-21)
    elif inDuration(self.kickStart, self.kickStart+2.0, self): ##START KICK
      if canKick:
        core.speech.say('KICK')
      else:
        self.kickStart = -1
    elif inDuration(self.kickStart+2.0, self.kickStart+3.0, self): ##KICK NOW
      core.speech.say('NOW')
      kreq = core.kick_request
      kreq.ball_seen_ = ball.seen
      kreq.ball_image_center_x_ = ball.imageCenterX
      kreq.ball_image_center_y_ = ball.imageCenterY
      kreq.kick_running_ = True
      kickFoot = core.Kick.RIGHT
      if( ball.imageCenterX < xc):
        kickFoot = core.Kick.LEFT
      kreq.set(core.Kick.STRAIGHT, kickFoot, 0, 2000)
    elif inDuration(self.kickStart+3.0, self.kickStart+5.0, self): ##END KICK
      kreq = core.kick_request
      kreq.kick_running = False

    ############################### MAIN FUNCTIONALITY
    else:
      if not goalline.seen:
        if ball.seen:#BALL HAS BEEN SEEN
          extremeAngle = math.pi
          angleError = -((ball.imageCenterX * ((extremeAngle*2.0)/imageWidth)) - extremeAngle)

          lastError = 0.0
          timeStep = 90
          if len(self.errorSamples)+1 > timeStep :
            lastError = self.errorSamples[timeStep-1]
          elif len(self.errorSamples)>0 :
            lastError = self.errorSamples[len(self.errorSamples)-1]

          #Calculate pidAngle
          pidP = angleError
          pidI = self.sampleSum
          pidD = (angleError - lastError)/timeStep

          kP = 0.5
          kI = 0.0
          kD = 0.0

          pidAngle = kP*pidP + kI*pidI + kD*pidD
          if pidAngle > extremeAngle:
            pidAngle = extremeAngle;
          elif pidAngle < -extremeAngle:
            pidAngle = -extremeAngle;

          #print('P=' + str(pidP) + '  I=' + str(pidI) + '  D=' + str(pidD) + '  newAngle=' + str(pidAngle))

          maxSamples = 1000
          self.errorSamples.append(angleError)
          self.sampleSum += angleError
          if len(self.errorSamples)+1 == maxSamples :
            self.sampleSum -= self.errorSamples.pop()

          #set the walkspeed############################################################
          walkSpeed = 0.3
          if not ball.fromTopCamera:
            #ball is in bottom camera
            walkSpeed = 0.1


          ##IS THE BALL CLOSE?
          if not ball.fromTopCamera: ##BALL IS CLOSE
            ##IS THE GOAL SEEN?
            if goal.seen: ##GOAL IS SEEN
              ##IS THE GOAL ALIGNED WITH THE BALL
              goalThresh = 30
              centerThresh = 40
              if ((xc-centerThresh) <= ball.imageCenterX and (xc+centerThresh) >= ball.imageCenterX and (goal.imageCenterX-goalThresh) <= ball.imageCenterX and (goal.imageCenterX+goalThresh) >= ball.imageCenterX): ##GOAL IS ALIGNED
                if not self.seenPercentage: ##BALL IS FAR FROM GOAL
                  if ballLinedUpForDribble:
                    core.speech.say('DRIBBLE')
                    commands.setWalkVelocity(0.7, 0.0, 0.0)
                  else:
                    commands.setWalkVelocity(-0.1, 0.0, 0.0)
                  pass
                else: ##BALL IS CLOSE ENOUGH TO GOAL TO KICK
                  if ball.imageCenterY >(imageHeight*.8) and ballLinedUpForKick:
                    commands.stand()
                    self.kickStart = self.getTime()
                  elif not ballLinedUpForKick and ball.imageCenterY > (imageHeight*.8): ##BALL ISN'T ON GOOD FEET
                    ##SIDE STEP TO SET UP BALL
                    commands.setWalkVelocity(-0.1, 0.0, 0.0)

                  else: ##BALL ISN'T CLOSE TO FEET
                    ##WALK CLOSE TO BALL
                    core.speech.say('WALK')
                    commands.setWalkVelocity(0.1, 0.0, 0.0)

              else: ##GOAL IS NOT ALIGNED
                sideStepDir = ball.imageCenterX - goal.imageCenterX
                sideStepSpeed = 0.0
                if (sideStepDir > 0):
                  sideStepSpeed = -0.4
                else:
                  sideStepSpeed = 0.4
                core.speech.say('ALIGN')
                commands.setWalkVelocity(0.0, sideStepSpeed, pidAngle)
            else: ##GOAL IS NOT SEEN
              #rotate around the ball until goal is visible
              core.speech.say('ROTATE')
              commands.setWalkVelocity(0.0, 1.0, -0.2)

          else: ##BALL IS FAR
            commands.setWalkVelocity(walkSpeed, 0.0, pidAngle)



        else:#BALL HAS NOT BEEN SEEN
          core.speech.say('SEARCH')
          commands.setWalkVelocity(0.0, 0.0, 0.2)
          self.sampleSum = 0
          self.errorSamples = []
      else:
        core.speech.say('WHITE')
        #print('WHITE\n')
        commands.stand()


class Playing(Task):
  sampleSum = 0
  errorSamples = []
  rotateCounter = 0
  kickStart = -1
  seenPercentage = False
  percInARow = 0


  def run(self):
    ball = core.world_objects.getObjPtr(core.WO_BALL)
    goal = core.world_objects.getObjPtr(core.WO_OPP_GOAL)
    goalline = core.world_objects.getObjPtr(core.WO_OPP_GOAL_LINE)
    imageWidth = 320.0
    imageHeight = 240.0
    xc = imageWidth/2
    ballLinedUpForKick = ball.imageCenterX>= (imageWidth*.35) and ball.imageCenterX<= (imageWidth*.65) and not (ball.imageCenterX<= (imageWidth*.55) and ball.imageCenterX>= (imageWidth*.45))
    ballLinedUpForDribble = ball.imageCenterX<= (imageWidth*.7) and ball.imageCenterX>= (imageWidth*.3) and ball.imageCenterY >(imageHeight*.8)
    canKick  = ball.seen and ball.imageCenterY >(imageHeight*.6) and ballLinedUpForKick
    
    #print("PERCINAROW: " + str(self.percInARow) + "\n")
    if not self.seenPercentage and goal.seen and goal.radius >= 0.10:
      self.percInARow+= 1
      if(self.percInARow >= 4):
        self.seenPercentage = True
    elif not self.seenPercentage:
      self.percInARow = 0

    if inDuration(0.0, 2.0, self): ##START UP
      commands.setStiffness()
      commands.stand()
      commands.setHeadTilt(-21)
    elif inDuration(self.kickStart, self.kickStart+2.0, self): ##START KICK
      if canKick:
        core.speech.say('KICK')
      else:
        self.kickStart = -1
    elif inDuration(self.kickStart+2.0, self.kickStart+3.0, self): ##KICK NOW
      core.speech.say('NOW')
      kreq = core.kick_request
      kreq.ball_seen_ = ball.seen
      kreq.ball_image_center_x_ = ball.imageCenterX
      kreq.ball_image_center_y_ = ball.imageCenterY
      kreq.kick_running_ = True
      kickFoot = core.Kick.RIGHT
      if( ball.imageCenterX < xc):
        kickFoot = core.Kick.LEFT
      kreq.set(core.Kick.STRAIGHT, kickFoot, 0, 2000)
    elif inDuration(self.kickStart+3.0, self.kickStart+5.0, self): ##END KICK
      kreq = core.kick_request
      kreq.kick_running = False

    ############################### MAIN FUNCTIONALITY
    else:
      if not goalline.seen:
        if ball.seen:#BALL HAS BEEN SEEN
          extremeAngle = math.pi
          angleError = -((ball.imageCenterX * ((extremeAngle*2.0)/imageWidth)) - extremeAngle)

          lastError = 0.0
          timeStep = 90
          if len(self.errorSamples)+1 > timeStep :
            lastError = self.errorSamples[timeStep-1]
          elif len(self.errorSamples)>0 :
            lastError = self.errorSamples[len(self.errorSamples)-1]

          #Calculate pidAngle
          pidP = angleError
          pidI = self.sampleSum
          pidD = (angleError - lastError)/timeStep

          kP = 0.5
          kI = 0.0
          kD = 0.0

          pidAngle = kP*pidP + kI*pidI + kD*pidD
          if pidAngle > extremeAngle:
            pidAngle = extremeAngle;
          elif pidAngle < -extremeAngle:
            pidAngle = -extremeAngle;

          #print('P=' + str(pidP) + '  I=' + str(pidI) + '  D=' + str(pidD) + '  newAngle=' + str(pidAngle))

          maxSamples = 1000
          self.errorSamples.append(angleError)
          self.sampleSum += angleError
          if len(self.errorSamples)+1 == maxSamples :
            self.sampleSum -= self.errorSamples.pop()

          #set the walkspeed############################################################
          walkSpeed = 0.3
          if not ball.fromTopCamera:
            #ball is in bottom camera
            walkSpeed = 0.1


          ##IS THE BALL CLOSE?
          if not ball.fromTopCamera: ##BALL IS CLOSE
            ##IS THE GOAL SEEN?
            if goal.seen: ##GOAL IS SEEN
              ##IS THE GOAL ALIGNED WITH THE BALL
              goalThresh = 30
              centerThresh = 40
              if ((xc-centerThresh) <= ball.imageCenterX and (xc+centerThresh) >= ball.imageCenterX and (goal.imageCenterX-goalThresh) <= ball.imageCenterX and (goal.imageCenterX+goalThresh) >= ball.imageCenterX): ##GOAL IS ALIGNED
                if not self.seenPercentage: ##BALL IS FAR FROM GOAL
                  if ballLinedUpForDribble:
                    core.speech.say('DRIBBLE')
                    commands.setWalkVelocity(0.3, 0.0, 0.0)
                  else:
                    commands.setWalkVelocity(-0.1, 0.0, 0.0)
                  pass
                else: ##BALL IS CLOSE ENOUGH TO GOAL TO KICK
                  if ball.imageCenterY >(imageHeight*.8) and ballLinedUpForKick:
                    commands.stand()
                    self.kickStart = self.getTime()
                  elif not ballLinedUpForKick and ball.imageCenterY > (imageHeight*.8): ##BALL ISN'T ON GOOD FEET
                    ##SIDE STEP TO SET UP BALL
                    commands.setWalkVelocity(-0.1, 0.0, 0.0)

                  else: ##BALL ISN'T CLOSE TO FEET
                    ##WALK CLOSE TO BALL
                    core.speech.say('WALK')
                    commands.setWalkVelocity(0.1, 0.0, 0.0)

              else: ##GOAL IS NOT ALIGNED
                sideStepDir = ball.imageCenterX - goal.imageCenterX
                sideStepSpeed = 0.0
                if (sideStepDir > 0):
                  sideStepSpeed = -0.4
                else:
                  sideStepSpeed = 0.4
                core.speech.say('ALIGN')
                commands.setWalkVelocity(0.0, sideStepSpeed, pidAngle)
            else: ##GOAL IS NOT SEEN
              #rotate around the ball until goal is visible
              core.speech.say('ROTATE')
              commands.setWalkVelocity(0.0, 1.0, -0.2)

          else: ##BALL IS FAR
            commands.setWalkVelocity(walkSpeed, 0.0, pidAngle)



        else:#BALL HAS NOT BEEN SEEN
          core.speech.say('SEARCH')
          commands.setWalkVelocity(0.0, 0.0, 0.2)
          self.sampleSum = 0
          self.errorSamples = []
      else:
        core.speech.say('WHITE')
        #print('WHITE\n')
        commands.stand()

class Testing(Task):
  blockStart = -10000
  searchDirection = 0


  def run(self):
    ball = core.world_objects.getObjPtr(core.WO_BALL)
    goal = core.world_objects.getObjPtr(core.WO_OPP_GOAL)
    goalline = core.world_objects.getObjPtr(core.WO_OPP_GOAL_LINE)
    imageWidth = 320.0
    imageHeight = 240.0
    xc = imageWidth/2
    if inDuration(0.0, 2.0, self):
      commands.setStiffness()
      commands.stand()
      commands.setHeadTilt(-21)
    #####START GOALIE WALKING AND STOPPING AT WHITE LINE########
    # elif inDuration(2.0, -1.0, self):
    #   if goalline.seen:
    #     core.speech.say('WHITE')
    #     commands.stand()
    #   else:
    #     commands.setWalkVelocity(0.1, 0.0, 0.0)
    #####END GOALIE WALKING AND STOPPING AT WHITE LINE##########
    elif inDuration(self.blockStart, self.blockStart+10, self):
      self.task.processFrame()
      #print("PROCESS\n")
      if self.task.finished():
        self._complete = True
        commands.stand()
        #print("FINISHED\n")
        self.blockStart = -10000
    # elif inDuration(self.blockStart+5.0, self.blockStart+8.0, self):
    #   commands.stand()
    elif inDuration(2.0, -1.0, self):
      #print("NORMAL\n")
      if(ball.seen):
        
        #commands.setHeadPan(0.0, 1.0, True)
        if(ball.width>0 and ball.fromTopCamera):
          self.searchDirection = 1
        elif ball.fromTopCamera:
          self.searchDirection = -1
        # print("BALL X: " + str(ball.width) + "   BALL Y: " + str(ball.height) + "\n")

        if(ball.width >= 50.0 and ball.width<= 650.0 and ball.height <= 75.0):
          core.speech.say('LEFT')
          # commands.setWalkVelocity(0.0, -5.0, 0.0)
          self.blockStart = self.getTime()
          if(ball.fieldLineIndex == 1):
            self.task = pose.BlockLeft()
          else:
            self.task = pose.Squat()
        elif(ball.width <= -50.0 and ball.width >=-650.0  and ball.height <= 75.0):
          core.speech.say('RIGHT')
          # commands.setWalkVelocity(0.0, 5.0, 0.0)
          self.blockStart = self.getTime()
          if(ball.fieldLineIndex==1):
            self.task = pose.BlockRight()
          else:
            self.task = pose.Squat()
        else:
          rotateX = ball.imageCenterX -xc
          extremeAngle = math.pi/4.0
          moveX = -((ball.imageCenterX * ((extremeAngle*2.0)/imageWidth)) - extremeAngle)
          
          if(math.fabs(moveX) > math.fabs(percepts.joint_angles[core.HeadPan]) and math.fabs(percepts.joint_angles[core.HeadPan]) > 0.4) or (rotateX <= 25 and rotateX >= -25):
            commands.setHeadPan(0, 2.0, True)
            #ASHER WALKING CODE DISABLED CURRENTLY
            # if(percepts.joint_angles[core.HeadPan] > 0.0):
            #   print("WALK RIGHT\n")
            #   commands.setWalkVelocity(0.0, 0.2, 0.0)
            # elif(percepts.joint_angles[core.HeadPan] < -0.0):
            #   print("WALK LEFT\n")
            #   commands.setWalkVelocity(0.0, -0.2, 0.0)
            # else:
            #   print("NOT TOO FAR\n")
            #   commands.stand()
            commands.stand()
          elif (rotateX > 25 or rotateX < -25):
            #print(str(percepts.joint_angles[core.HeadPan]) + " MOVEX: " + str(moveX) + "\n")
            commands.stand()
            commands.setHeadPan(moveX, 2.0)
            #commands.setWalkVelocity(0.0, 0.0, -0.1)
            
        #core.speech.say('YES')
        #print('BALL CENTER:' + str(ball.imageCenterX) + ', ' + str(ball.imageCenterY) + '\n')
      else:
        core.speech.say('SEARCH')
        commands.stand()
        if(self.searchDirection != 0):  
          if(percepts.joint_angles[core.HeadPan] > 0.6 and self.searchDirection==1.0):
            self.searchDirection = -1.0
          elif(percepts.joint_angles[core.HeadPan] < -0.6 and self.searchDirection==-1.0):
            self.searchDirection = 1.0
          commands.setHeadPan((math.pi/4.0)*self.searchDirection, 1.0)
          #print(str(math.fabs(percepts.joint_angles[core.HeadPan])) + "\n")
          #commands.setWalkVelocity(0.0, 0.0, 0.3 * self.searchDirection)
        else:
          pass
          #commands.setWalkVelocity(0.0, 0.0, 0.2 * self.searchDirection)
