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

class Ready(HeadBodyTask):
  def __init__(self):
    HeadBodyTask.__init__(self,
      head.Scan(period = 3.0, maxPan = 105.0 * core.DEG_T_RAD, numSweeps = 4),
      pose.Stand()
    )

  def run(self):
    commands.setStiffness()
    HeadBodyTask.run(self)

class Playing(Task):
  sampleSum = 0
  errorSamples = []
  rotateCounter = 0
  kickStart = -1

#Playing = kicks.Kick

  def run(self):
    ball = core.world_objects.getObjPtr(core.WO_BALL)
    goal = core.world_objects.getObjPtr(core.WO_OPP_GOAL)
    imageWidth = 320.0
    imageHeight = 240.0
    xc = imageWidth/2
    ballLinedUpForKick = ball.imageCenterX>= (imageWidth*.25) and ball.imageCenterX<= (imageWidth*.75) and not (ball.imageCenterX<= (imageWidth*.51) and ball.imageCenterX>= (imageWidth*.49))
    canKick  = ball.seen and ball.imageCenterY >(imageHeight*.6) and ballLinedUpForKick

    if inDuration(0.0, 2.0, self):
      commands.setStiffness()
      #commands.stand()
      commands.setHeadTilt(-21)
    elif inDuration(2.0, -1.0, self):
      if(ball.seen):
        if(ball.fieldLineIndex == 1):
          core.speech.say('RIGHT')
        elif(ball.fieldLineIndex == -1):
          core.speech.say('LEFT')
        pass
        #core.speech.say('YES')
        #print('BALL CENTER:' + str(ball.imageCenterX) + ', ' + str(ball.imageCenterY) + '\n')
      else:
        pass
        #core.speech.say('NO')
    elif inDuration(self.kickStart, self.kickStart+2.0, self):
      if canKick:
        core.speech.say('KICK')
      else:
        self.kickStart = -1
    elif inDuration(self.kickStart+2.0, self.kickStart+3.0, self):
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
    elif inDuration(self.kickStart+3.0, self.kickStart+5.0, self):
      kreq = core.kick_request
      kreq.kick_running = False
    else:
      if ball.seen and ball.ballBlobIndex==1:
        core.speech.say('SCORE')
        commands.stand()
        #self.subtask = pose.Sit()
        #self.finish()
      elif ball.seen:
        
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
        walkSpeed = 0.7
        if not ball.fromTopCamera:
          #ball is in bottom camera
          walkSpeed = 0.2
        ###############################################################################

        #Is robot in front of ball?####################################################
        inFrontOfBall = False
        if not ball.fromTopCamera and ball.imageCenterY > (imageHeight*.1) and  (ball.imageCenterX > (imageWidth*.3) and ball.imageCenterX < (imageWidth*.7)):
          walkSpeed = 0.0
          inFrontOfBall = True
        ###############################################################################


        #Can we see the goal?#########################################################
        if inFrontOfBall:
          goalThresh = 40
          if goal.seen and ((goal.imageCenterX-goalThresh) <= ball.imageCenterX and (goal.imageCenterX+goalThresh) >= ball.imageCenterX) :
            if ball.imageCenterY >(imageHeight*.8) and ballLinedUpForKick:
              commands.stand()
              self.kickStart = self.getTime()
            else:
              core.speech.say('WALK')
              commands.setWalkVelocity(0.5, 0.0, 0.0)
          # elif ball.imageCenterY >(imageHeight*.6) :
          #   core.speech.say('BACK')
          #   commands.setWalkVelocity(-0.4, 0.0, 0.0)
          elif goal.seen:
            #can see the ball
            sideStepSpeed = (goal.imageCenterX - xc)
            sideStepAngle = ((goal.imageCenterX * ((extremeAngle*2.0)/imageWidth)) - extremeAngle)
            if sideStepAngle < 0.3 and sideStepAngle > -0.3 :
              sideStepAngle = 0

            if sideStepSpeed < 0.3 and sideStepSpeed >=0:
              sideStepSpeed = 0.3
            elif sideStepSpeed > -0.3 and sideStepSpeed <=0:
              sideStepSpeed = -0.3

            core.speech.say('ALIGN')
            commands.setWalkVelocity(0.0, sideStepSpeed, sideStepAngle)
          else:
            #rotate around the ball until goal is visible
            core.speech.say('ROTATE')
            commands.setWalkVelocity(0.0, 1.0, -0.2)
           
            #commands.stand()
            # secondsToStrafe = 5

            # rotateDir = 1
            # strafeSpeed = .5
            # if(pidAngle < (math.pi/4) or pidAngle > (-math.pi/4)):
            #   self.rotateCounter = 0
            # elif(pidAngle > (math.pi/3) or pidAngle < (-math.pi/3)):
            #   self.rotateCounter = 60

            # if(self.rotateCounter>0):
            #   self.rotateCounter -= 1
            #   commands.setWalkVelocity(0.0, 0.0, -pidAngle)
            # else:
            #   commands.setWalkVelocity(0.0, strafeSpeed * rotateDir, 0.0)
        else:
          #make robot walk ##############################################################
          if walkSpeed == 0.0 : {commands.stand()}
          else : commands.setWalkVelocity(walkSpeed, 0.0, pidAngle)
          ###############################################################################



        ###############################################################################



        
      else:
          #find the ball
          core.speech.say('SEARCH')
          commands.setWalkVelocity(0.0, 0.0, 0.2)
          self.sampleSum = 0
          self.errorSamples = []



class Testing(Task):
  def run(self):
    commands.setStiffness()
    commands.setWalkVelocity(.5, .2, 0.0)
    if self.getTime() > 5.0:
      self.subtask = pose.Sit()
      self.finish()
