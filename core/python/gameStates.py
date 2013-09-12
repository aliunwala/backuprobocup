#!/usr/bin/env python

import core
from task import Task, HeadBodyTask, MachineTask
import pose, head
import math
import commands, cfgstiff
import testFSM
import percepts


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
  headIsDown = False


  def run(self):
    if inDuration(0.0, 2.0, self):
      commands.setStiffness()
      commands.stand()
      #commands.setHeadTilt(-21)
      #self.headIsDown = True
    else:
      ball = core.world_objects.getObjPtr(core.WO_BALL)
      if ball.seen or True:
        imageWidth = 320.0
        imageHeight = 240.0
        xc = imageWidth/2
        extremeAngle = math.pi
        angleError = -((ball.imageCenterX * ((extremeAngle*2.0)/imageWidth)) - extremeAngle)

        lastError = 0.0
        timeStep = 30
        if len(self.errorSamples)+1 > timeStep :
          lastError = self.errorSamples[timeStep-1]
        elif len(self.errorSamples)>0 :
          lastError = self.errorSamples[len(self.errorSamples)-1]

        #Calculate pidAngle
        pidP = angleError
        pidI = self.sampleSum
        pidD = (angleError - lastError)/timeStep

        kP = 1.0
        kI = 0.0
        kD = -0.45

        pidAngle = kP*pidP + kI*pidI + kD*pidD
        if pidAngle > extremeAngle:
          pidAngle = extremeAngle;
        elif pidAngle < -extremeAngle:
          pidAngle = -extremeAngle;
        print('P=' + str(pidP) + '  I=' + str(pidI) + '  D=' + str(pidD) + '  newAngle=' + str(pidAngle))

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
        inFrontOfBall = True
        if not ball.fromTopCamera and ball.imageCenterY > (imageHeight*.75) :
          walkSpeed = 0.0
          inFrontOfBall = True
          #self.headIsDown = False
          #commands.setHeadTilt()
        ###############################################################################

        #Can we see the goal?#########################################################
        if inFrontOfBall:
          goal = core.world_objects.getObjPtr(core.WO_OPP_GOAL)
          if False:
            #can see the ball
            pass
          else:
            #rotate around the ball until goal is visible
            core.speech.say('ROTATE')
            commands.setWalkVelocity(0.0, 0.45, -math.pi/20)
        else:
          #make robot walk ##############################################################
          if walkSpeed == 0.0 : {commands.stand()}
          else : commands.setWalkVelocity(walkSpeed, 0.0, pidAngle)
          ###############################################################################



        ###############################################################################



        
      else:
          #find the ball
          core.speech.say('SEARCHING')
          commands.setWalkVelocity(0.0, 0.0, 0.3)
          self.sampleSum = 0
          self.errorSamples = []



class Testing(Task):
  def run(self):
    commands.setStiffness()
    commands.setWalkVelocity(.5, .2, 0.0)
    if self.getTime() > 5.0:
      self.subtask = pose.Sit()
      self.finish()
