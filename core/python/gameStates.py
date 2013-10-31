#!/usr/bin/env python

import core
from task import Task, HeadBodyTask, MachineTask
import pose, head, kicks, state
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
  orientationSamples = []

  def __init__(self, maxPan = 2.0, period = 3.0, numSweeps = 1, direction = 1):
    Task.__init__(self)
    self.maxPan = maxPan
    self.period = period
    self.numSweeps = numSweeps
    self.direction = direction
    self.intDirection = direction
    self.sweepCounter = 0
    self.state = state.SimpleStateMachine(['firstScan', 'nextScans'])
    self.walkEnd = 0.0
    self.safeOrientation = 0.0
    self.safeX = 0.0
    self.safeY = 0.0

  def run(self):
    woself = core.world_objects.getObjPtr(core.robot_state.WO_SELF)
    if inDuration(0.0, 2.0, self): ##START UP
      commands.setStiffness()
      commands.stand()
      commands.setHeadTilt(-21)
      # print('X(' +  str(woself.loc.x) + ') Y(' + str(woself.loc.y) + ') ORIENT(' + str(woself.orientation) + ')\n' )
    elif inDuration(0.0, self.walkEnd, self):
      commands.setHeadPan(0.0, 1.0, False)
      # print('IN DURATION ' + str(self.walkEnd) + ' < ' + str(self.getTime()) + '\n')
      # actualDegrees = (woself.orientation * 180.0) / math.pi
      # print('X(' +  str(woself.loc.x) + ') Y(' + str(woself.loc.y) + ') ORIENT(' + str(woself.orientation) + ') DEGREES(' + str(actualDegrees) + ')\n' )
      # commands.setWalkVelocity(0.3, 0.0, 0.0)

      
      centAngle = math.atan(math.fabs(woself.loc.x)/math.fabs(woself.loc.y))
      print('CENT BEFORE: ' + str(centAngle * (180/math.pi)))
      if(woself.loc.x >= 0 and woself.loc.y >= 0):
        centAngle = -(centAngle + (math.pi/2.0))
      elif(woself.loc.x < 0 and woself.loc.y >= 0):
        centAngle = -centAngle
      elif(woself.loc.x > 0 and woself.loc.y < 0):
        centAngle = centAngle + (math.pi/2.0)
      else:
        centAngle = centAngle


      # print(' CENT ANGLE: ' + str(centAngle*(180.0/math.pi)) + ' TO THETA ' + str(woself.orientation*(180.0/math.pi)) + '\n')
      # angleDiff = centAngle-self.safeOrientation
      angleDiff = centAngle - woself.orientation#self.safeOrientation
      if(angleDiff > math.pi): 
        angleDiff -= 2.0*math.pi
      elif(angleDiff < -math.pi):
        angleDiff += 2.0*math.pi

      walkVelocity = 0.2
      turnVelocity = 0.0
      # print('FABS ' + str(math.fabs(angleDiff-math.pi))  + '   ' + str(math.fabs(angleDiff+math.pi)) + '\n')
      # if(math.fabs(angleDiff-math.pi) < math.pi/8 or math.fabs(angleDiff+math.pi) < math.pi/8):
      #   turnVelocity = 0.0
      #   walkVelocity = -0.5
      if(angleDiff < math.pi/8 and angleDiff > -math.pi/8):
        turnVelocity = 0.0
      elif(angleDiff > 0):
        turnVelocity = -0.2
      else:
        turnVelocity = 0.2

      if( woself.loc.x < 100 and woself.loc.x > -100 and woself.loc.y < 100 and woself.loc.y > -100):
        commands.stand()
      else:
        commands.setWalkVelocity(walkVelocity, 0.0, turnVelocity)

      

    else:
      commands.stand()
      numSteps = self.period / head.DiscreteScan.stepTime + 1
      stepSize = (2 * self.maxPan / numSteps) * 1.05

      st = self.state

      self.orientationSamples.append(woself.orientation)
      if len(self.orientationSamples)+1 >= 30:
        self.orientationSamples.pop()

      if self.sweepCounter > self.numSweeps: 
        self.walkEnd = self.getTime() + 6.0
        self.sweepCounter = 0

        self.safeOrientation = 0
        for i in self.orientationSamples:
          self.safeOrientation += i

        self.safeOrientation = self.safeOrientation / len(self.orientationSamples)

        self.orientationSamples = []
        self.safeX = woself.loc.x
        self.safeY = woself.loc.y
        return

      if st.inState(st.firstScan):
        self.intDirection = self.direction
        self.subtask = head.DiscreteScan(dest = self.direction * self.maxPan, stepSize = stepSize)
        st.transition(st.nextScans)
        return

      if st.inState(st.nextScans) and self.subtask.finished():
        self.sweepCounter += 1
        self.intDirection *= -1
        core.behavior_mem.completeBallSearchTime = core.vision_frame_info.seconds_since_start
        self.subtask = head.DiscreteScan(dest = self.intDirection * self.maxPan, stepSize = stepSize, skipFirstPause = True)
        return

      





class Testing(Task):
  blockStart = -10000
  searchDirection = 0


  def run(self):
    ball = core.world_objects.getObjPtr(core.WO_BALL)
    
