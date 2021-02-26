import pybullet as p
import time
import math
import random
import pybullet_data
from datetime import datetime

######################################################### Simulation Setup ############################################################################

clid = p.connect(p.GUI)
if (clid<0):
	p.connect(p.GUI)

p.setGravity(0,0,-9.8)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
planeId = p.loadURDF("plane.urdf", [0, 0, -1])
#p.loadURDF("plane.urdf",[0,0,-.98])
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING,0)
sawyerId = p.loadURDF("./sawyer_robot/sawyer_description/urdf/sawyer.urdf",[0,0,0], [0,0,0,3])  # load sawyer robot
tableId = p.loadURDF("table/table.urdf",[1.4,0, -1], p.getQuaternionFromEuler([0,0,1.56]))   # load table
#objectId = p.loadURDF("cube/marble_cube.urdf",[1.1,0,0], p.getQuaternionFromEuler([0,0,1.56]))  

######################################################### Load Object Here!!!!#############################################################################

# load object, change file name to load different objects
# p.loadURDF(finlename, position([X,Y,Z]), orientation([a,b,c,d])) 
# center of table is at [1.4,0, -1], adjust postion of object to put it on the table                                                             
objectId = p.loadURDF("random_urdfs/001/001.urdf",[1.1,0,0], p.getQuaternionFromEuler([0,0,1.56]))   

###########################################################################################################################################################


p.configureDebugVisualizer(p.COV_ENABLE_RENDERING,1)
p.resetBasePositionAndOrientation(sawyerId,[0,0,0],[0,0,0,1])
#bad, get it from name! sawyerEndEffectorIndex = 18
sawyerEndEffectorIndex = 16
numJoints = p.getNumJoints(sawyerId)   #65 with ar10 hand
#print(p.getJointInfo(sawyerId, 3))
#useRealTimeSimulation = 0
#p.setRealTimeSimulation(useRealTimeSimulation)
#p.stepSimulation()
# all R joints in robot
js = [3, 4, 8, 9, 10, 11, 13, 16, 21, 22, 23, 26, 27, 28, 30, 31, 32, 35, 36 ,37, 39, 40, 41, 44, 45, 46, 48, 49, 50, 53, 54, 55, 58, 61, 64] 
#lower limits for null space
ll = [-3.0503, -5.1477, -3.8183, -3.0514, -3.0514, -2.9842, -2.9842, -4.7104, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.85, 0.34, 0.17]
#upper limits for null space
ul = [3.0503, 0.9559, 2.2824, 3.0514, 3.0514, 2.9842, 2.9842, 4.7104, 1.57, 1.57, 0.17, 1.57, 1.57, 0.17, 1.57, 1.57, 0.17, 1.57, 1.57, 0.17, 1.57, 1.57, 0.17, 1.57, 1.57, 0.17, 1.57, 1.57, 0.17, 1.57, 1.57, 0.17, 2.15, 1.5, 1.5]
#joint ranges for null space
jr = [0, 0, 0, 0, 0, 0, 0, 0, 1.4, 1.4, 1.4, 1.4, 1.4, 0, 1.4, 1.4, 0, 1.4, 1.4, 0, 1.4, 1.4, 0, 1.4, 1.4, 0, 1.4, 1.4, 0, 1.4, 1.4, 0, 1.3, 1.16, 1.33]
#restposes for null space
rp = [0]*35
#joint damping coefficents
jd = [1.1]*35
######################################################### Inverse Kinematics Function ##########################################################################

# Finger tip ID: index:51, mid:42, ring: 33, pinky:24, thumb 62
# Palm ID: 20
# move palm (center point) to reach the target postion and orientation
# input: targetP --> target postion 
#        orientation --> target orientation of the palm
# output: joint positons of all joints in the robot
#         control joint to correspond joint position
def palmP(targetP, orientation): 
	jointP = [0]*65
	jointPoses = p.calculateInverseKinematics(sawyerId, 20, targetP, targetOrientation = orientation, jointDamping=jd)
	j=0
	for i in js:
		jointP[i] = jointPoses[j]
		j=j+1
	
	for i in range(p.getNumJoints(sawyerId)):		
        	p.setJointMotorControl2(bodyIndex=sawyerId, 
					jointIndex=i, 
					controlMode=p.POSITION_CONTROL, 
					targetPosition=jointP[i], 
					targetVelocity=0, 
					force=500, 
					positionGain=0.03, 
					velocityGain=1)
	return jointP

######################################################### Hand Direct Control Functions ##########################################################################

#control the lower joint and middle joint of pinky finger, range both [0.17 - 1.57]
def pinkyF(lower, middle):
	p.setJointMotorControlArray(bodyIndex=sawyerId,
                                jointIndices=[21, 26, 22, 27],
                                controlMode=p.POSITION_CONTROL,
                                targetPositions=[lower, lower, middle, middle],
                                targetVelocities=[0, 0, 0, 0],
                                forces=[500, 500, 500, 500],
                                positionGains=[0.03, 0.03, 0.03, 0.03],
                                velocityGains=[1, 1, 1, 1])	

#control the lower joint and middle joint of ring finger, range both [0.17 - 1.57]
def ringF(lower, middle):
	p.setJointMotorControlArray(bodyIndex=sawyerId,
                                jointIndices=[30, 35, 31, 36],
                                controlMode=p.POSITION_CONTROL,
                                targetPositions=[lower, lower, middle, middle],
                                targetVelocities=[0, 0, 0, 0],
                                forces=[500, 500, 500, 500],
                                positionGains=[0.03, 0.03, 0.03, 0.03],
                                velocityGains=[1, 1, 1, 1])

#control the lower joint and middle joint of mid finger, range both [0.17 - 1.57]
def midF(lower, middle):
	p.setJointMotorControlArray(bodyIndex=sawyerId,
                                jointIndices=[39, 44, 40, 45],
                                controlMode=p.POSITION_CONTROL,
                                targetPositions=[lower, lower, middle, middle],
                                targetVelocities=[0, 0, 0, 0],
                                forces=[500, 500, 500, 500],
                                positionGains=[0.03, 0.03, 0.03, 0.03],
                                velocityGains=[1, 1, 1, 1])

#control the lower joint and middle joint of index finger, range both [0.17 - 1.57]
def indexF(lower, middle):
	p.setJointMotorControlArray(bodyIndex=sawyerId,
                                jointIndices=[48, 53, 49, 54],
                                controlMode=p.POSITION_CONTROL,
                                targetPositions=[lower, lower, middle, middle],
                                targetVelocities=[0, 0, 0, 0],
                                forces=[500, 500, 500, 500],
                                positionGains=[0.03, 0.03, 0.03, 0.03],
                                velocityGains=[1, 1, 1, 1])

#control the lower joint and middle joint of thumb, range: low [0.17 - 1.57], mid [0.34, 1.5]
def thumb(lower, middle):
	p.setJointMotorControlArray(bodyIndex=sawyerId,
                                jointIndices=[58, 61, 64],
                                controlMode=p.POSITION_CONTROL,
                                targetPositions=[lower, middle, middle],
                                targetVelocities=[0, 0, 0],
                                forces=[500, 500, 500],
                                positionGains=[0.03, 0.03, 0.03],
                                velocityGains=[1, 1, 1])

######################################################### Simulation ##########################################################################

currentP = [0]*65
k = 0
while 1:
	k = k + 1

	# move palm to target postion
	i=0
	while 1:
		i+=1
		p.stepSimulation()
		currentP = palmP([1.15, -0.07, 0.25], p.getQuaternionFromEuler([0, -math.pi*1.5, 0]))
		time.sleep(0.03)
		if(i==150):
			break

	i=0
	while 1:
		i+=1
		p.stepSimulation()
		currentP = palmP([1.15, -0.05, -0.15], p.getQuaternionFromEuler([0, -math.pi*1.5, 0]))
		time.sleep(0.03)
		if(i==80):	
			break
	# grasp object and print postions and orientations of finger tips
	# record the postions and orentations for inverse kinematics 
	i=0
	while 1:
		i+=1
		p.stepSimulation()
		time.sleep(0.03)
		thumb(1.5, 1.57)
		if(i==10):				
			indexF(1.4, 2.47)
			midF(1.4, 1.47)
			ringF(1.4, 1.47)
			pinkyF(1.5, 1.57)

		if(i==50):	
			#index:51, mid:42, ring: 33, pinky:24, thumb 62
			print("Thumb Position: ", p.getLinkState(sawyerId, 62)[0]) # get position of thumb tip
			print("Thumb Orientation: ", p.getLinkState(sawyerId, 62)[1]) # get orientation of thumb tip
			print("Index Position: ", p.getLinkState(sawyerId, 51)[0])
			print("Index Orientation: ", p.getLinkState(sawyerId, 51)[1])
			print("Mid: Position: ", p.getLinkState(sawyerId, 42)[0])
			print("Mid: Orientation: ", p.getLinkState(sawyerId, 42)[1])
			print("Ring Position: ", p.getLinkState(sawyerId, 33)[0])
			print("Ring Orientation: ", p.getLinkState(sawyerId, 33)[1])
			print("Pinky Position: ", p.getLinkState(sawyerId, 24)[0])
			print("Pinky Orientation: ", p.getLinkState(sawyerId, 24)[1])
			print("Palm Position: ", p.getLinkState(sawyerId, 20)[0])
			print("Palm Orientation: ", p.getLinkState(sawyerId, 20)[1])
			break 

	# pick up obejct
	i=0
	while 1:
		i+=1
		p.stepSimulation()
		currentP = palmP([1.15, -0.07, 0.1], p.getQuaternionFromEuler([0, -math.pi*1.5, 0]))
		time.sleep(0.03)
		if(i==200):	
			break
	break
p.disconnect()
print("disconnected")



	



