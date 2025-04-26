import pybullet as p
import pybullet_data
import time
import numpy as np
import matplotlib.pyplot as plt

p.connect(p.GUI)
p.createCollisionShape(p.GEOM_PLANE)
p.createMultiBody(0,0)

sh_body = p.createCollisionShape(p.GEOM_BOX,halfExtents=[0.45, 0.08, 0.02] )
sh_extraweight = p.createCollisionShape(p.GEOM_BOX,halfExtents=[0.45, 0.12, 0.025])
sh_roll = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.02, 0.02, 0.02])
sh_hip = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.02, 0.02, 0.16])  
sh_knee = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.02, 0.02, 0.16]) 
sh_foot = p.createCollisionShape(p.GEOM_SPHERE, radius=0.05)

visual_body = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.45, 0.08, 0.02], rgbaColor=[1, 0, 0, 1])  # Red body
visual_shapes = [    
    p.createVisualShape(p.GEOM_BOX, halfExtents=[0.02, 0.02, 0.02], rgbaColor=[0, 0, 0, 1]),  
    p.createVisualShape(p.GEOM_BOX, halfExtents=[0.02, 0.02, 0.16], rgbaColor=[0, 0, 1, 1]), 
    p.createVisualShape(p.GEOM_BOX, halfExtents=[0.02, 0.02, 0.16], rgbaColor=[0, 1, 0, 1]),  
    p.createVisualShape(p.GEOM_SPHERE, radius=0.05, rgbaColor=[1, 1, 0, 1]), 
    
    p.createVisualShape(p.GEOM_BOX, halfExtents=[0.02, 0.02, 0.02], rgbaColor=[0, 0, 0, 1]),  
    p.createVisualShape(p.GEOM_BOX, halfExtents=[0.02, 0.02, 0.16], rgbaColor=[0, 0, 1, 1]), 
    p.createVisualShape(p.GEOM_BOX, halfExtents=[0.02, 0.02, 0.16], rgbaColor=[0, 1, 0, 1]), 
    p.createVisualShape(p.GEOM_SPHERE, radius=0.05, rgbaColor=[1, 1, 0, 1]),  

    p.createVisualShape(p.GEOM_BOX, halfExtents=[0.02, 0.02, 0.02], rgbaColor=[0, 0, 0, 1]),  
    p.createVisualShape(p.GEOM_BOX, halfExtents=[0.02, 0.02, 0.16], rgbaColor=[0, 0, 1, 1]),  
    p.createVisualShape(p.GEOM_BOX, halfExtents=[0.02, 0.02, 0.16], rgbaColor=[0, 1, 0, 1]),  
    p.createVisualShape(p.GEOM_SPHERE, radius=0.05, rgbaColor=[1, 1, 0, 1]), 

    p.createVisualShape(p.GEOM_BOX, halfExtents=[0.02, 0.02, 0.02], rgbaColor=[0, 0, 0, 1]),  
    p.createVisualShape(p.GEOM_BOX, halfExtents=[0.02, 0.02, 0.16], rgbaColor=[0, 0, 1, 1]),  
    p.createVisualShape(p.GEOM_BOX, halfExtents=[0.02, 0.02, 0.16], rgbaColor=[0, 1, 0, 1]),  
    p.createVisualShape(p.GEOM_SPHERE, radius=0.05, rgbaColor=[1, 1, 0, 1]),  

    p.createVisualShape(p.GEOM_BOX, halfExtents=[0.45, 0.12, 0.025], rgbaColor=[0, 0, 0, 0])  
]
body_Mass = 1
visualShapeId = visual_body
link_Masses=[.03, .1, .1, .1, .03, .1, .1, .1, .03, .1, .1, .1, .03, .1, .1, .1, 30]
linkCollisionShapeIndices=[sh_roll, sh_hip, sh_knee, sh_foot, sh_roll, sh_hip, sh_knee, sh_foot, sh_roll, 
                           sh_hip, sh_knee, sh_foot, sh_roll, sh_hip, sh_knee, sh_foot, sh_extraweight]
nlnk=len(link_Masses)
linkVisualShapeIndices = visual_shapes
xhipf=0.4
xhipb=-0.4
yhipl=0.12
xoffh = -0.1 
yoffh = 0.0   
zoffh = -0.1  
hu = 0.3  
hl = 0.3  
linkPositions = [
    [xhipf, yhipl, 0], [xoffh, yoffh, zoffh], [0.15, 0, -hu], [0, 0, -hl],  
    [xhipf, -yhipl, 0], [xoffh, -yoffh, zoffh], [0.15, 0, -hu], [0, 0, -hl],  
    [xhipb, yhipl, 0], [xoffh, yoffh, zoffh], [0.15, 0, -hu], [0, 0, -hl],  
    [xhipb, -yhipl, 0], [xoffh, -yoffh, zoffh], [0.15, 0, -hu], [0, 0, -hl],  
    [0, 0, 0.029]  
]
linkOrientations=[[0,0,0,1]]*nlnk
linkInertialFramePositions=[[0,0,0]]*nlnk
linkInertialFrameOrientations=[[0,0,0,1]]*nlnk
indices=[0, 1, 2, 3, 0, 5, 6, 7, 0, 9,10 ,11, 0,13,14,15, 0]
jointTypes=[p.JOINT_REVOLUTE, p.JOINT_REVOLUTE, p.JOINT_REVOLUTE, p.JOINT_PRISMATIC, p.JOINT_REVOLUTE, 
            p.JOINT_REVOLUTE, p.JOINT_REVOLUTE, p.JOINT_PRISMATIC, p.JOINT_REVOLUTE, p.JOINT_REVOLUTE, 
            p.JOINT_REVOLUTE, p.JOINT_PRISMATIC, p.JOINT_REVOLUTE, p.JOINT_REVOLUTE, p.JOINT_REVOLUTE,
            p.JOINT_PRISMATIC, p.JOINT_PRISMATIC]
axis=[[1,0,0], [0,1,0], [0,1,0], [0,0,1], [1,0,0], [0,1,0], [0,1,0], [0,0,1], [1,0,0], [0,1,0], [0,1,0], 
      [0,0,1], [1,0,0], [0,1,0], [0,1,0], [0,0,1], [0,0,1]]
basePosition = [0,0,1]
baseOrientation = [0,0,0,1]
dog = p.createMultiBody(body_Mass,sh_body,visual_body,basePosition,baseOrientation,
                        linkMasses=link_Masses,
                        linkCollisionShapeIndices=linkCollisionShapeIndices,
                        linkVisualShapeIndices=linkVisualShapeIndices,
                        linkPositions=linkPositions,
                        linkOrientations=linkOrientations,
                        linkInertialFramePositions=linkInertialFramePositions,
                        linkInertialFrameOrientations=linkInertialFrameOrientations,
                        linkParentIndices=indices,
                        linkJointTypes=jointTypes,
                        linkJointAxis=axis)			

joint=16
p.setJointMotorControl2(dog,joint,p.POSITION_CONTROL,targetPosition=0.01,force=100,maxVelocity=5)

joint=3
p.setJointMotorControl2(dog,joint,p.POSITION_CONTROL,targetPosition=0.0,force=100,maxVelocity=5)
joint=7
p.setJointMotorControl2(dog,joint,p.POSITION_CONTROL,targetPosition=0.0,force=100,maxVelocity=5)
joint=11
p.setJointMotorControl2(dog,joint,p.POSITION_CONTROL,targetPosition=0.0,force=100,maxVelocity=5)
joint=15
p.setJointMotorControl2(dog,joint,p.POSITION_CONTROL,targetPosition=0.0,force=100,maxVelocity=5)

p.setGravity(0,0,-9.81)
p.setRealTimeSimulation(1)
p.resetDebugVisualizerCamera( cameraDistance=1.5, cameraYaw=-30, cameraPitch=-30, cameraTargetPosition=[0.0, 0.0, 0.25])
if (0):
    t0=time.time()
    t=time.time()
    while ((t-t0)<10):
        t=time.time()
    p.disconnect()
boxHalfLength = 2.5
boxHalfWidth = 2.5
boxHalfHeight = 0.2
sh_colBox = p.createCollisionShape(p.GEOM_BOX,halfExtents=[boxHalfLength,boxHalfWidth,boxHalfHeight])
mass = 1
block=p.createMultiBody(baseMass=0,baseCollisionShapeIndex = sh_colBox,
                        basePosition = [-2,0,-0.1],baseOrientation=[0.0,0.1,0.0,1])
p.changeDynamics(dog,3,lateralFriction=2)
p.changeDynamics(dog,7,lateralFriction=2)
p.changeDynamics(dog,11,lateralFriction=2)
p.changeDynamics(dog,15,lateralFriction=2)
def xyztoang(x, y, z, yoffh, hu, hl):
    dyz = np.sqrt(y ** 2 + z ** 2)
    lyz = np.sqrt(dyz ** 2 - yoffh ** 2)
    gamma_yz = -np.arctan(y / z)
    gamma_h_offset = -np.arctan(-yoffh / lyz)
    gamma = gamma_yz - gamma_h_offset

    lxzp = np.sqrt(lyz ** 2 + x ** 2)
    n = (lxzp ** 2 - hl ** 2 - hu ** 2) / (2 * hu)
    beta = -np.arccos(n / hl)

    alfa_xzp = -np.arctan(x / lyz)
    alfa_off = np.arccos((hu + n) / lxzp)
    alfa = alfa_xzp + alfa_off

    if any(np.isnan([gamma, alfa, beta])):
        print("Invalid Angles: ", x, y, z, yoffh, hu, hl)

    return [gamma, alfa, beta]

def setlegsxyz(xvec,yvec,zvec,vvec):
    
    a=xyztoang(xvec[0]-xhipf,yvec[0]-yhipl,zvec[0],yoffh,hu,hl)  
    spd=1
    joint=0
    p.setJointMotorControl2(dog,joint,p.POSITION_CONTROL,targetPosition=a[0],force=100,maxVelocity=spd)
    joint=1
    p.setJointMotorControl2(dog,joint,p.POSITION_CONTROL,targetPosition=a[1],force=100,maxVelocity=vvec[0])
    joint=2
    p.setJointMotorControl2(dog,joint,p.POSITION_CONTROL,targetPosition=a[2],force=100,maxVelocity=vvec[0])

    a=xyztoang(xvec[1]-xhipf,yvec[1]+yhipl,zvec[1],-yoffh,hu,hl)  
    spd=1.0
    joint=4
    p.setJointMotorControl2(dog,joint,p.POSITION_CONTROL,targetPosition=a[0],force=100,maxVelocity=spd)
    joint=5
    p.setJointMotorControl2(dog,joint,p.POSITION_CONTROL,targetPosition=a[1],force=100,maxVelocity=vvec[1])
    joint=6
    p.setJointMotorControl2(dog,joint,p.POSITION_CONTROL,targetPosition=a[2],force=100,maxVelocity=vvec[1])

    a=xyztoang(xvec[2]-xhipb,yvec[2]-yhipl,zvec[2],yoffh,hu,hl)  
    spd=1.0
    joint=8
    p.setJointMotorControl2(dog,joint,p.POSITION_CONTROL,targetPosition=a[0],force=100,maxVelocity=spd)
    joint=9
    p.setJointMotorControl2(dog,joint,p.POSITION_CONTROL,targetPosition=a[1],force=100,maxVelocity=vvec[2])
    joint=10
    p.setJointMotorControl2(dog,joint,p.POSITION_CONTROL,targetPosition=a[2],force=100,maxVelocity=vvec[2])

    a=xyztoang(xvec[3]-xhipb,yvec[3]+yhipl,zvec[3],-yoffh,hu,hl)  
    spd=1.0
    joint=12
    p.setJointMotorControl2(dog,joint,p.POSITION_CONTROL,targetPosition=a[0],force=100,maxVelocity=spd)
    joint=13
    p.setJointMotorControl2(dog,joint,p.POSITION_CONTROL,targetPosition=a[1],force=100,maxVelocity=vvec[3])
    joint=14
    p.setJointMotorControl2(dog,joint,p.POSITION_CONTROL,targetPosition=a[2],force=100,maxVelocity=vvec[3])

setlegsxyz([xhipf,xhipf,xhipb,xhipb],[yhipl+0.1,-yhipl-0.1,yhipl+0.1,-yhipl-0.1],[-0.5,-0.5,-0.5,-0.5],[1,1,1,1])
t0=time.time()
t=time.time()
while ((t-t0)<2):
    t=time.time()
def RotYawr(yawr):
    Rhor=np.array([[np.cos(yawr),-np.sin(yawr),0], [np.sin(yawr),np.cos(yawr),0], [0,0,1]])
    return Rhor
yawri=1.3
xrOi=np.array([0,0,0.5])
legsRi=np.array([[xhipf,xhipf,xhipb,xhipb],
               [yhipl+0.1,-yhipl-0.1,yhipl+0.1,-yhipl-0.1],
               [-0.5,-0.5,-0.5,-0.5]])
xbOi=xrOi
quat=p.getQuaternionFromEuler([0,0,yawri])
p.resetBasePositionAndOrientation(dog,xbOi,quat)
Ryawri=RotYawr(yawri)
legsO=(np.dot(Ryawri,legsRi).T + xbOi).T   
yawr=yawri
xrO=xrOi
xbO=xrO
Ryawr=RotYawr(yawri)
dlegsO=(legsO.T-xbO).T
dlegsR=np.dot(Ryawr.T,dlegsO)
setlegsxyz(dlegsR[0],dlegsR[1],dlegsR[2],[1,1,1,1])
xfO=(legsO[:,0]+legsO[:,1])/2.0
xbO=(legsO[:,2]+legsO[:,3])/2.0
xrOn=(xfO+xbO)/2.0 + np.array([0,0,0.5])
xfmbO=xfO-xbO
yawrn=np.arctan2(xfmbO[1],xfmbO[0]) 
cyaw=10
cpitch=-15
cdist=1.5
walkLoopSpd=400
vvec=[12]*4
l=0
xrcO=xrO
xoff=0
yoff=0
dr=0
drp=0
lseq=[0,1,3,2]
lseqp=[0,1,3,2]
com_x = []
com_y = []
com_z = []
timestamps = []
t0 = time.time()
while (time.time() - t0 < 60):
    cubePos, cubeOrn = p.getBasePositionAndOrientation(dog)
    p.resetDebugVisualizerCamera( cameraDistance=cdist, cameraYaw=cyaw, cameraPitch=cpitch, cameraTargetPosition=cubePos)
    keys = p.getKeyboardEvents()
    
    if keys.get(100):  #D
        cyaw+=1
    if keys.get(97):   #A
        cyaw-=1
    if keys.get(99):   #C
        cpitch+=1
    if keys.get(98):  #B
        cpitch-=1
    if keys.get(122):  #Z
        cdist+=.01
    if keys.get(120):  #X
        cdist-=.01
    if keys.get(65297): #Up
        drp=0
    if keys.get(65298): #Down
        drp=2
    if keys.get(65296): #Right
        drp=1
        xrcO=xrO        
        lseqp=[1,0,2,3] 
    if keys.get(65295): #Left
        drp=3
        xrcO=xrO
        lseqp=[0,1,3,2] 
    
    tv=int(((time.time()-t0)*walkLoopSpd)  % 800)
    if tv<20 and (not dr==drp):
        dr=drp
        lseq=lseqp
    l=int(tv/200)
    k=lseq[l]
    if int(tv%200)<10:
        xoff=0
        yoff=0
    elif int(tv%200)<80:
        xoff+=0.002*(-1+2*int(k/2))  
        yoff+=0.002*(-1+2*(k%2))     
    elif int(tv%200)>160:
        xoff-=0.004*(-1+2*int(k/2))
        yoff-=0.004*(-1+2*(k%2))     
 
    dlegsO=(legsO.T-xrO).T  
    dlegsR=np.dot(Ryawr.T,dlegsO)  
    setlegsxyz(dlegsR[0]-xoff-0.03,dlegsR[1]-yoff,dlegsR[2],vvec)  
    
    if int(tv%200)>80:
        dlegsO=(legsO.T-xrcO).T
        yawlO=np.arctan2(dlegsO[1,k],dlegsO[0,k])
        rlO=np.sqrt(dlegsO[0,k]**2+dlegsO[1,k]**2)
        
        if dr==0:
            legsO[0,k]=rlO*np.cos(yawlO)+xrcO[0]+0.01*np.cos(yawr)
            legsO[1,k]=rlO*np.sin(yawlO)+xrcO[1]+0.01*np.sin(yawr)
        elif dr==1:
            yawlO-=0.015 
            legsO[0,k]=rlO*np.cos(yawlO)+xrcO[0]
            legsO[1,k]=rlO*np.sin(yawlO)+xrcO[1]
        elif dr==2:
            legsO[0,k]=rlO*np.cos(yawlO)+xrcO[0]-0.01*np.cos(yawr)
            legsO[1,k]=rlO*np.sin(yawlO)+xrcO[1]-0.01*np.sin(yawr)
        elif dr==3:
            yawlO+=0.015 
            legsO[0,k]=rlO*np.cos(yawlO)+xrcO[0]
            legsO[1,k]=rlO*np.sin(yawlO)+xrcO[1]
        
        if int(tv%200)<150:         
            legsO[2,k]+=.006
        else:         
            legsO[2,k]-=.006
    else:
        legsO[2,0]=0.0
        legsO[2,1]=0.0
        legsO[2,2]=0.0
        legsO[2,3]=0.0
        
    xfrO=(legsO[:,0]+legsO[:,1])/2.0
    xbkO=(legsO[:,2]+legsO[:,3])/2.0
    xrO=(xfrO+xbkO)/2.0 
    xrO[2]=0.5
    xfmbO=xfrO-xbkO
    yawr=np.arctan2(xfmbO[1],xfmbO[0])
    Ryawr=RotYawr(yawr)
    time.sleep(0.01)  
    
    pos, _ = p.getBasePositionAndOrientation(dog)
    com = np.array(pos)
    com_x.append(com[0])
    com_y.append(com[1])
    com_z.append(com[2])
    timestamps.append(time.time() - t0) 
    
if(time.time() - t0 >= 60):
    plt.figure(figsize=(10, 5))
    plt.plot(timestamps, com_x, label='COM X')
    plt.plot(timestamps, com_y, label='COM Y')
    plt.plot(timestamps, com_z, label='COM Z')
    plt.xlabel('Time (s)')
    plt.ylabel('Center of Mass Position (m)')
    plt.title('Robot Center of Mass Trajectory')
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.show()

for step in range(1000):  
    total_mass = 0
    weighted_pos_sum = [0, 0, 0]
    num_joints = p.getNumJoints(dog)
    mass, _, _, _, _, _ = p.getDynamicsInfo(dog, -1)
    pos, _ = p.getBasePositionAndOrientation(dog)
    total_mass += mass
    weighted_pos_sum = [weighted_pos_sum[i] + mass * pos[i] for i in range(3)]    
p.disconnect()






    
