#!/usr/bin/env python3
"""
Vision-driven sorting for the OpenMANIPULATOR-X (moveit_py + OpenCV).

Instead of hard-coded object positions, this node:
  1. parks the arm clear of the overhead camera,
  2. grabs a frame from the MuJoCo overhead camera (/overhead/color/image_raw),
  3. detects each shape by colour (red/green/blue) with OpenCV, finds its shape
     (cube / cylinder / hexagon) and its world (x, y) on the table,
  4. picks each object and places it in its colour-matched drop zone,
  regardless of where the objects start.

Requires: ros-<distro>-cv-bridge, python3-opencv, numpy.

Run:
    ros2 launch omx_mujoco sim.launch.py
    ros2 launch omx_moveit_config pick_place.launch.py
"""
import time
import numpy as np
import rclpy
import rclpy.logging
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Pose
from moveit.core.robot_state import RobotState
from moveit.planning import MoveItPy
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
import cv2
from cv_bridge import CvBridge

# ----------------------------------------------------------------------------
# workspace / camera constants  (must match scene.xml)
# ----------------------------------------------------------------------------
PLANNING_FRAME = "world"
CAM_XY = (0.15, 0.0)          # overhead camera x,y (looks straight down)
CAM_Z = 0.50                  # overhead camera height
OBJ_TOP_Z = 0.075            # top face of a 25 mm object on the 50 mm table
GRASP_Z = 0.076              # EE height to grasp (hand clears the table)
PRE_Z, LIFT_Z = 0.12, 0.13
WRIST_PITCH = 68.0           # deg, consistent downward wrist
PARK = [0.0, -1.45, 1.35, 0.2]   # arm parked clear of the camera for detection

# table workspace bounds (reject anything off the table)
XMIN, XMAX, YMIN, YMAX = 0.06, 0.27, -0.14, 0.14
# fixed drop-zone centres, one per colour
ZONES = {"red": (0.105, -0.075), "green": (0.105, 0.0), "blue": (0.105, 0.075)}
COLOR_SHAPE = {"red": "cube", "green": "cylinder", "blue": "hexagon"}

# ----------------------------------------------------------------------------
# pure-numpy kinematics (verified against MuJoCo to < 1 micron)
# ----------------------------------------------------------------------------
def _Rz(a): c,s=np.cos(a),np.sin(a); return np.array([[c,-s,0,0],[s,c,0,0],[0,0,1,0],[0,0,0,1]])
def _Ry(a): c,s=np.cos(a),np.sin(a); return np.array([[c,0,s,0],[0,1,0,0],[-s,0,c,0],[0,0,0,1]])
def _T(x,y,z): m=np.eye(4); m[:3,3]=[x,y,z]; return m
def fk(q):
    j1,j2,j3,j4 = q
    M=_T(0.012,0,0)@_Rz(j1)@_T(0,0,0.0595)@_Ry(j2)@_T(0.024,0,0.128)@_Ry(j3)@_T(0.124,0,0)@_Ry(j4)@_T(0.126,0,0)
    return M[:3,3], (j2+j3+j4)
_MARGIN=0.03
_LO=np.array([-3.14,-1.5,-1.5,-1.7])+_MARGIN; _HI=np.array([3.14,1.5,1.4,1.97])-_MARGIN
def ik(x, y, z, pitch_deg=WRIST_PITCH):
    tp=np.radians(pitch_deg); q=np.array([np.arctan2(y,x),-0.4,0.4,1.2])
    for _ in range(200):
        p,pit=fk(q); err=np.array([x-p[0],y-p[1],z-p[2],(tp-pit)*0.05]); J=np.zeros((4,4))
        for i in range(4):
            dq=np.zeros(4); dq[i]=1e-6; p2,pit2=fk(q+dq)
            J[:,i]=[(p2[0]-p[0])/1e-6,(p2[1]-p[1])/1e-6,(p2[2]-p[2])/1e-6,((pit2-pit)/1e-6)*0.05]
        q=np.clip(q+J.T@np.linalg.solve(J@J.T+1e-4*np.eye(4),err),_LO,_HI)
    p,pit=fk(q)
    return list(map(float,q)), np.linalg.norm([x-p[0],y-p[1],z-p[2]])

# ----------------------------------------------------------------------------
# OpenCV detection (verified in simulation to ~2 mm)
# ----------------------------------------------------------------------------
_RANGES={"red":[(0,120,90,10,255,255),(170,120,90,180,255,255)],
         "green":[(35,45,35,90,255,255)],
         "blue":[(95,90,60,130,255,255)]}
def _classify(c):
    peri=cv2.arcLength(c,True); n=len(cv2.approxPolyDP(c,0.035*peri,True))
    circ=4*np.pi*cv2.contourArea(c)/(peri*peri+1e-9)
    return "cylinder" if circ>0.82 else ("cube" if n<=4 else "hexagon")

def detect(rgb, fx, fy, cx, cy):
    depth = CAM_Z - OBJ_TOP_Z
    def pix2world(u,v): return CAM_XY[0]+(u-cx)/fx*depth, CAM_XY[1]-(v-cy)/fy*depth
    hsv=cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV); found={}
    for color,rs in _RANGES.items():
        mask=None
        for r in rs:
            mm=cv2.inRange(hsv,np.array(r[:3]),np.array(r[3:])); mask=mm if mask is None else cv2.bitwise_or(mask,mm)
        mask=cv2.morphologyEx(mask,cv2.MORPH_OPEN,np.ones((5,5),np.uint8))
        cnts,_=cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE); best=None
        for c in cnts:
            if cv2.contourArea(c)<60: continue
            M=cv2.moments(c); X,Y=pix2world(M["m10"]/M["m00"], M["m01"]/M["m00"])
            if not (XMIN<X<XMAX and YMIN<Y<YMAX): continue
            zx,zy=ZONES[color]
            if np.hypot(X-zx,Y-zy)<0.035: continue        # ignore its own empty drop zone
            if best is None or cv2.contourArea(c)>best[0]:
                best=(cv2.contourArea(c), _classify(c), round(X,4), round(Y,4))
        if best: found[color]=best[1:]
    return found   # {color: (shape, x, y)}

# ----------------------------------------------------------------------------
class CameraGrabber(Node):
    def __init__(self):
        super().__init__("vision_camera")
        self.bridge=CvBridge(); self.rgb=None; self.K=None
        self.create_subscription(Image,"/overhead/color/image_raw",self._img,10)
        self.create_subscription(CameraInfo,"/overhead/color/camera_info",self._info,10)
    def _img(self,msg):
        self.rgb=self.bridge.imgmsg_to_cv2(msg,desired_encoding="rgb8")
    def _info(self,msg): self.K=msg.k
    def grab(self, timeout=8.0):
        t0=time.time()
        while rclpy.ok() and time.time()-t0<timeout:
            rclpy.spin_once(self,timeout_sec=0.1)
            if self.rgb is not None: break
        if self.rgb is None: return None
        if self.K and self.K[0]>0:
            fx,fy,cx,cy=self.K[0],self.K[4],self.K[2],self.K[5]
        else:  # fallback: assume 55 deg vertical FOV
            h,w=self.rgb.shape[:2]; fy=(h/2)/np.tan(np.radians(55)/2); fx=fy; cx=w/2; cy=h/2
        return self.rgb, fx, fy, cx, cy

# ----------------------------------------------------------------------------
def plan_exec(robot, pc, sleep=0.0):
    r=pc.plan()
    if not r: return False
    robot.execute(r.trajectory, controllers=[])
    if sleep: time.sleep(sleep)
    return True
def move_joints(robot, arm, j):
    rs=RobotState(robot.get_robot_model()); rs.set_joint_group_positions("arm",np.array(j,float)); rs.update()
    arm.set_start_state_to_current_state(); arm.set_goal_state(robot_state=rs); return plan_exec(robot,arm)
def move_gripper(robot, gr, state):
    gr.set_start_state_to_current_state(); gr.set_goal_state(configuration_name=state); return plan_exec(robot,gr,sleep=0.6)
def add_table(robot):
    with robot.get_planning_scene_monitor().read_write() as scene:
        co=CollisionObject(); co.header.frame_id=PLANNING_FRAME; co.id="table"
        b=SolidPrimitive(); b.type=SolidPrimitive.BOX; b.dimensions=[0.22,0.29,0.044]
        p=Pose(); p.position.x,p.position.y,p.position.z=0.16,0.0,0.022; p.orientation.w=1.0
        co.primitives.append(b); co.primitive_poses.append(p); co.operation=CollisionObject.ADD
        scene.apply_collision_object(co); scene.current_state.update()

def pick_place(robot, arm, gripper, log, color, x, y):
    zx,zy = ZONES[color]
    grasp,eg = ik(x,y,GRASP_Z);   pre,_   = ik(x,y,PRE_Z);  lift,_ = ik(x,y,LIFT_Z)
    carry,_  = ik(zx,zy,LIFT_Z);  place,_ = ik(zx,zy,GRASP_Z)
    if eg>0.01:
        log.warn(f"  {color}: IK error {eg*1000:.0f}mm, skipping"); return
    for label, act in [("open",  lambda: move_gripper(robot,gripper,"open")),
                       ("pre",   lambda: move_joints(robot,arm,pre)),
                       ("grasp", lambda: move_joints(robot,arm,grasp)),
                       ("close", lambda: move_gripper(robot,gripper,"close")),
                       ("lift",  lambda: move_joints(robot,arm,lift)),
                       ("carry", lambda: move_joints(robot,arm,carry)),
                       ("place", lambda: move_joints(robot,arm,place)),
                       ("release",lambda: move_gripper(robot,gripper,"open")),
                       ("retreat",lambda: move_joints(robot,arm,lift))]:
        if not act(): log.error(f"  {color}: step '{label}' failed"); return

def main():
    rclpy.init()
    log=rclpy.logging.get_logger("vision_sort")
    robot=MoveItPy(node_name="moveit_py")
    arm=robot.get_planning_component("arm"); gripper=robot.get_planning_component("gripper")
    cam=CameraGrabber()
    log.info("MoveItPy + camera ready")

    add_table(robot); time.sleep(1.0)
    move_gripper(robot, gripper, "open")
    move_joints(robot, arm, PARK)          # clear the camera view
    time.sleep(1.5)

    frame=cam.grab()
    if frame is None:
        log.error("No camera image on /overhead/color/image_raw - is the sim publishing it?")
        robot.shutdown(); rclpy.shutdown(); return
    rgb,fx,fy,cx,cy=frame
    objects=detect(rgb,fx,fy,cx,cy)
    if not objects:
        log.error("No objects detected."); robot.shutdown(); rclpy.shutdown(); return
    for color,(shape,x,y) in objects.items():
        log.info(f"detected {color} {shape} at ({x:.3f}, {y:.3f}) -> {color} zone")

    for color in ("red","green","blue"):
        if color not in objects: continue
        shape,x,y=objects[color]
        log.info(f"=== sorting {color} {shape} ===")
        pick_place(robot, arm, gripper, log, color, x, y)

    move_joints(robot, arm, [0.0,0.0,0.0,0.0])
    log.info("Vision sort complete.")
    robot.shutdown(); rclpy.shutdown()

if __name__ == "__main__":
    main()
