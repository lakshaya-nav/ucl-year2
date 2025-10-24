import pybullet as p
import pybullet_data
import time
import math

class Gripper:
    def __init__(self, urdf_path, base_position, base_orientation):
        self.urdf_path = urdf_path
        self.base_position = list(base_position)
        self.base_orientation = p.getQuaternionFromEuler(base_orientation)
        self.id = None
        self.constraint_id = None
        self.grasp_moving = False

    def load(self):
        self.id = p.loadURDF(self.urdf_path, basePosition = self.base_position, baseOrientation = self.base_orientation)
        return self.id


    def attach_fixed(self, offset):
        self.constraint_id = p.createConstraint(
            parentBodyUniqueId=self.id,
            parentLinkIndex=-1,
            childBodyUniqueId=-1,
            childLinkIndex=-1,
            jointType=p.JOINT_FIXED,
            jointAxis=[0, 0, 0],
            parentFramePosition=offset,
            childFramePosition=self.base_position,
            childFrameOrientation=self.base_orientation
        )

    def find_orientation(self, obj_position):
        vector = (self.base_position[0] - obj_position[0],
                  self.base_position[1] - obj_position[1],
                  self.base_position[2] - obj_position[2])
        length = math.sqrt(sum(v ** 2 for v in vector))
        self.direction = [v / length for v in vector]

        yaw = ((3/2) * math.pi) - math.atan2(vector[0], vector[1])
        roll = math.atan2(vector[2], math.sqrt(vector[0]**2 + vector[1]**2))
        print(roll, yaw)
        self.base_orientation = p.getQuaternionFromEuler((0, roll, yaw))


    def move_towards_obj(self, target_pos, step_size=0.01):
        if self.constraint_id is None:
            raise ValueError("Gripper must be fixed before moving.")
        gripper_pos,_ = p.getBasePositionAndOrientation(self.id)
        while gripper_pos[2] >= target_pos[2]:
            # Move along direction vector
            self.base_position[0] -= self.direction[0] * step_size
            self.base_position[1] -= self.direction[1] * step_size
            self.base_position[2] -= self.direction[2] * step_size

            p.changeConstraint(
                self.constraint_id,
                jointChildPivot=self.base_position,
                jointChildFrameOrientation=self.base_orientation,
                maxForce=50
            )

            p.stepSimulation()
            time.sleep(1. / 240.)
            gripper_pos, _ = p.getBasePositionAndOrientation(self.id)
            print(gripper_pos)

    def move(self, z, yaw=0.0):
        if self.constraint_id is None:
            raise ValueError("Gripper must be fixed before moving.")
        p.changeConstraint(
            self.constraint_id,
            jointChildPivot=[self.base_position[0], self.base_position[1], z],
            jointChildFrameOrientation=self.base_orientation,
            maxForce=50
        )

    def update_camera(self, z, yaw):
        p.resetDebugVisualizerCamera(
            cameraDistance=0.5,
            cameraYaw=50 + (yaw * 180 / 3.1416),
            cameraPitch=-60,
            cameraTargetPosition=[self.base_position[0], self.base_position[1], z]
        )


# ---------- Environment Setup ----------
def setup_environment():
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.resetSimulation()
    p.setGravity(0, 0, -10)
    p.setRealTimeSimulation(0)
    plane_id = p.loadURDF("plane.urdf")
    return plane_id


# ---------- Main Simulation ----------
if __name__ == "__main__":
    setup_environment()

    # Create and set up the gripper
    base_position = (-0.5,-1,1)
    obj_position = (0,0,0)
    target_pos = (-0.15, -0.3, 0.3)
    gripper = Gripper('pr2_gripper.urdf', base_position, (0, 0, 0))
    gripper.find_orientation(obj_position)
    gripper.load()
    gripper.attach_fixed(offset=[0.2, 0, 0])
    gripper.move_towards_obj(target_pos)

    # Keep GUI open briefly
    for _ in range(10000):
        p.stepSimulation()
        time.sleep(1. / 240.)

    p.disconnect()

