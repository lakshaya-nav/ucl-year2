import pybullet as p
import pybullet_data
import time
import math
import numpy as np

# ---------- Base Classes ----------
class SceneObject:
    def __init__(self, urdf_file, position, orientation):
        self.urdf_file = urdf_file
        self.position = position
        self.orientation = p.getQuaternionFromEuler(orientation)
        self.id = None
        self.name = None

    def load(self):
        self.id = p.loadURDF(self.urdf_file, basePosition=self.position, baseOrientation=self.orientation)
        return self.id

    def update_name(self, id):
        self.name = f'{self.__class__.__name__}{id}'


class Box(SceneObject):
    def __init__(self, position, orientation=(0, 0, 0)):
        super().__init__('cube_small.urdf', position, orientation)
        self.grasp_height = 0.03


class Cylinder(SceneObject):
    def __init__(self, position, orientation=(0, 0, 0)):
        super().__init__('cylinder.urdf', position, orientation)
        self.grasp_height = 0.1


# ---------- Gripper Classes ----------
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

        yaw = ((3 / 2) * math.pi) - math.atan2(vector[0], vector[1])
        roll = math.atan2(vector[2], math.sqrt(vector[0] ** 2 + vector[1] ** 2))
        print(roll, yaw)
        self.base_orientation = p.getQuaternionFromEuler((0, roll, yaw))


    def move_towards_obj(self, target_pos, step_size=0.01):
        if self.constraint_id is None:
            raise ValueError("Gripper must be fixed before moving.")
        gripper_pos,_ = p.getBasePositionAndOrientation(self.id)
        while gripper_pos[2] >= target_pos[2] and abs(gripper_pos[1]) >= abs(target_pos[1]) and abs(gripper_pos[0]) >= abs(target_pos[0]):
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

    def move_up(self, z, yaw=0.0):
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

class TwoFingerGripper(Gripper):
    def __init__(self):
        super().__init__('pr2_gripper.urdf', (0, 0, 0), (0, 0, 0))

    def start(self):
        initial_positions = [0.550569, 0.0, 0.549657, 0.0]
        for i, pos in enumerate(initial_positions):
            p.resetJointState(self.id, i, pos)

    def open(self):
        for joint in [0, 2]:
            p.setJointMotorControl2(self.id, joint, p.POSITION_CONTROL,
                                    targetPosition=0.0, maxVelocity=1, force=10)

    def close(self):
        for joint in [0, 2]:
            p.setJointMotorControl2(self.id, joint, p.POSITION_CONTROL,
                                    targetPosition=0.1, maxVelocity=10, force=100)

    def move_grasp_lift(self, obj, target_pos, lift_height=0.4, lift_steps=150):
        yaw_angle = 0.0
        grasp_height = obj.grasp_height

        # --- Set friction on contact surfaces ---
        p.changeDynamics(obj.id, -1, lateralFriction=2.0, rollingFriction=0.1, spinningFriction=0.1)
        p.changeDynamics(self.id, -1, lateralFriction=2.0, rollingFriction=0.1, spinningFriction=0.1)
        self.move_towards_obj(target_pos)
        for _ in range(10):  # allow contact to form
            p.stepSimulation()
            time.sleep(1. / 240.)
        self.close()
        for _ in range(50):  # allow contact to form
            p.stepSimulation()
            time.sleep(1. / 240.)

            # --- Continuous hold while lifting step-by-step ---
        z_current = grasp_height
        z_target = lift_height
        z_step = (z_target - z_current) / lift_steps

        for _ in range(lift_steps):
            z_current += z_step
            self.move_up(z_current, yaw_angle)

            # Continuously reapply strong grip to prevent slip
            for joint in [0, 2]:
                p.setJointMotorControl2(self.id, joint, p.POSITION_CONTROL,
                                            targetPosition=0.12, force=400, maxVelocity=2)

            p.stepSimulation()
            time.sleep(1. / 240.)


def create_points_sphere(radius, n):
    u = np.random.rand(n)
    v = np.random.rand(n)

    theta = 2 * np.pi * u
    phi = np.arccos(v)
    x = []
    y = []
    z = []
    for i in range(n):
        x.append(radius * np.sin(phi[i]) * np.cos(theta[i]))
        y.append(radius * np.sin(phi[i]) * np.sin(theta[i]))
        z.append(radius * np.cos(phi[i]))

    return x,y,z


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
    box = Cylinder((0,0,0))
    box.load()
    obj_position = (0,0,0)
    x,y,z = create_points_sphere(0.3,1000)
    target_pos = (x[0], y[0], z[0])
    print(target_pos)
    scalar = 2
    base_pos = (target_pos[0]*scalar, target_pos[1]*scalar, target_pos[2]*scalar)
    gripper = TwoFingerGripper()
    gripper.base_position = list(base_pos)
    gripper.find_orientation(obj_position)
    gripper.load()
    gripper.start()
    gripper.attach_fixed(offset=[0.2, 0, 0])
    gripper.move_grasp_lift(box, target_pos)

    # Keep GUI open briefly
    for _ in range(1000):
        p.stepSimulation()
        time.sleep(1. / 240.)

    p.disconnect()

