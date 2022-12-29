# Foundational Software layer for Omniverse
import carb

# Omniverse physics API
import omni.physics

# Isaac Speficic extensions API
from omni.isaac.core.utils.prims import get_prim_at_path
from omni.isaac.core.utils.stage import get_current_stage

class QuadrotorRobot:
    """[summary]
    Generic Multirotor Robot Setup. Extract from USD or compile from user input the necessary information for holonomic controller.
    Args:
        name (str): [description]
        prim_path (str): path of the robot articulation
        com_prim_path (str): path of the xform representing the center of mass of the vehicle
    """
        
    def __init__(self, robot_prim_path: str, com_prim_path: str):
        self._robot_prim_path = robot_prim_path
        self.from_usd(self._robot_prim_path, self._com_prim_path)
        
    def from_usd(self, robot_prim_path, com_prim_path):
        """
           if the USD contains all the necessary information, automatically extract them and compile 
        """
        
        # Get the current stage (work environment)
        stage = get_current_stage()
        
        # Get the primit
        robot_prim = get_prim_at_path(robot_prim_path)
        
        if self._com_prim_path == "":
            com_prim = robot_prim  # if no com prim given, assume robot root prim is also com prim
        else:
            com_prim = get_prim_at_path(com_prim_path)

        self._rotational_joints = [j for j in Usd.PrimRange(robot_prim) if j.GetAttribute("isaacmecanumwheel:angle")]
        self._num_rotors = len(self._mecanum_joints)
        self._wheel_dof_names = [j.GetName() for j in self._rotational_joints]
        
        self._wheel_positions = np.zeros((self._num_wheels, 3), dtype=float)  ## xyz for position
        self._wheel_orientations = np.zeros((self._num_wheels, 4), dtype=float)  ## quaternion for orientation
        com_pose = Gf.Matrix4f(omni.usd.utils.get_world_transform_matrix(com_prim))
        
        for i, j in enumerate(self._mecanum_joints):
            joint = UsdPhysics.RevoluteJoint(j)
            chassis_prim = stage.GetPrimAtPath(joint.GetBody0Rel().GetTargets()[0])
            chassis_pose = Gf.Matrix4f(omni.usd.utils.get_world_transform_matrix(chassis_prim))
            p_0 = joint.GetLocalPos0Attr().Get()
            r_0 = joint.GetLocalRot0Attr().Get()
            local_0 = Gf.Matrix4f()
            local_0.SetTranslate(p_0)
            local_0.SetRotateOnly(r_0)
            joint_pose = local_0 * chassis_pose
            self._wheel_positions[i, :] = joint_pose.ExtractTranslation() - com_pose.ExtractTranslation()
            self._wheel_orientations[i, :] = gf_rotation_to_np_array(
                joint_pose.ExtractRotation() * ((com_pose.ExtractRotation()).GetInverse())
            )
        
    @property
    def initialized(self) -> bool:
        """

        Returns:
            bool: True if the view object was initialized (after the first call of .initialize()). False otherwise.
        """
        return self._is_initialized

    def initialize(self, physics_sim_view: omni.physics.tensors.SimulationView = None) -> None:
        """Create a physics simulation view if not passed and creates an articulation view using physX tensor api.

        Args:
            physics_sim_view (omni.physics.tensors.SimulationView, optional): current physics simulation view. Defaults to None.
        """
        if physics_sim_view is None:
            physics_sim_view = omni.physics.tensors.create_simulation_view(self._backend)
            physics_sim_view.set_subspace_roots("/")
            
        carb.log_info("initializing view for {}".format(self._name))
        
        self._physics_sim_view = physics_sim_view
    