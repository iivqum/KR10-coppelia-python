import threading
from time import sleep

import coppeliasim.bridge

class generic_ik_thread(threading.Thread):
    def __init__(self, is_model = True, model_name = ""):
        super().__init__()
        ik = generic_ik(is_model, model_name)
        self.is_model = is_model
        
        self.terminate = False
        self.setDaemon(True)
        self.start()
        print("IK thread started")
       
    def kill(self):
        self.terminate = True
    
    def get_ik(self):
        return self._kwargs["ik_object"]
    
    def run(self):
        # Thread
        ik = self._kwargs["ik_object"]
        client = RemoteAPIClient()
        self.sim = client.require('self.sim')
        ik = generic_ik(self.is_model, self.model_name)
        
        try:
            while not self.terminate:
                self.sim.step()
        except Exception as e:
            print(f"Thread failure\n{e}")
            raise
        
        print("IK Thread terminated successfully")

class generic_ik:
    # Generic arm controller class that performs IK and positioning of the arm
    
    def __init__(self, sim, sim_ik, is_model = True, model_name = ""):
        """
        * A dummy object must be connected to the end effector called "tip".
        * A dummy object called "target" must be in the model hierarchy
        * Object must not be dynamic.
        
        If is_model is False, the object will be searched for in the scene,
        otherwise it is loaded from the models/ directory.
        """
        
        # TODO find a way of doing this without doing this, perhaps using global variables
        self.sim = sim
        self.simik = sim_ik
        
        print(f"Attempting to load {str(model_name)}")
        
        if not isinstance(model_name, str):
            raise TypeError("Model name not string")
        
        self.joints = list()
        
        try:
            if is_model:
                self.handle = self.sim.loadModel(f"models/{model_name}.ttm")
            else:
                self.handle = self.sim.getObject(f"/{model_name}", {"noError" : False})
        except Exception as e:
            print(f"Something went wrong loading the model\n{e}")
            raise
            
        # No model found
        if not hasattr(self, "handle"):
            raise Exception("Tried to load '", model_name, 
                "\nModel not found in scene or file.")
        # Path to the scene object
        self.root_path = f"/{model_name}/"

        self.setup_required()
        self.setup_ik(1, 99)
        
        print(f"Successfully loaded {str(model_name)}")
    
    def get_tip(self):
        return self.tip_handle
 
    def get_target(self):
        return self.target_handle
        
    def get_handle(self):
        return self.handle
    
    def is_valid(self):
        return self.handle > 0 and self.target_handle > 0 and self.tip_handle and len(self.joints) > 0
        
    def get_child_object(self, name = ""):
        try:
            for child in self.sim.getObjectsInTree(self.handle):
                alias = self.sim.getStringProperty(child, "alias", {"noError" : False}).lower()
                if alias == name:
                    return child
        except:
            raise
        return None
        
    def get_joint_handle(self, joint):
        return self.joints[joint]
    
    def setup_required(self):
        # TODO dynamic joint warning
        # Add joints
        for joint_handle in self.sim.getObjectsInTree(self.handle, 
            self.sim.sceneobject_joint):
            self.joints.append(joint_handle)
        # Find tip and target
        tip = self.get_child_object("tip")
        target = self.get_child_object("target")
        
        if not tip:
            raise Exception("Couldn't find tip dummy")
        if not target:
            raise Exception("Couldn't find target dummy")
            
        self.tip_handle = tip
        self.target_handle = target
    
    def setup_ik(self, damping_factor, max_iterations):
        try:
            self.ik_environment = self.simik.createEnvironment()
            self.ik_group_undamped = self.simik.createGroup(self.ik_environment)
            self.ik_group_undamped_constrained = self.simik.createGroup(self.ik_environment)
            self.ik_group_damped = self.simik.createGroup(self.ik_environment)
             
            self.simik.setGroupCalculation(self.ik_environment, self.ik_group_undamped, 
                self.simik.method_pseudo_inverse, 0, max_iterations)
            self.simik.setGroupCalculation(self.ik_environment, self.ik_group_undamped_constrained, 
                self.simik.method_pseudo_inverse, 0, max_iterations)
            self.simik.addElementFromScene(self.ik_environment, self.ik_group_undamped, self.handle, 
                self.tip_handle, self.target_handle, self.simik.constraint_pose)
            self.simik.addElementFromScene(self.ik_environment, self.ik_group_undamped_constrained, self.handle, 
                self.tip_handle, self.target_handle, self.simik.constraint_position)
                
            self.simik.setGroupCalculation(self.ik_environment, self.ik_group_damped, 
                self.simik.method_damped_least_squares, damping_factor, max_iterations)
        except:
            print("Creating IK environment failed")
            raise
    
    def update_ik(self, constrained = False):
        if constrained:
            self.simik.handleGroup(self.ik_environment, self.ik_group_undamped_constrained, {"syncWorlds" : True})
            return
        self.simik.handleGroup(self.ik_environment, self.ik_group_undamped, {"syncWorlds" : True})

    def move_to_callback(self, data):
        self.sim.setObjectPose(data["auxData"], data["pose"])
        self.update_ik()
  
    def reset_target(self):
        self.sim.setObjectPose(self.target_handle, self.sim.getObjectPose(self.tip_handle))
        
    def set_target_pose(self, position, angles):
        self.sim.setObjectPosition(self.target_handle, position)
        self.sim.setObjectOrientation(self.target_handle, position)
        
    def move_to(self, position, angles):
        pose = self.sim.buildPose(position, angles)
        # TODO dont hardcode params
        params = {
            "object" : self.target_handle,
            "targetPose" : pose,
            "maxVel" : [10, 10, 10, 10],
            "maxAccel" : [10, 10, 10, 10],
            "maxJerk" : [0.01, 0.01, 0.01, 0.01],
            "auxData" : self.target_handle,
            "callback" : self.move_to_callback
        }
        self.sim.moveToPose(params)
        
    def rotate_to_callback(self, data):
        self.sim.setJointPosition(data["auxData"], data["pos"][0])
        
    def rotate_to(self, position, max_velocity, max_acceleration, max_jerk, delta = False):
        if delta:
            position += self.sim.getJointPosition(self.get_joint_handle(0))
        
        self.sim.moveToConfig({
            "joints" : [self.get_joint_handle(0)],
            "targetPos" : [position],
            "maxVel" : [max_velocity],
            "maxAccel" : [max_acceleration],
            "maxJerk" : [max_jerk],
            "cyclicJoints" : [True],
            "auxData" : self.get_joint_handle(0),
            "callback" : self.rotate_to_callback
        })
        
    