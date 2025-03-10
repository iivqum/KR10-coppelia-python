import threading
from dependencies import *
from time import sleep

import coppeliasim.bridge

class generic_ik_thread(threading.Thread):
    def __init__(self, is_model = True, model_name = ""):
        super().__init__()
        ik = generic_ik(is_model, model_name)
        self._kwargs["ik_object"] = ik
        self._kwargs["sim"] = sim
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
        sim = self._kwargs["sim"]
        
        try:
            while not self.terminate:
                sim.step()
        except Exception as e:
            print(f"Thread failure\n{e}")
            raise
        
        print("IK Thread terminated successfully")
        
        

class generic_ik:
    # Generic arm controller class that performs IK and positioning of the arm
    
    def __init__(self, is_model = True, model_name = ""):
        """
        * A dummy object must be connected to the end effector called "tip".
        * A dummy object called "target" must be in the model hierarchy
        * Object must not be dynamic.
        
        If is_model is False, the object will be searched for in the scene,
        otherwise it is loaded from the models/ directory.
        """
        
        print(f"Attempting to load {str(model_name)}")
        
        if not isinstance(model_name, str):
            raise TypeError("Model name not string")
        
        self.joints = list()
        
        try:
            if is_model:
                self.handle = sim.loadModel(f"models/{model_name}.ttm")
            else:
                self.handle = sim.getObject(f"/{model_name}", {"noError" : False})
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
    
    def get_tip(self):
        return self.tip_handle
 
    def get_target(self):
        return self.target_handle
    
    def is_valid(self):
        return self.handle > 0 and self.target_handle > 0 and self.tip_handle and len(self.joints) > 0
    
    def setup_required(self):
        # TODO is this in order?
        # TODO dynamic joint warning
        # Add joints
        for joint_handle in sim.getObjectsInTree(self.handle, 
            sim.sceneobject_joint):
            self.joints.append(joint_handle)
        # Find tip and target
        for dummy_handle in sim.getObjectsInTree(self.handle, 
            sim.sceneobject_dummy):
            # TODO create tip and target when none are found
            # Last object in tree is selected !!!
            try:
                alias = sim.getStringProperty(dummy_handle, "alias", {"noError" : False}).lower()
                               
                if alias == "tip":
                    self.tip_handle = dummy_handle
                elif alias == "target":
                    self.target_handle = dummy_handle
            except:
                raise
        if not hasattr(self, "tip_handle"):
            raise Exception("Couldn't find tip dummy")
        if not hasattr(self, "target_handle"):
            raise Exception("Couldn't find target dummy")
    
    def setup_ik(self, damping_factor, max_iterations):
        try:
            self.ik_environment = simIK.createEnvironment()
            self.ik_group_undamped = simIK.createGroup(self.ik_environment)
            self.ik_group_damped = simIK.createGroup(self.ik_environment)
             
            simIK.setGroupCalculation(self.ik_environment, self.ik_group_undamped, 
                simIK.method_pseudo_inverse, 0, max_iterations)
            simIK.addElementFromScene(self.ik_environment, self.ik_group_undamped, self.handle, 
                self.tip_handle, self.target_handle, simIK.constraint_pose)
            simIK.setGroupCalculation(self.ik_environment, self.ik_group_damped, 
                simIK.method_damped_least_squares, damping_factor, max_iterations)
        except:
            print("Creating IK environment failed")
            raise
    
    def update(self):
        simIK.handleGroup(self.ik_environment, self.ik_group_undamped, {"syncWorlds" : True})