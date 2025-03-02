from dependencies import *

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
        self.setup_ik()
        
    def is_valid(self):
        return self.handle > 0 and self.target_handle > 0 and self.tip_handle and len(self.joints) > 0
    
    def setup_required(self):
        # TODO is this in order?
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
    
    def setup_ik(self):
        try:
            self.ik_environment = simIK.createEnvironment()
            
            self.ik_group_undamped = simIK.createGroup(self.ik_environment)
        
            simIK.setGroupCalculation(self.ik_environment, self.ik_group_undamped, 
                simIK.method_pseudo_inverse, 0, 6)
            simIK.addElementFromScene(self.ik_environment, self.ik_group_undamped, self.handle, 
                self.tip_handle, self.target_handle, simIK.constraint_pose)
            
            self.ik_group_damped = simIK.createGroup(self.ik_environment)
            simIK.setGroupCalculation(self.ik_environment, self.ik_group_damped, simIK.method_damped_least_squares, 1, 99)
        except:
            print("Creating IK environment failed")
            raise