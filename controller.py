from dependencies import *

class generic_ik:
    # Generic arm controller class that performs IK and positioning of the arm
    
    def __init__(self, is_model = True, model_name = ""):
        """
        * Joints must be labelled as joint<number>.
        * A dummy object must be connected to the end effector called "tip".
        * Object must not be dynamic.
        
        If is_model is False, the object will be searched for in the scene,
        otherwise it is loaded from the models/ directory.
        """
        
        if not isinstance(model_name, str):
            raise TypeError()
        
        self.joints = list()
        # Unique object handle
        self.handle = -1
        
        try:
            if is_model:
                self.handle = sim.loadModel(f"models/{model_name}.ttm")
            else:
                self.handle = sim.getObject(f"/{model_name}", {"noError" : False})
        except Exception:
            raise Exception("Something went wrong loading the model")
            
        # No model found
        if self.handle == -1:
            raise Exception("Tried to load '", model_name, 
                "\nModel not found in scene or file.")
        # Path to the scene object
        self.root_path = f"/{model_name}"
        
        print(model_name + " loaded successfully!")
        
        self.setup_joints()
        self.setup_ik()
        
    def is_valid(self):
        return self.handle >= 0 and len(self.joints) > 0
    
    def setup_joints(self):
        pass
    
    def setup_ik(self):
        pass