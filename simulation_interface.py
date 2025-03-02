# coppeliasim.bridge.load must be called from main.py
from dependencies import *

import controller

def setup():
    # Called when simulation is initialized
    # Returns False when setup failed
    # handle_kr10 = sim.loadModel("models/KR10.ttm")
    
    try:
        kr10_controller = controller.generic_ik(True, "KR10")
        print("Succesfully configured KR10")
    except Exception as e:
        print(f"Failed to create controller\n {e}")
        return False
    
    sim.startSimulation()
    
    return True
        
def run():
    # Called every time the simulation runs
    # Simulation thread exits when this function returns false
    
    #print(sim.getSimulationTime())
    
    return False
    
def finish():
    # Called when simulation finishes (when run() returns false)
    sim.stopSimulation()