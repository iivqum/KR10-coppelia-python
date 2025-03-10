# coppeliasim.bridge.load must be called from main.py
from dependencies import *

import math
import controller

def setup():
    # Called when simulation is initialized
    # Returns False when setup failed
    # handle_kr10 = sim.loadModel("models/KR10.ttm")
    
    try:
        global kr10_controller_thread 
        kr10_controller_thread = controller.generic_ik_thread(True, "KR10")
        print("Succesfully loaded KR10\n")
    except Exception as e:
        print(f"Failed to create controller\n {e}")
        return False
    
    return True
        
def run():
    # Called every time the simulation runs
    # Simulation thread exits when this function returns false
    
    #print(sim.getSimulationTime())

    return True
    
def finish():
    # Called when run() returns falls or there is an exit request
    kr10_controller_thread.kill()
    sim.stopSimulation()