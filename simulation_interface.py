# coppeliasim.bridge.load must be called from main.py
from dependencies import *

import controller

def setup():
    # Called when simulation is initialized
    # handle_kr10 = sim.loadModel("models/KR10.ttm")
    
    
    
    kr10_controller = controller.generic_ik(True, "KR10")
    
    
    
    sim.startSimulation()
        
def run():
    # Called every time the simulation runs
    # Simulation thread exits when this function returns false
    if sim.getSimulationTime() >= 4:
        return False
    
    print(sim.getSimulationTime())
    
    return True
    
def finish():
    # Called when simulation finishes (when run() returns false)
    sim.stopSimulation()