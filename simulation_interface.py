# coppeliasim.bridge.load must be called from main.py
import coppeliasim.bridge

def setup():
    # Called when simulation is initialized
    global sim, simIK
    sim = coppeliasim.bridge.require("sim")
    simIK = coppeliasim.bridge.require("simIK")

    sim.startSimulation()
        
def run():
    # Called every time the simulation runs
    # Simulation thread exits when this function returns false
    if sim.getSimulationTime() >= 4:
        return False
    
    
    return True
    
def finish():
    sim.stopSimulation()