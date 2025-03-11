import threading
from time import sleep
import math

from dependencies import *

# Using KR10_and_battery.ttt

def action():
    # Also imports dependencies
    import controller
    try:
        kr10 = controller.generic_ik(False, "KR10")
        
        kr10.reset_target()
        
        bolt = sim.getObject("/battery/bolt6/bolt_btm/bolt_top")
        
        kr10.move_to(sim.getObjectPosition(bolt), sim.getObjectOrientation(bolt))
    except:
        print("Thread failed")
        raise

sim.startSimulation()

test = threading.Thread(target = action)
test.start()
test.join()

sim.stopSimulation()

print("Simulation stopped")