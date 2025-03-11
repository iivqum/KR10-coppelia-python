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
        
        default_pos = sim.getObjectPosition(kr10.get_target())
        default_orient = sim.getObjectOrientation(kr10.get_target())
        
        for i in range(1, 7):
            bolt = sim.getObject(f"/battery/bolt{i}/bolt_btm/bolt_top/")
                
            kr10.move_to(sim.getObjectPosition(bolt), sim.getObjectOrientation(bolt))
            kr10.move_to(default_pos, default_orient)
    except:
        print("Thread failed")
        raise

sim.startSimulation()

test = threading.Thread(target = action)
test.start()
test.join()

sim.stopSimulation()

print("Simulation stopped")