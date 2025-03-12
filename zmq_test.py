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
        # Distance above the bolt before it couples 
        bolt_offset = 0.3        
        radians_per_second = math.pi / 2
        
        def spin_up():
            sim.setJointTargetVelocity(kr10.get_joint_handle(5), radians_per_second)
            
        def spin_down():
            sim.setJointTargetVelocity(kr10.get_joint_handle(5), 0)
        
        def unscrew_bolt():
            # How far the bolt should be unscrewed before trying to move it
            minimum_clearance = 0.2
            # How far the bolt moves for every revolution of the thread
            distance_per_revolution = 0.02
            revolutions = minimum_clearance / distance_per_revolution
            # How many radians/s the end effector can do
            
            total_rotation = 0
            max_rotation = revolutions * 2 * math.pi
            
            while (total_rotation <= max_rotation):
                radians_per_tick = sim.getSimulationTimeStep() * radians_per_second
                distance_per_tick = distance_per_revolution * radians_per_tick / (2 * math.pi)
                joint_pos = sim.getJointPosition(kr10.get_joint_handle(5))
                
                #sim.setJointTargetPosition(kr10.get_joint_handle(5), joint_pos + radians_per_tick)
                spin_up()
                #im.setJointPosition(kr10.get_joint_handle(5), joint_pos + radians_per_tick)
                
                total_rotation += radians_per_tick
                # Move in direction of end effector
                target_position = sim.getObjectPosition(kr10.get_target())
                target_position[2] += distance_per_tick
                
                sim.setObjectPosition(kr10.get_target(), target_position)
                kr10.update_ik(constrained = False)
                
            spin_down()

        
        for i in range(1, 2):
            bolt = sim.getObject(f"/battery/bolt{i}/bolt_btm/bolt_top/")
            bolt_pos = sim.getObjectPosition(bolt)
            # Position just above the bolt
            kr10.move_to([bolt_pos[0], bolt_pos[1], bolt_pos[2] + bolt_offset], sim.getObjectOrientation(bolt))
            # Go to bolt head
            kr10.move_to(sim.getObjectPosition(bolt), sim.getObjectOrientation(bolt))
            sim.setObjectParent(bolt, kr10.get_joint_handle(5))
            unscrew_bolt()
            # Return to default position
            kr10.move_to(default_pos, default_orient)
    except:
        print("Thread failed")
        raise

sim.startSimulation()
print("Simulation started")

test = threading.Thread(target = action)
test.start()
test.join()

sim.stopSimulation()

print("Simulation stopped")