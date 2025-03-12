import threading
from time import sleep
import math
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

# Using KR10_and_battery.ttt

def action_kr10_1():
    import controller
    
    client = RemoteAPIClient()
    sim = client.require('sim')
    simIK = client.require('simIK')
    
    try:
        kr10 = controller.generic_ik(sim, simIK, False, "KR10_1")
        
        kr10.reset_target()
        
        default_pos = sim.getObjectPosition(kr10.get_target())
        default_orient = sim.getObjectOrientation(kr10.get_target())
        # Distance above the bolt before it couples 
        bolt_offset = 0.3
        # Rotational speed of the end effector (rad/s)
        radians_per_second = math.pi / 2
        
        def spin_up():
            sim.setJointTargetVelocity(kr10.get_joint_handle(5), radians_per_second)
            
        def spin_down():
            sim.setJointTargetVelocity(kr10.get_joint_handle(5), 0)
            sim.setJointPosition(kr10.get_joint_handle(5), 0)
        
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
                
                spin_up()
                
                total_rotation += radians_per_tick
                # Move in direction of end effector
                target_position = sim.getObjectPosition(kr10.get_target())
                target_position[2] += distance_per_tick
                
                sim.setObjectPosition(kr10.get_target(), target_position)
                kr10.update_ik(constrained = False)
                
            spin_down()

        bolt_recepticle = kr10.get_child_object("bolt_recepticle")

        for i in range(1, 3):
            bolt = sim.getObject(f"/battery/bolt{i}/bolt_btm/bolt_top/")
            bolt_pos = sim.getObjectPosition(bolt)
            bolt_orient = sim.getObjectOrientation(bolt)
            # Position just above the bolt
            kr10.move_to([bolt_pos[0], bolt_pos[1], bolt_pos[2] + bolt_offset], sim.getObjectOrientation(bolt))
            sleep(0.5)
            # Go to bolt head
            kr10.move_to(sim.getObjectPosition(bolt), sim.getObjectOrientation(bolt))
            sleep(0.5)
            sim.setObjectParent(bolt, kr10.get_joint_handle(5))
            unscrew_bolt()
            sleep(0.5)
            # Return to default position
            kr10.move_to(default_pos, default_orient)
            kr10.rotate_to(math.pi, 0.5, 0.1, 0.1, delta = True)
            old_pos = sim.getObjectPosition(kr10.get_tip())
            old_orient = sim.getObjectOrientation(kr10.get_tip())
            kr10.reset_target()
            kr10.move_to(sim.getObjectPosition(bolt_recepticle), bolt_orient)
            sim.setObjectParent(bolt, -1)
            kr10.move_to(old_pos, old_orient)
            kr10.rotate_to(math.pi, 0.5, 0.1, 0.1, delta = True)
            kr10.reset_target()
            kr10.move_to(default_pos, default_orient)
    except:
        print("Thread failed")
        raise
        
def action_kr10_2():
    import controller
    
    client = RemoteAPIClient()
    sim = client.require('sim')
    simIK = client.require('simIK')
    
    try:
        kr10 = controller.generic_ik(sim, simIK, False, "KR10_2")
        
        kr10.reset_target()
        
        default_pos = sim.getObjectPosition(kr10.get_target())
        default_orient = sim.getObjectOrientation(kr10.get_target())
        # Distance above the bolt before it couples 
        bolt_offset = 0.3
        # Rotational speed of the end effector (rad/s)
        radians_per_second = math.pi / 2
        
        def spin_up():
            sim.setJointTargetVelocity(kr10.get_joint_handle(5), radians_per_second)
            
        def spin_down():
            sim.setJointTargetVelocity(kr10.get_joint_handle(5), 0)
            sim.setJointPosition(kr10.get_joint_handle(5), 0)
        
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
                
                spin_up()
                
                total_rotation += radians_per_tick
                # Move in direction of end effector
                target_position = sim.getObjectPosition(kr10.get_target())
                target_position[2] += distance_per_tick
                
                sim.setObjectPosition(kr10.get_target(), target_position)
                kr10.update_ik(constrained = False)
                
            spin_down()

        bolt_recepticle = kr10.get_child_object("bolt_recepticle")

        for i in range(12, 10, -1):
            bolt = sim.getObject(f"/battery/bolt{i}/bolt_btm/bolt_top/")
            bolt_pos = sim.getObjectPosition(bolt)
            bolt_orient = sim.getObjectOrientation(bolt)
            # Position just above the bolt
            kr10.move_to([bolt_pos[0], bolt_pos[1], bolt_pos[2] + bolt_offset], sim.getObjectOrientation(bolt))
            sleep(0.5)
            # Go to bolt head
            kr10.move_to(sim.getObjectPosition(bolt), sim.getObjectOrientation(bolt))
            sleep(0.5)
            sim.setObjectParent(bolt, kr10.get_joint_handle(5))
            unscrew_bolt()
            sleep(0.5)
            # Return to default position
            kr10.move_to(default_pos, default_orient)
            kr10.rotate_to(math.pi, 0.5, 0.1, 0.1, delta = True)
            old_pos = sim.getObjectPosition(kr10.get_tip())
            old_orient = sim.getObjectOrientation(kr10.get_tip())
            kr10.reset_target()
            kr10.move_to(sim.getObjectPosition(bolt_recepticle), bolt_orient)
            sim.setObjectParent(bolt, -1)
            kr10.move_to(old_pos, old_orient)
            kr10.rotate_to(math.pi, 0.5, 0.1, 0.1, delta = True)
            kr10.reset_target()
            kr10.move_to(default_pos, default_orient)
    except:
        print("Thread failed")
        raise        

client = RemoteAPIClient()
sim = client.require('sim')

print("Simulation started")

kr10_1 = threading.Thread(target = action_kr10_1)
kr10_1.start()

kr10_2 = threading.Thread(target = action_kr10_2)
kr10_2.start()

sim.startSimulation()
kr10_1.join()
kr10_2.join()
sim.stopSimulation()

print("Simulation stopped")