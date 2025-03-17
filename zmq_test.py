import threading
import math

from time import sleep
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

# Using KR10_and_battery_multi.ttt

def unscrew_bolts(params):
    import controller
    
    client = RemoteAPIClient()
    sim = client.require('sim')
    simIK = client.require('simIK')
    
    try:
        kr10 = controller.generic_ik(sim, simIK, False, params["id"])
        
        kr10.reset_target()
        
        default_pos = sim.getObjectPosition(kr10.get_target())
        default_orient = sim.getObjectOrientation(kr10.get_target())
        # Distance above the bolt before it couples 
        bolt_offset = 0.3
        # Rotational speed of the end effector (rad/s)
        radians_per_second = math.pi
        
        #sim.setStepping(True)
        
        def unscrew_bolt():
            # How far the bolt should be unscrewed before trying to move it
            minimum_clearance = 0.3
            # How far the bolt moves for every revolution of the thread
            distance_per_revolution = 0.2
            revolutions = minimum_clearance / distance_per_revolution
            # How many radians/s the end effector can do
            
            total_rotation = 0
            max_rotation = revolutions * 2 * math.pi

            sim.setJointTargetVelocity(kr10.get_joint_handle(5), radians_per_second)
            
            while (total_rotation <= max_rotation):
                radians_per_tick = sim.getSimulationTimeStep() * radians_per_second
                distance_per_tick = distance_per_revolution * radians_per_tick / (2 * math.pi)
                
                #joint_pos = sim.getJointPosition(kr10.get_joint_handle(5))
                
                total_rotation += radians_per_tick
                # Move in direction of end effector
                target_position = sim.getObjectPosition(kr10.get_target())
                # Start moving only when the end effector is spinning
                target_position[2] += distance_per_tick * min(sim.getJointVelocity(kr10.get_joint_handle(5)) / radians_per_second, 1)
                # Move robot upwards while unscrewing
                sim.setObjectPosition(kr10.get_target(), target_position)
                kr10.update_ik(constrained = False)
                
                #sim.step()
                
            sim.setJointTargetVelocity(kr10.get_joint_handle(5), 0)
    
        bolt_recepticle = kr10.get_child_object("bolt_recepticle")

        for i in params["bolt_order"]:
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
            kr10.rotate_to(math.pi, 10, 10, 10, delta = True)
            old_pos = sim.getObjectPosition(kr10.get_tip())
            old_orient = sim.getObjectOrientation(kr10.get_tip())
            kr10.reset_target()
            kr10.move_to(sim.getObjectPosition(bolt_recepticle), bolt_orient)
            sim.setObjectParent(bolt, -1)
            kr10.move_to(old_pos, old_orient)
            kr10.rotate_to(math.pi, 10, 10, 10, delta = True)
            kr10.reset_target()
            kr10.move_to(default_pos, default_orient)
            
        sim.setStepping(False)
    except:
        print("Thread failed")
        raise
                
def move_to_smooth(obj_handle, end_position, duration, delta = False):
    if duration <= 0:
        raise RuntimeError("move_to_smooth: duration <= 0")

    pos = sim.getObjectPosition(obj_handle)
    goal = [end_position[0], end_position[1], end_position[2]]
    if delta:
        goal[0] += pos[0];
        goal[1] += pos[1];
        goal[2] += pos[2];
    
    dx = (goal[0] - pos[0])
    dy = (goal[1] - pos[1])
    dz = (goal[2] - pos[2])
    
    step = 0

    sim.setStepping(True)
    
    while step <= duration:
        t = math.sin(step / duration * math.pi * 0.5)
        
        sim.setObjectPosition(obj_handle, [
            pos[0] + dx * t,
            pos[1] + dy * t,
            pos[2] + dz * t,
        ])
        step += sim.getSimulationTimeStep()
        sim.step()
        
    sim.setStepping(False)    

client = RemoteAPIClient()
sim = client.require('sim')

#sim.adjustView(0, sim.getObject("/camera"), 64)
#sim.adjustView(0, sim.getObject("/DefaultCamera"), 64)

print("Simulation started")

kr10_1 = threading.Thread(target = unscrew_bolts, args = ({
    "id" : "KR10_1",
    "bolt_order" : [1, 2]
},))

kr10_2 = threading.Thread(target = unscrew_bolts, args = ({
    "id" : "KR10_2",
    "bolt_order" : [12, 11]
},))


sim.startSimulation()

sim.setObjectPosition(sim.getObject("/battery"), [-5, 0, 0])
move_to_smooth(sim.getObject("/battery"), [0, 0, 0], 4, delta = False)

kr10_1.start()
kr10_2.start()

"""
sim.setObjectParent(sim.getObject("/camera_topdown"), sim.getObject("/battery"))
sim.setObjectPosition(sim.getObject("/battery"), [-5, 0, 0])

sim.adjustView(0, sim.getObject("/camera_topdown"), 64)
move_to_smooth(sim.getObject("/battery"), [0, 0, 0], 4, delta = False)

kr10_1.start()
kr10_2.start()

sim.wait(1)
sim.adjustView(0, sim.getObject("/camera_closein"), 64)
sim.wait(4)
sim.adjustView(0, sim.getObject("/camera_tip"), 64)
sim.wait(6)
sim.adjustView(0, sim.getObject("/camera_long"), 64)
sim.wait(15)
sim.adjustView(0, sim.getObject("/camera_topdown"), 64)
sim.wait(6)
sim.adjustView(0, sim.getObject("/camera_long"), 64)
"""

kr10_1.join()
kr10_2.join()

# All bolts are removed at this point

move_to_smooth(sim.getObject("/battery/upper_housing"), [0, 0, 1.5], 4, delta = True)

sim.stopSimulation()

sim.adjustView(0, sim.getObject("/DefaultCamera"), 64)

print("Simulation stopped")