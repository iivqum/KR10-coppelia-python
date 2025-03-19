import threading
import math
import queue

from time import sleep
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

# Using KR10_and_battery_multi.ttt

def unscrew_bolts(ik_controller, params):
    import controller

    kr10 = ik_controller
    
    client = RemoteAPIClient()
    sim = client.require('sim')
    simIK = client.require('simIK')
    
    try:
        kr10.reset_target()
        
        default_pos = sim.getObjectPosition(kr10.get_target())
        default_orient = sim.getObjectOrientation(kr10.get_target())
        # Distance above the bolt before it couples 
        bolt_offset = 0.3
        # Rotational speed of the end effector (rad/s)
        radians_per_second = math.pi
        
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
                sim.wait(sim.getSimulationTimeStep())
                
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
        print("Action failed")
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

def corner_grab(ik_controller, params):
    kr10 = ik_controller
    client = RemoteAPIClient()
    sim = client.require('sim')
    simIK = client.require('simIK')
    
    try:
        kr10.reset_target()
        
        grab = kr10.get_child_object("corner_grab")
        grab_pos = sim.getObjectPosition(grab)
        grab_orient = sim.getObjectOrientation(grab)
        
        kr10.move_to([grab_pos[0], grab_pos[1], grab_pos[2] + 0.3], grab_orient)
        kr10.move_to(grab_pos, grab_orient)
        
        kr10.reset_target()
        
        sim.setObjectParent(kr10.get_target(), sim.getObject("/battery/upper_housing"))
    except:
        print("Action failed")
        raise

def task(params):
    import controller
    
    client = RemoteAPIClient()
    sim = client.require('sim')
    simIK = client.require('simIK')
    
    queue = params["queue"]
    terminate = False
    kr10 = controller.generic_ik(sim, simIK, False, params["id"])
    
    while not terminate and sim.getSimulationState() != sim.simulation_stopped:
        try:
            command = queue.get_nowait()
            
            print(f"{params["id"]} -> {command["cmd"]}")
            
            if command["cmd"] == "kill":
                terminate = True
            elif command["cmd"] == "unscrew":
                unscrew_bolts(kr10, command["params"])
            elif command["cmd"] == "corner_grab":
                corner_grab(kr10, command["params"])
            print(f"{params["id"]} -> {command["cmd"]} -> finish")
            queue.task_done()
        except:
            kr10.update_ik(constrained = False)
    
client = RemoteAPIClient()
sim = client.require('sim')

print("Simulation started")

kr10_1_cmd = queue.Queue(maxsize = 0)
kr10_1_task= threading.Thread(target = task, args = ({"queue" : kr10_1_cmd, "id" : "KR10_1"},))

kr10_2_cmd = queue.Queue(maxsize = 0)
kr10_2_task = threading.Thread(target = task, args = ({"queue" : kr10_2_cmd, "id" : "KR10_2"},))

sim.startSimulation()

sim.adjustView(0, sim.getObject("/camera_topdown"), 64)
sim.setObjectParent(sim.getObject("/camera_topdown"), sim.getObject("/battery"))
sim.setObjectPosition(sim.getObject("/battery"), [-5, 0, 0])
move_to_smooth(sim.getObject("/battery"), [0, 0, 0], 4, delta = False)

kr10_1_task.start()
kr10_2_task.start()

kr10_1_cmd.put({"cmd" : "unscrew", "params" : {"bolt_order" : [1, 2]}})
kr10_2_cmd.put({"cmd" : "unscrew", "params" : {"bolt_order" : [12, 11]}})

sim.wait(1)
sim.adjustView(0, sim.getObject("/camera_closein"), 64)
sim.wait(4)
sim.adjustView(0, sim.getObject("/camera_tip"), 64)
sim.wait(6)
sim.adjustView(0, sim.getObject("/camera_long"), 64)
sim.wait(10)
sim.adjustView(0, sim.getObject("/camera_long"), 64)
sim.wait(6)
sim.adjustView(0, sim.getObject("/camera_topdown"), 64)

kr10_1_cmd.join()
kr10_2_cmd.join()

# All bolts are removed at this point
"""
move_to_smooth(sim.getObject("/battery/upper_housing"), [0, 0, 1.2], 4, delta = True)
sim.wait(1)

for i in range(1, 9):
    move_to_smooth(sim.getObject(f"/battery/cell{i}"), [0, 0, 0.5], 0.25, delta = True)
"""
kr10_1_cmd.put({"cmd" : "corner_grab", "params" : {}})
kr10_2_cmd.put({"cmd" : "corner_grab", "params" : {}})

kr10_1_cmd.join()
kr10_2_cmd.join()

sim.wait(4)

move_to_smooth(sim.getObject("/battery/upper_housing"), [0, 0, 0.4], 4, delta = True)
sim.setObjectParent(sim.getObject("/battery/upper_housing"), -1)
sim.setObjectParent(sim.getObject("/camera_topdown"), -1)
move_to_smooth(sim.getObject("/battery"), [5, 0, 0], 4, delta = True)

sim.wait(4)

sim.stopSimulation()
sim.adjustView(0, sim.getObject("/DefaultCamera"), 64)

print("Simulation stopped")