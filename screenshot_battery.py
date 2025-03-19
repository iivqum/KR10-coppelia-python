import math

from coppeliasim_zmqremoteapi_client import RemoteAPIClient

# battery_screenshot.ttt

client = RemoteAPIClient()
sim = client.require('sim')

try:
    sensor = sim.getObject("/battery/upper_housing/center/sensor")
    center = sim.getObject("/battery/upper_housing/center")

    sim.startSimulation()
    print("Simulation started")
    
    images_per_axis = 16
    default_pose = sim.getObjectPose(sensor)
    capture_number = -1
    
    def capture():
        global capture_number
        image, resolution = sim.getVisionSensorImg(sensor, 2)
        sim.saveImage(image, resolution, 1, f"battery{capture_number}.jpg", 100)
        capture_number = capture_number + 1
        
    # For some reason the first image is always black
    capture()

    # With bolts
    for i in range(1, images_per_axis + 1):
        new_pose = sim.rotateAroundAxis(sim.getObjectPose(sensor), [0, 1, 0], sim.getObjectPosition(center), 2 * math.pi / images_per_axis)
        sim.setObjectPose(sensor, new_pose)
        capture()
        
    for i in range(1, images_per_axis + 1):
        new_pose = sim.rotateAroundAxis(sim.getObjectPose(sensor), [1, 0, 0], sim.getObjectPosition(center), 2 * math.pi / images_per_axis)
        sim.setObjectPose(sensor, new_pose)
        capture()
        
    for i in range(1, int(images_per_axis / 2) + 1):
        new_pose = sim.rotateAroundAxis(sim.getObjectPose(sensor), [0, 0, 1], sim.getObjectPosition(center), 2 * math.pi / images_per_axis)
        sim.setObjectPose(sensor, new_pose)
        capture()
 
    sim.setObjectPose(sensor, default_pose)
 
    new_pose = sim.rotateAroundAxis(sim.getObjectPose(sensor), [1, 0, 0], sim.getObjectPosition(center), -math.pi * 0.5)
    sim.setObjectPose(sensor, new_pose)
    
    for i in range(1, images_per_axis + 1):
        new_pose = sim.rotateAroundAxis(sim.getObjectPose(sensor), [0, 0, 1], sim.getObjectPosition(center), 2 * math.pi / images_per_axis)
        sim.setObjectPose(sensor, new_pose)
        capture()
        
    sim.setObjectPose(sensor, default_pose)
    
    new_pose = sim.rotateAroundAxis(sim.getObjectPose(sensor), [-1, 0, 0], sim.getObjectPosition(center), math.pi * 0.25)
    sim.setObjectPose(sensor, new_pose)    
  
    for i in range(1, images_per_axis + 1):
        new_pose = sim.rotateAroundAxis(sim.getObjectPose(sensor), [0, 0, 1], sim.getObjectPosition(center), 2 * math.pi / images_per_axis)
        sim.setObjectPose(sensor, new_pose)
        capture()
  
    sim.setObjectPose(sensor, default_pose)
except:
    raise
    
sim.stopSimulation()

print("Simulation stopped")