from time import sleep
from coppeliasim_zmqremoteapi_client import RemoteAPIClient


try:
    print("Connecting to remote API")
    client = RemoteAPIClient()
    sim = client.require('sim')

    print("success")

    sim.setStepping(True)
    sim.startSimulation()

    while sim.getSimulationTime() < 10:
        print(sim.getSimulationTime())
        sim.step()

    sim.stopSimulation()
except Exception as e:
    print(e)