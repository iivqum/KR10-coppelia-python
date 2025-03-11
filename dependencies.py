from coppeliasim_zmqremoteapi_client import RemoteAPIClient

global sim

client = RemoteAPIClient()
sim = client.require("sim")
simIK = client.require("simIK")

"""
# If using api
import coppeliasim.bridge

sim = coppeliasim.bridge.require("sim")
simIK = coppeliasim.bridge.require("simIK")
"""