import argparse
from time import sleep

def sim_thread():
    from coppeliasim.lib import (appDir, simInitialize, simLoop, 
        simDeinitialize, simGetExitRequest)
        
    import coppeliasim.bridge
    
    simInitialize(appDir().encode('utf-8'), 0)    
    
    def step():
        # Steps through the simulation once
        if sim.getSimulationState() != sim.simulation_stopped:
            t = sim.getSimulationTime()
            while t == sim.getSimulationTime():
                simLoop(None, 0)    
    
    try:
        coppeliasim.bridge.load()
        sim = coppeliasim.bridge.require("sim")
        import simulation_interface
        
        simulation_interface.setup()
        
        while (simulation_interface.run() and not simGetExitRequest()):
            simLoop(None, 0)
            # TODO Find a better way of doing this
            sleep(sim.getSimulationTimeStep());
        
        simulation_interface.finish()
    except Exception:
        import traceback
        print(traceback.format_exc())
    
    simDeinitialize()
    
if __name__ == "__main__":    
    import coppeliasim.cmdopt
    # Use -L to pass the binaries folder (Coppelia/CoppeliaSim.dll)
    parser = argparse.ArgumentParser(description = "Coppelia Sim", add_help = False)

    coppeliasim.cmdopt.add(parser)

    args = parser.parse_args()
    # Set up Coppelia C bindings
    coppeliasim.cmdopt.read_args(args)

    from coppeliasim.lib import (simRunGui, sim_gui_headless)
    import threading
    #py main.py -L "C:\Users\Josh\Desktop\roboreclaim\Coppelia\CoppeliaSim.dll"

    # If Coppelia isn't running in headless mode, then it must be threaded
    # Coppelia runs in its own thread so it doesn't block
    t = threading.Thread(target = sim_thread)
    t.start()
    # No GUI
    simRunGui(sim_gui_headless)
    t.join()

    print("main thread terminated\n")