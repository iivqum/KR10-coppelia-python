import argparse
import coppeliasim.cmdopt

def thread_func():
    from coppeliasim.lib import (
        appDir,
        simInitialize,
        simLoop,
        simDeinitialize,
        simGetExitRequest,
    )

    simInitialize(appDir().encode('utf-8'), 0)

    while not simGetExitRequest():
        simLoop(None, 0)

    simDeinitialize()

# Use -L to pass the binaries folder (Coppelia/CoppeliaSim.dll)
parser = argparse.ArgumentParser(description = "Coppelia Sim", add_help = False)

coppeliasim.cmdopt.add(parser)

args = parser.parse_args()
# Set up Coppelia C bindings
coppeliasim.cmdopt.read_args(args)

from coppeliasim.lib import *
import threading

t = threading.Thread(target = thread_func)
t.start()
# Headless mode runs Coppelia without the GUI
simRunGui(sim_gui_headless)
t.join()