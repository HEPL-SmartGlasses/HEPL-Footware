import os
os.environ['LD_LIBRARY_PATH'] = os.getcwd()
import pyximport
pyximport.install(setup_args={"script_args" : ["--verbose"]})
from wrapper import call

call()
