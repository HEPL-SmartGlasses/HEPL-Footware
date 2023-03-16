import csv
import matplotlib

import os
os.environ['LD_LIBRARY_PATH'] = os.getcwd()
import pyximport
pyximport.install(setup_args={"script_args" : ["--verbose"]})
from wrapper import call


with open(""C:\Users\evanm\Documents\Michigan\eecs373\Project\PyShoe Dataset\pyshoe\data\vicon\raw\2018-02-22-10-10-56\imudata.csv"") as csvfile:
    csv_reader = csv.DictReader(csvfile, delimiter=',')
    for row in csv_reader:
        if line_count == 0:
            continue
        
        data = ()
        
        
        call(*data, timeDelta)



