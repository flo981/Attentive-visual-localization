
from rosbag import Bag
import os

path = '/home/flo/testbag'


files = []
# r=root, d=directories, f = files
for r, d, f in os.walk(path):
    i=0
    j=0
    l=0
    for file in f:
        if file.endswith(".bag"):
            print file
            i+=1
            j+=1
            l+=1
            if file[0] == "0":
                with Bag("new_"+file, 'w') as Y:
                    for topic, msg, t in Bag(file):
                        if topic == 'orb_pose':
                            Y.write('orb_pose_density_'+str(i), msg, t)
                        if topic == 'groundtruth':
                            Y.write('groundtruth_d_'+str(i), msg, t)


            if file[0] == "1":
                with Bag("new_"+file, 'w') as Y:
                    for topic, msg, t in Bag(file):

                        if topic == 'orb_pose':
                            Y.write('orb_pose_vocus2_'+str(j), msg, t)
                        if topic == 'groundtruth':
                            Y.write('groundtruth_v_'+str(j), msg, t)
            if file[0] == "n":
                with Bag("new_"+file, 'w') as Y:
                    for topic, msg, t in Bag(file):

                        if topic == 'orb_pose':
                            Y.write('orb_pose_'+str(l), msg, t)
                        if topic == 'groundtruth':
                            Y.write('groundtruth_'+str(l), msg, t)
