import os
os.chdir(os.path.dirname(os.path.abspath(__file__)))

ins_csv = '/media/moon/moon_ssd/moon ubuntu/oxford/0514/2014-05-14-13-46-12_gps/2014-05-14-13-46-12/gps/ins.csv'
front_cam = '/media/moon/moon_ssd/moon ubuntu/oxford/0514/2014-05-14-13-46-12_stereo_centre_01/2014-05-14-13-46-12/stereo.timestamps'

camera_time_list = []
ins_time_list = []

with open(front_cam, 'r') as file:
    for line in file:
        camera_time_list.append(line.split(' ')[0])

print(len(camera_time_list))

with open(ins_csv, 'r') as file:
    for line in file:
        if line[0] == 't': continue
        if line.split(',')[1] == 'INS_SOLUTION_GOOD':
            ins_time_list.append(line.split(',')[0])

print(len(ins_time_list))

test = []
for cam in camera_time_list:
    for ins in ins_time_list:
        if abs(int(cam) - int(ins)) < 10000:
            test.append(cam)
            break

print(len(test))