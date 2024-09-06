import os
os.chdir(os.path.dirname(os.path.abspath(__file__)))

ins_csv = '/media/moon/moon_ssd/moon ubuntu/oxford/0514/2014-05-14-13-46-12_gps/2014-05-14-13-46-12/gps/ins.csv'
front_cam = '/media/moon/moon_ssd/moon ubuntu/oxford/0514/2014-05-14-13-46-12_stereo_centre_01/2014-05-14-13-46-12/stereo.timestamps'

camera_time_list = []

with open(front_cam, 'r') as file:
    for line in file:
        camera_time_list.append(line.split(' ')[0])

print(len(camera_time_list))

with open(ins_csv, 'r') as file:
    cnt = 0

    for line in file:
        print(line)
        cnt += 1

        if cnt > 10: break