import os
os.chdir(os.path.dirname(os.path.abspath(__file__)))

ins_csv = '/media/moon/moon_ssd/moon ubuntu/oxford/0514/2014-05-14-13-46-12_gps/2014-05-14-13-46-12/gps/ins.csv'
front_cam = '/media/moon/moon_ssd/moon ubuntu/oxford/0514/2014-05-14-13-46-12_stereo_centre_01/2014-05-14-13-46-12/stereo.timestamps'
left_cam = '/media/moon/moon_ssd/moon ubuntu/oxford/0514/2014-05-14-13-46-12_mono_left_01/2014-05-14-13-46-12/mono_left.timestamps'
right_cam = '/media/moon/moon_ssd/moon ubuntu/oxford/0514/2014-05-14-13-46-12_mono_right_01/2014-05-14-13-46-12/mono_right.timestamps'
rear_cam = '/media/moon/moon_ssd/moon ubuntu/oxford/0514/2014-05-14-13-46-12_mono_rear_01/2014-05-14-13-46-12/mono_rear.timestamps'

output_prefix = '/media/moon/moon_ssd/moon ubuntu/post_oxford/'
front_output = os.path.join(output_prefix, '0514/front')
left_output = os.path.join(output_prefix, '0514/left')
right_output = os.path.join(output_prefix, '0514/right')
rear_output = os.path.join(output_prefix, '0514/rear')
concat_output = os.path.join(output_prefix, '0514/concat')

cam_list = [front_cam, left_cam, right_cam, rear_cam]
# front_camera_time_list = []
cam_time_lists = []
ins_time_list = []

# read ins.csv and save with list
with open(ins_csv, 'r') as file:
    for line in file:
        if line[0] == 't': continue
        if line.split(',')[1] == 'INS_SOLUTION_GOOD':
            ins_time_list.append(line.split(',')[0])

print(f'length of ins time list: {len(ins_time_list)}')

# read camera timestamps and save with list
front_time_list = []
left_time_list = []
right_time_list = []
rear_time_list = []

cam_time_list = [front_time_list, left_time_list, right_time_list, rear_time_list]

for input, output in zip(cam_list, cam_time_list):
    with open(input, 'r') as file:
        for line in file:
            output.append(line.split(' ')[0])

print(f'length of front time list: {len(front_time_list)}')
print(f'length of left time list: {len(left_time_list)}')
print(f'length of right time list: {len(right_time_list)}')
print(f'length of rear time list: {len(rear_time_list)}')

ins_save_list = []
front_save_list = []
left_save_list = []
right_save_list = []
rear_save_list = []

front_flag = False
left_flag = False
right_flag = False
rear_flag = False

std_time_gap = 30000

def is_synced(cam, ins, std_time_gap, save_list) -> bool:
    if (len(save_list) > 1) and save_list[-1] == cam:
        return False

    if abs(int(cam) - int(ins)) < std_time_gap:
        save_list.append(cam)
        return True
    else:
        return False
    

for ins in ins_time_list:
    
    for cam in front_time_list:
        if is_synced(cam, ins, std_time_gap, front_save_list):
            front_flag = True
            break

    for cam in left_time_list:
        if is_synced(cam, ins, std_time_gap, left_save_list):
            left_flag = True
            break

    for cam in right_time_list:
        if is_synced(cam, ins, std_time_gap, right_save_list):
            right_flag = True
            break

    for cam in rear_time_list:
        if is_synced(cam, ins, std_time_gap, rear_save_list):
            rear_flag = True
            break

    # sync check
    if front_flag and left_flag and right_flag and rear_flag:
        ins_save_list.append(ins)

    else:
        if front_flag:
            front_save_list.pop()
        
        if left_flag:
            left_save_list.pop()

        if right_flag:
            right_save_list.pop()

        if rear_flag:
            rear_save_list.pop()
    
    # flag initialize
    front_flag = False
    left_flag = False
    right_flag = False
    rear_flag = False

print('===============================================================')
print(f'length of ins save list: {len(ins_save_list)}')
print(f'length of front save list: {len(front_save_list)}')
print(f'length of left save list: {len(left_save_list)}')
print(f'length of right save list: {len(right_save_list)}')
print(f'length of rear save list: {len(rear_save_list)}')


'''
for cam in front_time_list:
    for ins in ins_time_list:
        if abs(int(cam) - int(ins)) < 10000:
            front_save_list.append(cam)
            break
print(f'length of front save list: {len(front_save_list)}')

for cam in left_time_list:
    for ins in ins_time_list:
        if abs(int(cam) - int(ins)) < 10000:
            left_save_list.append(cam)
            break
print(f'length of left save list: {len(left_save_list)}')

for cam in right_time_list:
    for ins in ins_time_list:
        if abs(int(cam) - int(ins)) < 10000:
            right_save_list.append(cam)
            break
print(f'length of right save list: {len(right_save_list)}')

for cam in rear_time_list:
    for ins in ins_time_list:
        if abs(int(cam) - int(ins)) < 10000:
            rear_save_list.append(cam)
            break
print(f'length of rear save list: {len(rear_save_list)}')
'''


'''
with open(front_cam, 'r') as file:
    for line in file:
        front_camera_time_list.append(line.split(' ')[0])

print(len(front_camera_time_list))


test = []
for cam in front_camera_time_list:
    for ins in ins_time_list:
        if abs(int(cam) - int(ins)) < 10000:
            test.append(cam)
            break

print(len(test))
'''