import math

trajectory_dir = '/media/moon/T7 Shield/naverlabs_datasets/GangnamStation_B2/GangnamStation_B2_release_mapping/GangnamStation/B2/release/mapping/sensors/trajectories.txt'
save_dir = '/media/moon/T7 Shield/naverlabs_undistort'

dataset_date = '2019-12-03_10-54-54'
query_date = '2019-12-03_11-43-39'

def q_to_e(w, x, y, z):
    w = float(w)
    x = float(x)
    y = float(y)
    z = float(z)
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x**2 + y**2))
    t_2 = 2 * (w * y - z * x)
    t_2 = 1 if t_2 > 1 else t_2
    t_2 = -1 if t_2 < -1 else t_2
    pitch = math.asin(t_2)

    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y**2 + z**2))
    r_to_d = 180 / math.pi
    return roll * r_to_d, pitch* r_to_d, yaw * r_to_d

def main():

    trajectory_list_dataset = []
    trajectory_list_query = []
    

    with open(trajectory_dir, 'r') as file:
        for line in file:
            if line[0] == '#': continue

            line = line.split('\n')[0]

            splited_line = line.split(', ')

            if splited_line[1] == '40027089_01':
                roll, pitch, yaw = q_to_e(splited_line[2], splited_line[3], splited_line[4], splited_line[5])
                sentence = f'{dataset_date}/{splited_line[0]}.jpg {roll} {pitch} {yaw} {splited_line[6]} {splited_line[7]} {splited_line[8]}'
                # print(roll, pitch, yaw)
                trajectory_list_dataset.append(sentence)

            if splited_line[1] == '40027089_28':
                roll, pitch, yaw = q_to_e(splited_line[2], splited_line[3], splited_line[4], splited_line[5])
                sentence = f'{dataset_date}/{splited_line[0]}.jpg {roll} {pitch} {yaw} {splited_line[6]} {splited_line[7]} {splited_line[8]}'
                # print(roll, pitch, yaw)
                trajectory_list_query.append(sentence)

    # print(len(trajectory_list_dataset))
    # print(len(trajectory_list_query))

    with open(f'{save_dir}/dataset_gt.txt', 'a') as file:
        for line in trajectory_list_dataset:
            file.write(line + '\n')
        
    with open(f'{save_dir}/query_gt.txt', 'a') as file:
        for line in trajectory_list_query:
            file.write(line + '\n')

if __name__ == '__main__':
    main()