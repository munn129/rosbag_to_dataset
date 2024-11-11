from random import randint

remove_iter = 44900
max = 19000 + 1 # 0828
max = 18966 + 1 # 0519

gap = '500'

input_file = '/media/moon/moon_ssd/moon_ubuntu/icrca/0828/0828_front_gt.txt'
input_file = '/media/moon/moon_ssd/moon_ubuntu/icrca/0519/0519_front_gt.txt'

output_file = f'{input_file[:-4]}_{gap}.txt'

remove_index = []
for _ in range(remove_iter):
    
    random_int = randint(1, max)
    
    if random_int not in remove_index:
        remove_index.append(random_int)

    else:
        while random_int not in remove_index:
            random_int = randint(1, max)

        remove_index.append(random_int)

remove_index.sort()

file_list = []

with open(input_file, 'r') as file:
    for line in file:
        file_list.append(line)

with open(output_file, 'w') as file:
    file.write('')

for idx, val in enumerate(file_list):
    
    if idx not in remove_index:
        with open(output_file, 'a') as file:
            file.write(f'{val}')

from math import pi, sin, cos, atan2, sqrt

def gps_to_meter(lat1, long1, lat2, long2) -> float:
    R = 6378.137 # radius of the earth in KM
    lat_to_deg = lat2 * pi/180 - lat1 * pi/180
    long_to_deg = long2 * pi/180 - long1 * pi/180

    a = sin(lat_to_deg/2)**2 + cos(lat1 * pi/180) * cos(lat2 * pi/180) * sin(long_to_deg/2)**2
    c = 2 * atan2(sqrt(a), sqrt(1-a))
    d = R * c
    
    return d * 1000 # meter

past_gps = 0,0
idx = 0
sum = 0

with open(output_file, 'r') as file:
    for line in file:
        idx += 1
        lat = float(line.split(' ')[1])
        lon = float(line.split(' ')[2])
        
        if idx == 1:
            past_gps = lat, lon
            continue

        sum += gps_to_meter(past_gps[0], past_gps[1], lat, lon)
        past_gps = lat, lon

print(f'average gap: {sum/idx}')