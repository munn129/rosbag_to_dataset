import os
os.chdir(os.path.dirname(os.path.abspath(__file__)))

from math import pi, sin, cos, atan2, sqrt


def gps_to_meter(lat1, long1, lat2, long2) -> float:
    R = 6378.137 # radius of the earth in KM
    lat_to_deg = lat2 * pi/180 - lat1 * pi/180
    long_to_deg = long2 * pi/180 - long1 * pi/180

    a = sin(lat_to_deg/2)**2 + cos(lat1 * pi/180) * cos(lat2 * pi/180) * sin(long_to_deg/2)**2
    c = 2 * atan2(sqrt(a), sqrt(1-a))
    d = R * c
    
    return d * 1000 # meter

def main():
    # 0519/front/1432044414730300.png 51.7605932001 -1.2612594905 1.7848552051
    # file_dir/name lat lon heading
    gt_dir = '/media/moon/moon_ssd/moon_ubuntu/icrca/0519/0519_front_gt.txt'
    gt_dir = '/media/moon/moon_ssd/moon_ubuntu/icrca/0828/0828_front_gt.txt'


    past_gps = 0,0
    idx = 0
    sum = 0

    with open(gt_dir, 'r') as file:
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

if __name__ == '__main__':
    main()