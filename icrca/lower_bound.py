from math import pi, sin, cos, atan2, sqrt

PI_DIV_180 = pi/180

def gps_to_meter(lat1, long1, lat2, long2) -> float:
    R = 6378.137 # radius of the earth in KM
    lat_to_deg = lat2 * PI_DIV_180 - lat1 * PI_DIV_180
    long_to_deg = long2 * PI_DIV_180 - long1 * PI_DIV_180

    a = sin(lat_to_deg/2)**2 + cos(lat1 * PI_DIV_180) * cos(lat2 * PI_DIV_180) * sin(long_to_deg/2)**2
    c = 2 * atan2(sqrt(a), sqrt(1-a))
    d = R * c
    
    return d * 1000 # meter

gap_list = ['200', '300', '400', '500']

for gap in gap_list:
    
    dataset_dir = f'/media/moon/moon_ssd/moon_ubuntu/icrca/0519/0519_front_gt_{gap}.txt'
    query_dir = f'/media/moon/moon_ssd/moon_ubuntu/icrca/0828/0828_front_gt_050.txt'

    dataset_list = []
    query_list = []

    with open(query_dir, 'r') as file:
        for line in file:
            query_list.append(line.split('\n')[0]) # without '\n'

    with open(dataset_dir, 'r') as file:
        for line in file:
            dataset_list.append(line.split('\n')[0]) # without '\n'

    lower_bound_sum = 0
    upper_bound_sum = 0
    idx = 0

    for query in query_list:
        
        lower_bound = 9999
        upper_bound = 0

        query_lat = float(query.split(' ')[1])
        query_lon = float(query.split(' ')[2])

        for dataset in dataset_list:
            
            dataset_lat = float(dataset.split(' ')[1])
            dataset_lon = float(dataset.split(' ')[2])

            distance = gps_to_meter(query_lat, query_lon, dataset_lat, dataset_lon)
            
            if distance < lower_bound:
                lower_bound = distance
            
            if distance > upper_bound:
                upper_bound = distance

        lower_bound_sum += lower_bound
        upper_bound_sum += upper_bound
        idx += 1

    print(f'{gap} lower bound: {lower_bound_sum/idx}')
    print(f'{gap} upper bound: {upper_bound_sum/idx}')

'''
case 1
050 lower bound: 1.6301831488280807
050 upper bound: 1076.521098478386
075 lower bound: 1.7011705516826126
075 upper bound: 1077.3452192291245
100 lower bound: 1.7839971205349365
100 upper bound: 1077.1412476047665
125 lower bound: 1.866460305627852
125 upper bound: 1074.570864316501
150 lower bound: 1.995439829119352
150 upper bound: 1081.2997993620775

case 2: dataset 050
075 lower bound: 1.6359232628959217
075 upper bound: 1077.348836032986
100 lower bound: 1.6385042237970313
100 upper bound: 1077.1448029265787
125 lower bound: 1.6243422831436478
125 upper bound: 1074.6485740477037
150 lower bound: 1.6390546402587445
150 upper bound: 1081.3101776738724

case 3: query 050
075 lower bound: 1.6946284775630838
075 upper bound: 1076.5175003109184
100 lower bound: 1.7777528344113933
100 upper bound: 1076.5175140727836
125 lower bound: 1.8729542059492506
125 upper bound: 1076.4452932119545
150 lower bound: 1.9797875803720522
150 upper bound: 1076.5107151883637
'''