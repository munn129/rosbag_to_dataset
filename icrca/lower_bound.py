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

gap_list = ['050', '075', '100', '125', '150']

for gap in gap_list:
    
    dataset_dir = f'/media/moon/moon_ssd/moon_ubuntu/icrca/0519/0519_front_gt_{gap}.txt'
    query_dir = f'/media/moon/moon_ssd/moon_ubuntu/icrca/0828/0828_front_gt_{gap}.txt'

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

    print(f'{gap} lower bound: {lower_bound/idx}')
    print(f'{gap} upper bound: {upper_bound/idx}')