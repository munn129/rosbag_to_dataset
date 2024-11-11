
date_list = ['0519']
gap_list = ['200', '300', '400', '500']


for date in date_list:
    for gap in gap_list:
        input_dir = f'/media/moon/moon_ssd/moon_ubuntu/icrca/{date}/{date}_front_gt_{gap}.txt'

        output_dir = f'./icrca/imagenames_{date}_{gap}.txt'

        with open(input_dir, 'r') as input_file:

            for line in input_file:

                with open(output_dir, 'a') as output_file:
                    image_name = line.split(' ')[0]
                    output_file.write(f'{image_name}\n')