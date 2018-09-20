filename = 'experimental_results_simple_goal_complexity_lvl3.txt'

lines = []
reader = open(filename, 'r')
for line in reader:
    lines.append(line)
    if 'Time taken with zooming' in line:
        zoom_time = float(line[30:]) + 60*float(line[27:29])
    if 'Time taken without zooming' in line:
        if 'TIMEOUT' in line: non_zoom_time = float(1200)
        else: non_zoom_time = float(line[33:]) + 60*float(line[30:32])
        ratio = non_zoom_time/zoom_time
        lines.append('Ratio: ' + str(ratio) + '\n')
reader.close

writer = open(filename, 'w')
for line in lines: writer.write(line)
writer.close()
