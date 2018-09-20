import numpy

ratio_list = []
results = open('experimental_results_simple_goal_complexity_lvl3.txt', 'r')
for line in results:
    if 'Ratio' in line: ratio_list.append(float(line[7:].strip('\n')))
results.close()

ratio_array = numpy.array(ratio_list)
mean = numpy.mean(ratio_array, axis=0)
std_dev = numpy.std(ratio_array, axis=0)
print ('MEAN:')
print (mean)
print ('STD DEV:')
print (std_dev)
