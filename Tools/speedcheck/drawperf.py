
import matplotlib.pyplot as plt 
import csv 
with open('../100Hz/log0003_speedchecker_info_0.csv') as csvfile:

    reader = csv.DictReader(csvfile)
    isstart = False 
    count = 0
    x = [] #sequence
    y = [] #increment
    prev_time = 0 
    for row in reader:
        cur_seq = int(row['sequence'])
        cur_time = int(row['curtime'])
        if isstart == False :
            if cur_seq == 1000 :
                isstart = True
                count = 1
                prev_time = cur_time

        elif isstart == True:
            y.append(round(float(cur_time - prev_time)/1000, 1))  #4ms time
            x.append(count)
            count = count + 1
            prev_time = cur_time
            if count > (100 * 60 * 30): #250Hz * 60 (sec) * 30 min
                break
    print(y)
    plt.plot(x, y)
    plt.xlabel('topics count')
    plt.ylabel('topics diff (ms)')
    plt.title("Performance Test")
    plt.show()
