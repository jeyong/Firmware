
import matplotlib.pyplot as plt 
import csv 
with open('../250Hz/log0011_speedchecker_info_0.csv') as csvfile:

    reader = csv.DictReader(csvfile)
    isstart = False 
    count = 0
    x = [] #sequence
    y = [] #increment
    for row in reader:
        cur_seq = int(row['sequence'])
        if isstart == False :
            if cur_seq == 1000 :
                isstart = True
                count = 1
        elif isstart == True:
            y.append(cur_seq)
            x.append(count)
            count = count + 1
            if count > (250 * 60 * 30): #250Hz * 60 (sec) * 30 min
                break

    plt.plot(x, y)
    plt.xlabel('topics count')
    plt.ylabel('sequence num')
    plt.title("Integrity Test")
    plt.show()
