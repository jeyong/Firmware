
import matplotlib.pyplot as plt 
import csv 
with open('../250Hz_old/log0004_speedchecker_info_0.csv') as csvfile:

    reader = csv.DictReader(csvfile)
    isstart = False 
    count = 0
    prev_seq = 0
    x = [] #sequence
    y = [] #increment
    for row in reader:
        cur_seq = int(row['sequence'])
        if isstart == False :
            if cur_seq == 1000 :
                isstart = True
                count = 1
        elif isstart == True:
            if (prev_seq%1000) > cur_seq :
                y.append(cur_seq + (1000 - (prev_seq%1000)) - 1)
                print("cur_seq : "+str(cur_seq) + "prev_seq : " + str(prev_seq))
            else :
                y.append(cur_seq - (prev_seq%1000) - 1)
            x.append(count)
            count = count + 1
            if count > (250 * 60 * 30): #250Hz * 60 (sec) * 30 min
                break
        prev_seq = cur_seq

    plt.plot(x, y)
    plt.xlabel('topics count')
    plt.ylabel('sequence num')
    plt.title("Integrity Test")
    plt.show()
