import csv 
with open('../250Hz/log0011_speedchecker_info_0.csv') as csvfile:

    cur_seq = prev_seq = 0
    dummys = []
    for n in range(0, 87):
        dummys.append('dummy_data[' + str(n) + ']')
    
    reader = csv.DictReader(csvfile)
    for row in reader:
        for n in range(0, 87):
            if n != int(row[dummys[n]]):
                print('data inconsistent occured!!')                
                print('At dummy_data[' + str(n) + '] : ' + 'expected : ' + str(n) + ' but '+ str(row[dummys[n]]) )
                exit() 

        cur_seq = int(row['sequence'])
        if cur_seq != (prev_seq+1):
            print('expected : ' + str(prev_seq+1) + ' but cur_seq : '+ str(cur_seq) )
        prev_seq = cur_seq 
