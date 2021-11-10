myfile = open('/home/darshan_k_t/skillup/python/dar.txt')
contents = myfile.read()
print contents
# print myfile.read()
print myfile.seek(0)    #reset file cursor location
print myfile.readline()

with open("/home/darshan_k_t/skillup/python/dar.txt") as my_new_file:    #change file name
    content = my_new_file.read()
    print content
    
with open('/home/darshan_k_t/skillup/python/my_new_file', mode='a') as f:  
    f.write("\n Add BOSS is back!")  #added new line
   
with open('/home/darshan_k_t/skillup/python/my_new_file', mode='r') as f:    
    print (f.read())   # reading the added file