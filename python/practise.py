#Check the number is an inter and entered is within the range

def check_input():

    choice = 'wrong'
    within_range = False

    while choice.isdigit() == False or within_range:     #Here initialy itself making the condition as True by using dummy assignment to variable to make while loop run continously

        choice = input("Please enter a number (0-10): ")  #Enter only string data for non-digit element

        if choice.isdigit() == False:
            print('Sorry that is not a digit!')

        if choice.isdigit() == True:
            if int(choice) in range(0,10):
                within_range = True
            else:
                print("hi")
                within_range = False

    return int(choice)

print(check_input())



            


