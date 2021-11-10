#!/usr/bin/env python

# 1. Syntax

# class NameOfClass():

# 	def __init__(self, attribute_name1, param2):   #Init method for intialize data members, it will automatically calls whenever object for this class was created simillarly constructor in C++ and initializes DM to object
# 		self.attribute_name1 = attribute_name1             # assign the parameter to data member variable or attribute
# 		self.attribute_name2 = param2


# 	de some_method(self):   # Data function
# 	# perform some action
# 	print(self.attribute_name1)


# 2. Syntax for creating instace(object) for class
# class Boss():
# 	pass

# darshan = Boss()   #created a object called 'darshan' for Boss() class.


# 3. Syntax for creating atrribute(DM) for class
# class Dog():
# 	def __init__(self, breed):
# 		self.breed = breed     # most of the time programmers indicates both attribute and param with same name convention

# my_dog = Dog(breed= 'Lab')   #Creating object for class with passing param this param will assign to attribute defined in that class
# print(my_dog.breed)   #Acessing attribute using of class using object
# type.(my_dog)         # gives the class name of this object 


# 4. Creating multiple attribute for class
# class Dog():
	# species = 'mamal'           #This is the initialization of class object attribute it same for all object created for this class and no need to call as param while creating object
# 	def __init__(self, name, color, spots):
# 		self.name = name
# 		self.color = color
# 		self.spots = spots

# my_dog = Dog(name = 'Lab', color= 'Red', spots= False)
# print(my_dog.species, my_dog.name)
# your_dog = Dog(name= 'Pomerian', color= 'Black', spots= True)
#print(your_dog.species, your_dog.name)


# # 5 syntax for creating Method/functions/actions for class
# class Dog():
# 	def __init__(self, name, color):
# 		self.name = name
# 		self.color = color

# 	def bark(self):     #Method sysntax
# 		print(self.name, "barking as Woff!")   #Output: Pink barking as Woff!   This using class attribute here(self.name)

# 	def legs(self, number):
# 		print("How many legs can dog has: ", number)

# owner_dog = Dog(name = 'pinky', color= 'orange')
# owner_dog.bark()     #calling method using object
# owner_dog.legs(4)  # calling method using object and outside param 


# 6 Sytax for mehtod operation
# class Circle():
# 	pi = 3.14  #Class attribute
# 	def __init__(self, radius = 1):   #definging default attributes value
# 		self.radius = radius

# 	def get_circumference(self):
# 		return 2 * self.pi * self.radius   #The class attribute can be call using 2 ways either using self keyward like self.pi or using class name keyward like Circle.pi

# smallCircle = Circle()
# print("radius of small circle", smallCircle.get_circumference())   #It uses the default radius value
# bigCircle = Circle(30)
# print("radius of big circle", bigCircle.get_circumference())   #it uses the object passing param as radius value



## Inheritance and Polymorphism concept
## Inheritance is feature of using feature of parent in derived/child class. This will saves the time
## Syntax for inheritance
# class Animal():                                  #Parent class syntax
# 	def __init__(self):
# 		print ("Animal class initialized")

# 	def behavoir(self, behavoir):
# 		print("Behaviour of animal", behavoir)

# class Dog(Animal):             #Child class syntax
# 	def __init__(self):
# 		Animal.__init__(self)       #Has to initialize parent here also
# 		print("Dog class is initialized")

# 	def color(self, color):
# 		print("Color of dog is", color)

# 	def bark(self):
# 		print("Dog barking as Woff!")

# my_dog = Dog()
# my_dog.behavoir(behavoir = 'soft')
# my_dog.bark()
# my_dog.color(color = 'Red')