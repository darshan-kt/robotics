# listing even and odd integers

odd = [1,3,5,7,9]
even = [2,4,6,8]

for oddnum, evennum in zip(odd,even):
	print(oddnum)
	print(evennum)


#sorting function with for loop
l = [1,5,8,3,0,6]
lsorted = []
for i in sorted(l):
	print(i)
	lsorted.append(i)
print(lsorted)
