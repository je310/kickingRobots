import math
print "hello"
values= [0]*64
for i in range (0,64):
	values[i] = 100+math.floor(100*math.sin(2*math.pi*i/64))
print values
