from numpy import *
a = array([2,3,4])
print a
b = linspace(0,10,12)
print "b:%r,\tb type:%r" % (b,b.dtype)
print b.reshape(2,6)
print shape(b.reshape(2,6))