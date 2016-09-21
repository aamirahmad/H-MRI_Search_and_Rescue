# -*- coding: utf-8 -*-
"""
Created on Thu Nov  5 16:11:44 2015

@author: eruff

"""


from splitOctree import getOrigins


tree = [[3, 3, 3, 3, 3, 3, 3, 3],
[0, 0, 0, 0, 0, 0, 0, 3],
[0, 0, 0, 0, 0, 0, 0, 3],
[0, 0, 0, 0, 0, 0, 0, 3],
[0, 0, 0, 0, 0, 0, 0, 3],
[0, 0, 0, 0, 0, 0, 0, 3],
[0, 0, 0, 0, 0, 0, 0, 3],
[0, 0, 0, 0, 0, 0, 0, 3],
[0, 0, 0, 0, 3, 0, 3, 3],
[0, 0, 0, 0, 0, 0, 3, 3],
[0, 0, 0, 0, 0, 0, 0, 3]]

tree = [[3, 3, 3, 3, 3, 3, 3, 3],
[0, 0, 0, 0, 3, 0, 0, 0],
[0, 0, 0, 0, 3, 0, 0, 0],
[0, 0, 0, 0, 3, 0, 0, 0],
[0, 0, 0, 0, 3, 0, 0, 0],
[0, 0, 0, 0, 3, 0, 0, 0],
[0, 0, 0, 0, 3, 0, 0, 0],
[0, 0, 0, 0, 3, 0, 0, 0],
[0, 0, 0, 0, 3, 3, 0, 3],
[0, 0, 0, 0, 3, 3, 0, 0],
[0, 0, 0, 0, 3, 0, 0, 0],
[0, 0, 0, 0, 1, 0, 0, 0],
[0, 0, 0, 0, 3, 3, 3, 3]]


#[0, 0, 0, 0, 3, 3, 0, 0],
#[0, 0, 0, 0, 3, 0, 0, 0]]
'''
coordinates  -819.2 -819.2 -819.2
coordinates  -409.6 -409.6 -409.6
coordinates  -204.8 -204.8 -204.8
coordinates  -102.4 -102.4 -102.4
coordinates  -51.2 -51.2 -51.2
coordinates  -25.6 -25.6 -25.6
coordinates  -12.8 -12.8 -12.8
coordinates  -6.4 -6.4 -6.4
coordinates  -9.600000000000001 -9.600000000000001 -3.2
coordinates  -11.200000000000001 -8.0 -1.6
'''
origin = [0,0,0]
depth = 0
res = 0.05

print (.9*10**-7 > 10**-4)


origins_root = getOrigins(tree[0],0,origin)

#print origins

origins =  []
origins.append(origins_root[-1])


for i in range(1,len(tree)):
    origin = origins[-1]
    node = tree[i]
    print (node,i)
    print (origin)
    print ('getOrigins',getOrigins(node,i,origin))
    
    origins.append(getOrigins(tree[i],i,origin))
    
    
    
    print ()

