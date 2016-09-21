# -*- coding: utf-8 -*-
"""
Created on Fri Aug  7 15:03:54 2015

@author: eruff

"""
from numpy import random
from time import time
import sys

import subprocess

try:
    from octomap import  *
except ImportError as err:
    print("Could not find python-octomap")
    raise err

global filname 
filename =  b'/home/eruff/workspace/arhms_telecmt/src/telekyb/packages/telekyb_users/tk_eruff/tk_eruff/maps/pillar_map.bt'    
filename =  b'/home/eruff/workspace/arhms_telecmt/src/telekyb/packages/telekyb_users/tk_eruff/tk_eruff/maps/fr_078_tidyup.bt'


global REQUIRED_VERSION
global CURRENT_VERSION
REQUIRED_VERSION = (3,0)
CURRENT_VERSION = sys.version_info





## @brief calculates the resolution at a certain depth
#    
#    @param res tree resolution
#    @param depth depth for which the leaf size should be determined
#    
#    @return leaf size at certain depth

def calcSize(res,depth,maxDepth = 16):
    
    return float(res*2**(maxDepth-depth))
    
    


## reads a binary octree from file
#     
#     @param filename binary ocree file
#     
#     @return the binary representation of the octree
def readOctreeFromFile(filename):   

    tree = []
    with open(filename, "rb") as f:
        # read header
        for i in range(7):
            print (f.readline(),)
        start = time()
        byte = f.read(2)
        count = 0
        while byte != b'':

#            child_1to4 = ''.join('{0:08b}'.format(x, 'b') for x in byte)[:8]
#            child_5to8 = ''.join('{0:08b}'.format(x, 'b') for x in byte)[8:]

            count += 1
            # Do stuff with byte.
            # check the python interpreter version because of the switch from 2.xx to 3.xx
            if CURRENT_VERSION < REQUIRED_VERSION:
                node = ''.join('{0:08b}'.format(ord(x), 'b') for x in byte)        
            elif CURRENT_VERSION >= REQUIRED_VERSION:
                node = ''.join('{0:08b}'.format(x, 'b') for x in byte)        
                
            
            n = 2
            int_node = [int(node[i:i+n],2) for i in range(0, len(node), n)]

#            child_1to4 = int_node[:4]
#            child_5to8 = int_node[4:]
#            child_1to4.reverse()
#            child_5to8.reverse()            
#            int_node = child_1to4 + child_5to8
            
#            if count < 17:
#                print int_node
#                count += 1
            tree.append(int_node)
            byte = f.read(2)   
        print("the while loop took ", time() - start)
        f.close()    
    return tree

##Write header of binary octomap
#
# @param f filepointer opened in wb
def writeBinHeader(f,numNodes):
    header = [b'# Octomap OcTree binary file\n',
              b'# (feel free to add / change comments, but leave the first line as it is!)\n',
              b'#\n',
              b'id OcTree\n',
              b'size '+str.encode(str(numNodes))+b'\n',
              b'res 0.1\n',
              b'data\n']
              
    for line in header:
        f.write(line)
    



## write binary data to ocotmap file
#    
# @param data binary data for octomap
# @param filename ...

def writeBinaryOctree(data,numNodes,filename):
    with open(filename, "wb") as f:
        ## Write the default header with the number of nodes provided
        writeBinHeader(f,numNodes)
        for byte in data:
            txt = b''.join('{:02b}'.format(x, 'b') for x in byte)
            byte1 = 0
            byte2 = 0
            
            for bit,n in zip(list(reversed(txt[:8])),range(8)):
                if int(bit) == 1:
                    byte1 += 2**n                    
            for bit,n in zip(list(reversed(txt[8:])),range(8)):
                if int(bit) == 1:
                    byte2 += 2**n   

#            f.write(hex(byte1))
#            f.write(hex(byte2))
            f.write(chr(byte1))
            f.write(chr(byte2))
            

##Identiefies the subtrees of a binary octomap octree data
#
# Given a binary representation of an octomap octree this function finds the eight subtrees of its root
# 
# @param tree the binray representation of an octomap octree data
# @param edges list of tuples (in,out) defining the trees structure

def getSubtrees(tree,edges):
    subtrees = [[0],[0],[0],[0],[0],[0],[0],[0]]
    
    i = -1
    for e in edges:
        if e[0] == 0:
            i += 1
        subtrees[i].append(e[1])
    return subtrees

def findSubtrees(tree,treeptr = 0,depth=0,sID=0,subTree_idx = [[],[],[],[],[],[],[],[]],origin=[0.,0.,0.]\
                            ,size = calcSize(0.1,0,16),maxDepth =16):
    numChildNodes = 0

    # return on empty tree
    # break condition of this revursive function    
    if len(tree) == 0:
        return 0

    

    for n in tree[0]:
        if int(n) == 3:
            numChildNodes += 1
#            print sID,treeptr+numChildNodes
            subTree_idx[sID].append(treeptr+numChildNodes)
            # since the tree is ordered in a depth first manner we need to remove all child subTree_idx
            # to get an accurate child counter
            numChildNodes += findSubtrees(tree[numChildNodes:],treeptr + numChildNodes,depth+1,sID,subTree_idx,origin)
        if depth == 0:
            ## Add the origin and size as last node to the subtree
            subTree_idx[sID] += [[calcOrigin(0.1,depth,origin,size)[sID],size/2]]
            sID += 1
#            break
    return numChildNodes

def moreSubtrees(tree,subtrees_idx,subtree_position,size):
    ## List of Trees covering the specified size    
    treeOfSize = []

    if size < 13:
        treeOfSize.append(tree + [[subtree_position,size]])
    else:
        ## @todo the root is cut off maybe we need to implement a check to prevent data loss
        findSubtrees(tree,subTree_idx=subtrees_idx,origin=subtree_position,size=size)
        
        for subtree_idx in subtrees_idx:
            subtree = [tree[j] for j in subtree_idx[:-1] if j < len(tree)]
            subtree_position,size = subtree_idx[-1]
            if len(subtree) > 0:
                subtrees_idx = [[],[],[],[],[],[],[],[]]
                treeOfSize += moreSubtrees(subtree,subtrees_idx,subtree_position,size)
                
    return treeOfSize

## Get the number of nodes in a tree
# 
# @param tree The int list representation of a tree 
def calcNumNodes(tree):
    numNodes = 0
    for node in tree:
            for n in node:
                if int(n) != 0:
                    numNodes += 1
    # all nodes plus the root
    return numNodes+1


      
        
## Tests the interpretaion of the octomap octree
#
# Reads a binary octomap tree, generates subtrees, puts the subtrees back together and write the result to file. 
# Then the input and output files are compared with diff
#
# This function tests the following
# * Depth First interpretation of the binary octree
# * calNumNodes() The function to determine the number of nodes within a octree
# * writeBinaryOctree() The function to write a binary octree representation to file
# * Subsequently also the readOctreeFromFile() function.
#
# @param inputFile tree file to read from
# @param tmpFile output file (default '/tmp/octomapTest.bt')
#
# @return diff output When diff finds no difference output is empty
# @return diff error When there are no errors this return value is empty
def testTreeInterpretation(inputFile,tmpFile = '/tmp/octomapTest.bt'):
    
    ## Read the Octree from file    
    tree = readOctreeFromFile(inputFile)    

    subtrees_idx = [[],[],[],[],[],[],[],[]]
    ## Finding the node indicies in the subtrees
    findSubtrees(tree,subTree_idx = subtrees_idx)
    ## Build the new tree starting with the old root (which is not considered a part of a subtree)
    newTree = [tree[0]]
    for subtree_idx,i in zip(subtrees_idx,range(len(subtrees_idx))):
        ## Generate the actual subtrees from the subtree indicies
        subtree = [tree[j] for j in subtree_idx if j < len(tree)]
        ## Concatinate the subtree to the new tree. This reflects the depth first approach
        newTree += subtree
        
    sub
        
    ## Get the number of nodes in the new tree
    numNodes = calcNumNodes(newTree)
    print ('the new tree has ',numNodes, 'and the old tree has ', calcNumNodes(tree))
    ## Write the new tree as binary octree file with the newly calculated number of nodes
    writeBinaryOctree(newTree,numNodes,tmpFile)

    
    ## The command to compare the output and the input file
    cmd = 'diff ' + str(tmpFile) + ' ' + str(inputFile)
    output,error = subprocess.Popen(cmd, shell=True, executable="/bin/bash", stdout=subprocess.PIPE, stderr=subprocess.PIPE).communicate()
    return output,error



## Check if the node is a leaf
#
# If the node can be considrered a leaf node it can not have any outgoing branches
#
# @param node The reference node
#
# @return boolean representing either the node is a leaf or not
def isLeafNode(node):
    return all([(int(x) != 3) for x in node])

## Get the number of outgoing branches for a given node
# 
# @param node The reference node
#
# @return The number of outgoing branches     
def getBranchCount(node):
    return len([x for x in node if int(x) == 3])


##calculates the position of a node 
#
#    Given a tree resolution and the parents position this function calculates 
#    the positions of the child nodes in a certain depth
#    
#    @param res resolution of the entire tree
#    @param current_depth the level for which the position should be calculated
#    @param origin the position of the parent node
#    @param size The size covered by the parent
#    
#    @return array of all eight child positions

def calcOrigin(res,current_depth,origin,size):
    
   
    # array defining the order in which children are sorted in octomap
    xyz = np.array([[ -1, 1,-1, 1,-1, 1,-1,1],
                    [ -1,-1, 1, 1,-1,-1, 1,1],
                    [ -1,-1,-1,-1, 1, 1, 1,1]])

    xyz = np.array([[-1, 1, 1,-1,-1, 1, 1,-1],
                    [-1,-1, 1,-1,-1,-1, 1, 1],
                    [-1, 1, 1, 1, 1,-1,-1,-1]])

    xyz = np.zeros((8,3))
    xyz[:] = np.array([[-1,-1,-1],
                        [1,-1,-1],
                        [-1,1,-1],
                        [1,1,-1],
                        [-1,-1,1],
                        [1,-1,1],
                        [-1,1,1],
                        [1,1,1]])
    xyz[:] = np.array([[1,1,-1],[-1,1,-1],[1,-1,-1],[-1,-1,-1],
                      [1,1,1],[-1,1,1],[1,-1,1],[-1,-1,1]])
    
    # quater the resolution
    r4 = size*.25
    # init np array for all eight children
    child_positions = np.zeros(shape=(8,3))
    # get the position for all eight children
    for child,i in zip(xyz,range(8)):
        # calc x y and z according to the order adding or subtracting res/4 depending on the childs position
        child_positions[i] =  np.array([y+r4*x for x,y in zip(child,origin)])
        
    # return the array whith all child positions
    return child_positions



## Get the origins of the children of the given node
#
# All origins of the children for this node are calculated, and only those for which a child node 
# exists are returned. Since the tree is traveres in a left depth first manner and accessed in a
# last in first out fashion the list of origins need to be reversed
# 
#
# @param node To get the child origins for
# @param depth The depth in which the node is placed
# @param origin The position of the node
# @return the reveresd list of origins
#
def getOrigins(node,depth,origin):
    res = .05
    
    origins = calcOrigin(res,depth,origin,calcSize(res,depth))
    origins_1to4 = origins[:4]
    origins_5to8 = origins[4:]
    # bytes are read from right to left thats why the origins need to be switched like that
    b = [list(origins_5to8[k]) for k in [i for i, val in enumerate(node[4:]) if int(val) == 3]] + \
            [list(origins_1to4[k]) for k in [i for i, val in enumerate(node[:4]) if int(val) == 3]]
    

#    a.reverse()
    return b

## Get The subtrees not using recursion rather iteration
#
# this is much faster ans scales way better then recursion
# @param tree
# @return list of subtrees with start idx in tree, depth, origin and size    
def getSubtreesIter(tree):
    if len(tree) < 2:
        return 
    
    subTrees = []        
    branchCounter = 0
    depth = 0 
    subTreePointer = 0
    ## loose the root
    root = tree[0]
    subTreeOrigins = getOrigins(root,0,[0.,0.,0.])
    for sto in subTreeOrigins:
        print(sto)
    tree = tree[1:]
    

    depthAtBranch = []    
    origins = []
    subTree = 0
    for node,i in zip(tree,range(len(tree))):

        if depth == 10:

            subTreePointer = i+1
            subTrees.append([subTreePointer,depth,origins[-1],calcSize(.05,depth),0])
#        print 'branchCounter',branchCounter
#        print 'depth', depth
        ## When the subTreeCounter is a lvl 0 a new sub tree starts
        if branchCounter == 0:

            if len(subTrees) > 0:
                subTrees[-1][-1] = i 
            
            origins= [list(subTreeOrigins[subTree])]
            subTree += 1
            depth = 1
            depthAtBranch.append(1)
            branchCounter = 1
            subTreePointer = i+1
            #subTrees.append((subTreePointer,depth))


            
        ## If the node is a leaf node a branch ends and we reduce the branchCounter by one

        if isLeafNode(node):
            branchCounter -= 1
            
            depth = depthAtBranch.pop()

            
        else:
            ## If the node is an inner node at least one branch is added
            last_origin = origins.pop()
            origins += getOrigins(node,depth,last_origin)
            if i < 12:
                print (node,depth,last_origin)
                print (getOrigins(node,depth,last_origin))
                print ()
            #print origins[-1]
            branchCount = getBranchCount(node)-1
            branchCounter += branchCount
            if branchCount > 0:
                depthAtBranch += [depth+1]*branchCount
            depth += 1
            
    ## return the start idx for the subtrees       
    return subTrees       
     
## Main Function
if __name__ == "__main__":
    if(True):
        if(CURRENT_VERSION >= REQUIRED_VERSION):
            filename = str.encode(sys.argv[1])
        else:
            filename = str(sys.argv[1])
    #    start = time()
    #    print 'Test the tree functions ',testTreeInterpretation(filename)[0]
    #    print("testTreeInterpretation took ",time()-start)
        # result is good
    #
        ocTree = OcTree(.1)
        start = time()
        print("read octree from ",filename)
        ocTree.readBinary(filename)
        print("read OcTreeFromFile took ",time()-start)
    #    
    #    print("number of nodes in the Tree",ocTree.calcNumNodes())
    #    print("ocTree Resolution ",ocTree.getResolution()) 
        start = time()
        tree = readOctreeFromFile(filename)    
        print("read OcTreeFromFile took ",time()-start)
        tmpfile = '/tmp/testOctomap2.bt'
        
    
        print ("node 38",tree[9])
        print ("node 38",tree[38])
    #
    #    ###############################33
    #    # calc position test    
    #
    ##    current_position = [0.,0.,0.]
    ##    for i in range(1,16):
    ##        if i == 1:
    ##            c=3
    ##        else:
    ##            c=5
    ##        current_position = calcOrigin(0.1,i,current_position)[c]
    ##        print current_position,i
    #    # calc position test END
    ##    start = time()
    ##    buildPyGraph(tree,maxDepth=16) 
    ##    print("building the graph took ",time()-start)
    #    buildPyGraph(tree,maxDepth=16) 
    #    edges = []
        start = time()
        subTrees = getSubtreesIter(tree)
        print("getSubtreesIter took ",time()-start)
    #    print (subTrees[:4])
#        print ("number of subtrees ", len(subTrees))
#        for sT in subTrees:
#            print (sT)
    
    
#        for t in tree[:11]:
#            print(t)
            
        
    
        '''
            
        start = time()
        tmpfile = b'/tmp/testOctomap'
        # because of some python 3 conversion trouble for now it is only possible to write with python 2.xx
        if CURRENT_VERSION < REQUIRED_VERSION:
            for x,y,i in zip(subTrees[:-1],subTrees[1:],range(len(subTrees[1:]))):
                if x[1] != 1:
        #            print x[0],y[0], x[2],x[3]
        #            print tmpfile + str(i)  + '.bt'
                    subTree = tree[x[0]:y[0]]
                    
                    writeBinaryOctree(subTree,calcNumNodes(subTree),tmpfile + str.encode(str(i)) +b'.bt')
            ## write the last tree
            if len(subTrees[-1]) > 2:
                i += 1
                x,depth,o,s = subTrees[-1]
                subTree = tree[x:]
                writeBinaryOctree(subTree,calcNumNodes(subTree),tmpfile + str.encode(str(i)) +b'.bt')
                
            print("writing ",i+1,"OcTreeFromFiles took ",time()-start)    
         '''   
    #    
    #    start = time()
    #    subtrees = moreSubtrees(tree,subtrees_idx,origin,0.1*2**16)
    #    print("finding subtrees took ",time()-start)
    ##    print foundTreeCluster
    #    print 'foundTreeCluster',len(subtrees)
    #    tmpfile = '/tmp/testOctomap'
    #    for subtree,i in zip(subtrees,range(len(subtrees))):
    #        ## Add the subtree number to the file
    #        print tmpfile + str(i)  + '.bt'
    #        writeBinaryOctree(subtree[:-1],calcNumNodes(subtree[:-1]),tmpfile + str(i) +'.bt')
    #        print("writing OcTreeFromFile took ",time()-start)    
        
            ## Get the number of nodes in the subtree
    #    numNodes = calcNumNodes(subtree)
    #        ## position of subtree kids
    ##        print 'origin of kids', calcOrigin(0.1,1,current_position)
    ##        print 'numNodes of newTree is',numNodes
    ##        ## Write the subtree to file
    ##        start = time()
    ##        ## Add the subtree number to the file
    ##        print tmpfile + str(i)  + '.bt'
    ##        writeBinaryOctree(subtree,numNodes,tmpfile + str(i) +'.bt')
    ##        print("writing OcTreeFromFile took ",time()-start)    
    #
    ##    print("find children took ",time()-start)
    ##    print edges
    ##    print getSubTrees(tree,edges)
    ##    subtree = subtrees[7]
    ##    print 'subtrees',subtree
    ##    print 'len tree',len(tree)
    #
    #        
    ##    buildPyGraph(list( tree[h] for h in subtree if h < len(tree)))
    ##    for subtree in subtrees:
    ##        buildPyGraph(list( tree[h] for h in subtree if h < len(tree)))
    