#include <fstream>
#include <stdio.h>      /* printf */
#include <string.h>     /* strcat */
#include <stdlib.h>     /* strtol */
#include <iostream>
#include <bitset>
#include <sys/time.h>
#include <vector>
#include <math.h>
#include <time.h>
// own files
#include "binaryoctree.h"
#include "compareoctrees.h"






//def findChildren(tree,nid = 0,edges = [],lvl=0,maxDepth=16):
//    children = 0

//    if not len(tree) > 0:
//        return 0

//    for n in tree[0]:
//        if n == 3:
//            children += 1
//#            print 'edge[',nid,',',nid + children,']'
//            if lvl < maxDepth:
//                edges.append([nid,nid + children])
//                children += findChildren(tree[children:],nid + children,edges,lvl+1,maxDepth)
//    return children

int main( int argc, char *argv[] ){
   CompareOcTrees comp_bio;
    if (argc != 3){
        std::cout << "not in my house" << std::endl << "  usage: " << argv[0] << " <octomap.bt>  <octomap2.bt>" << std::endl;
    }else{
        clock_t overall_start;
        overall_start = clock();
        std::string filename = argv[1];
        std::string filename2 = argv[2];
        std::cout << filename.c_str() << std::endl;
        std::cout << filename2.c_str() << std::endl;
        std::ifstream file(filename.c_str(),  std::ios_base::in | std::ios_base::binary);
        std::ifstream file2(filename2.c_str(),  std::ios_base::in | std::ios_base::binary);

        clock_t cstart;
        cstart = clock();

        BinaryOctree bioctree_1(file);
        std::cout <<"reading first file " << float(clock()-cstart)/float(CLOCKS_PER_SEC) << std::endl << std::endl;

        BinaryOctree bioctree_2(file2);
        std::cout <<"reading both files " << float(clock()-cstart)/float(CLOCKS_PER_SEC) << std::endl << std::endl;

        CompareOcTrees compTrees;
        cstart = clock();
        std::vector<Subtree> subtrees_1 = compTrees.getSubtrees_iter(bioctree_1.getData(),bioctree_1.getResolution());
        std::cout <<"get the first Subtree took " << float(clock()-cstart)/float(CLOCKS_PER_SEC) << std::endl << std::endl;

        std::vector<Subtree> subtrees_2 = compTrees.getSubtrees_iter(bioctree_2.getData(),bioctree_2.getResolution());
        std::cout <<"get both Subtree took " << float(clock()-cstart)/float(CLOCKS_PER_SEC) << std::endl << std::endl;

         std::cout <<"number of subtrees found in tree 1 " << subtrees_1.size()  << std::endl;
         std::cout <<"number of subtrees found in tree 2 " << subtrees_2.size()  << std::endl;

         std::cout <<"comparing octrees " << std::endl;
         compTrees.compare(&bioctree_1,&subtrees_1,&bioctree_2,&subtrees_2);


//        tree.erase(tree.begin() + 2000,tree.end());
//        int childCount = comp_bio.getChildCount_rec(&tree,0);
//        std::cout <<"getChildCount took " << float(clock()-cstart)/float(CLOCKS_PER_SEC) << std::endl;
//        std::cout << "childCount " << childCount << std::endl;
        std::cout <<"read the octomap took " << float(clock()-overall_start)/float(CLOCKS_PER_SEC) << std::endl;

    }
}


