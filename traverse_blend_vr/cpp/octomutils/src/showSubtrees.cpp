
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
#include<ctime>
#include <sstream>

// include octomap header
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap/AbstractOcTree.h>

// own files
#include "binaryoctree.h"
#include "compareoctrees.h"



using namespace std;

int main( int argc, char *argv[] ){

    CompareOcTrees comp_bio;
    if (argc != 2){
        std::cout << "not in my house" << std::endl << "  usage: " << argv[0] << " <octomap.bt> " << std::endl;
        return 1;
    }
    clock_t overall_start,cstart;
    overall_start = clock();
    std::string filename = argv[1];
    std::cout << filename.c_str() << std::endl;
    std::ifstream file(filename.c_str(),  std::ios_base::in | std::ios_base::binary);



    BinaryOctree bioctree(file);
    std::cout <<"reading first file " << float(clock()-cstart)/float(CLOCKS_PER_SEC) << std::endl << std::endl;


    CompareOcTrees compTrees;
    cstart = clock();
    std::vector<Subtree> subtrees = compTrees.getSubtrees_iter(bioctree.getData(),bioctree.getResolution());
    std::cout <<"get the first Subtree took " << float(clock()-cstart)/float(CLOCKS_PER_SEC) << std::endl << std::endl;


    cout << "found " << subtrees.size() << " subtrees" << endl;
    for(std::vector<Subtree >::iterator subT_it = subtrees.begin(); subT_it != subtrees.end(); ++subT_it){
        (*subT_it).printSubTree();
    }

    return 0;
}
