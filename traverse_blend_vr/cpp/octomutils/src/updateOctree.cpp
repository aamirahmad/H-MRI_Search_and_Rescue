
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
#include <ctime>
#include <sstream>
#include <cstdlib>

// include octomap header
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap/AbstractOcTree.h>

// own files
#include "binaryoctree.h"
#include "compareoctrees.h"


using namespace std;

using namespace octomap;


std::vector<int> RandomSamplingWithoutReplacement(std::vector<int>& vData, int N){
    // Random generator
    std::random_device rd;
    std::mt19937 randomGenerator(rd());

    int max = static_cast<int>(vData.size()-1);
    std::vector<int> vResult;

    for (int n=1; n<=N; n++) {
        std::uniform_int_distribution<> uniformDistribution(0,max);
        int index = uniformDistribution(randomGenerator);
        std::swap(vData[index],vData[max]);
        vResult.push_back(vData[max]);
        max--;
    }

    return vResult;
}


void updateLeaf(bitset<8> &node_1to4, bitset<8> &node_5to8){
    for (size_t i =0; i < 8; i += 2){

        if((node_1to4[i] == 0 && node_1to4[i+1] == 1)){
            node_1to4[i] = ~node_1to4[i];
            node_1to4[i+1] = ~node_1to4[i+1];
        }
        if((node_5to8[i] == 0 && node_5to8[i+1] == 1)){
            node_5to8[i] = ~node_5to8[i];
            node_5to8[i+1] = ~node_5to8[i+1];
        }
    }
}



int main( int argc, char *argv[] ){

    if (argc != 4){
        std::cout << "not in my house" << std::endl << "  usage: " << argv[0] << " <octomap.bt> <update_factor[0-1]> <#update_subtrees>" << std::endl;
        return 1;
    }

    clock_t cstart;
    cstart = clock();

    std::srand(std::time(0)); // use current time as seed for random generator
    std::string filename = argv[1];
    float update_factor = (float) std::atof(argv[2]);
    int update_subtree_count = (int) std::atoi(argv[3]);

    std::cout << filename.c_str() << std::endl;
    std::ifstream file(filename.c_str(),  std::ios_base::in | std::ios_base::binary);
    std::cout <<"reading first file " << float(clock()-cstart)/float(CLOCKS_PER_SEC) << std::endl << std::endl;


    BinaryOctree bioctree(file);

    CompareOcTrees compTrees;
    cstart = clock();
    std::vector<Subtree> subtrees = compTrees.getSubtrees_iter(bioctree.getData(),bioctree.getResolution());
    std::cout <<"get the first Subtree took " << float(clock()-cstart)/float(CLOCKS_PER_SEC) << std::endl << std::endl;

    std::vector<int> v(subtrees.size()) ; // vector with 100 ints.
    std::iota (std::begin(v), std::end(v), 0);

    vector<int> update_subtrees_idx;
    update_subtrees_idx = RandomSamplingWithoutReplacement(v,update_subtree_count);

    std::sort(update_subtrees_idx.begin(),update_subtrees_idx.end());

    cout << "[";
    for(vector<int>::iterator it = update_subtrees_idx.begin();it != update_subtrees_idx.end(); it++){
        cout << (*it) << ",";
    }
    cout << "]" << endl;


    std::bitset<8> node_1to4;
    std::bitset<8> node_5to8;

    bool update = false;
    int leaf_count = 0;
    int update_count = 0;

    for(vector<int>::iterator jt = update_subtrees_idx.begin(); jt != update_subtrees_idx.end();jt++){
        int subtreeBegin = subtrees[(*jt)].getSubTreePointer();
        int subtreeEnd = min(subtrees[(*jt)].getSubTreeEndPointer(),(int)bioctree.getData()->size());

        cout << subtreeBegin << " " << subtreeEnd << endl;

        for(vector<bitset<8> >::iterator it=bioctree.getData()->begin()+subtreeBegin; it!=bioctree.getData()->begin()+subtreeEnd;++it){
            node_1to4 = *it; // store the current node in the node varialble
            it++; // advance the iterator once more because one node is stored not in 8 but 16 bits
            node_5to8 = *it; // store the current node in the node varialble

            if(CompareOcTrees::isLeafNode(node_1to4,node_5to8)){
                //            cout << node_1to4 << " " << node_5to8 << endl;
                leaf_count += 1;
                int random_variable = std::rand();
                update = ((float) random_variable)/float(RAND_MAX) > 1-update_factor? true: false;
                if(update){
                    update_count += 1;
                    updateLeaf(node_1to4,node_5to8);
                }
                //            std::cout << "Random value on [0 " << RAND_MAX << "]: "
                //                      << update << '\n';
            }
            it--;
            *it = node_1to4;
            it++;
            *it = node_5to8;


        }
    }



    cout << "update_count " << update_count<< " leaf_count " << leaf_count << endl;
    cout << "update rate " << (float)update_count/(float)leaf_count << endl;

    vector<unsigned char> data_uchar;
    data_uchar.reserve(bioctree.getData()->size());

    data_uchar.clear();

    clock_t cconversion = clock();
    for(std::vector<bitset<8> >::iterator bioctree_it = bioctree.getData()->begin(); bioctree_it != bioctree.getData()->end(); ++bioctree_it) {
        data_uchar.push_back(static_cast<u_int8_t> ((*bioctree_it).to_ulong() ));
    }
    std::cout <<"conversion of the complete three " << float(clock()-cconversion)/float(CLOCKS_PER_SEC) << std::endl << std::endl;


    std::stringstream datastream_uchar_complete_tree;
    datastream_uchar_complete_tree.write((const char*) &data_uchar[0] ,data_uchar.size());
    octomap::OcTree* octree_complete = new octomap::OcTree(bioctree.getResolution());
    octree_complete->readBinaryData(datastream_uchar_complete_tree);
    std::stringstream name;

    filename = filename.substr(filename.find_last_of("/")+1, string::npos);
    filename.erase(filename.find_last_of("."),string::npos);

    name << "/tmp/" << (int) update_subtrees_idx.size() << "_updated_" << filename.c_str()<< ".bt";
    //    cout << name.str() << endl;
    octree_complete->writeBinary(name.str());

    return 0;
}




