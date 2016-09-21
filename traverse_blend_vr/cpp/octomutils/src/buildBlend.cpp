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
#include <sstream>

// include octomap header
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap/AbstractOcTree.h>

// own files
#include "binaryoctree.h"
#include "compareoctrees.h"
#include "octomaptoblend.h"


using namespace std;

using namespace octomap;



int calcNumNodes(std::vector<bitset<8> > *data,int lb,int ub){
    bitset<8> node;
    int n = 0;
    for(size_t j = lb; j < ub;j++){
        node = (*data)[j];
        //    std::bitset<8> occupied;
        for(int i =0;i < 4; i++){
            /// See if the this node is a parent node
            if (!(node[2*i+1] == 0 && node[2*i] == 0)){
                //                std::cout << node[2*i+1] << node[2*i] <<  std::endl;
                n+=1;
            }
        }
    }
    return n+1;
}

BinaryOctree bioctree_0;
std::vector<Subtree> subTree_0;
int main( int argc, char *argv[] ){

    CompareOcTrees comp_bio;
    if (argc != 2){
        std::cout << "not in my house" << std::endl << "  usage: " << argv[0] << " <octomap.bt> " << std::endl;
    }else{
        clock_t overall_start;
        overall_start = clock();
        std::string filename = argv[1];
        std::cout << filename.c_str() << std::endl;
        std::ifstream file(filename.c_str(),  std::ios_base::in | std::ios_base::binary);
        std::ifstream file2(filename.c_str(),  std::ios_base::in | std::ios_base::binary);


        clock_t cstart;
        cstart = clock();

        BinaryOctree bioctree_1(file);
        std::cout <<"reading first file " << float(clock()-cstart)/float(CLOCKS_PER_SEC) << std::endl << std::endl;


        CompareOcTrees compTrees;
        cstart = clock();
        std::vector<Subtree> subtrees_1 = compTrees.getSubtrees_iter(bioctree_1.getData(),bioctree_1.getResolution());
        std::cout <<"get the first Subtree took " << float(clock()-cstart)/float(CLOCKS_PER_SEC) << std::endl << std::endl;


        std::cout <<"number of subtrees found in tree 1 " << subtrees_1.size()  << std::endl;

        if(bioctree_0.getData()->size() > 0){
            std::vector<Subtree> subtrees_0 = compTrees.getSubtrees_iter(bioctree_0.getData(),bioctree_0.getResolution());
            std::cout <<"get the first Subtree took " << float(clock()-cstart)/float(CLOCKS_PER_SEC) << std::endl << std::endl;

            std::cout <<"comparing octrees " << std::endl;
            compTrees.compare(&bioctree_0,&subtrees_0,&bioctree_1,&subtrees_1);
        }

        BinaryOctree bioctree_2;
        vector<bitset<8> >data_bit8 = bioctree_2.read(file2);


        cstart = clock();

        vector<unsigned char> data_uchar;
        data_uchar.reserve(data_bit8.size());

        //        for(std::vector<bitset<8> >::iterator data_bit8_it = data_bit8.begin(); data_bit8_it != data_bit8.end(); ++data_bit8_it) {
        //            data_uchar.push_back(static_cast<u_int8_t> ((*data_bit8_it).to_ulong() ));
        //        }
        //        std::stringstream datastream_uchar;
        //        datastream_uchar.write((const char*) &data_uchar[0] ,data_uchar.size());

        //          cout << data_uchar.size() << endl;

        //        octomap::OcTree* octree_complete = new octomap::OcTree(0.05);

        //        octree_complete->readBinaryData(datastream_uchar);
        //        std::stringstream name;
        //        name << "/tmp/octomapTest_" << "complete" << ".bt";
        //        cout << name.str() << endl;
        ////        octree_complete->writeBinary(name.str());

        octomapToBlend *omtb = new octomapToBlend();
        for(std::vector<Subtree >::iterator subT_it = subtrees_1.begin(); subT_it != subtrees_1.end(); ++subT_it){
//            (*subT_it).printSubTree();
            int lb = (*subT_it).getSubTreePointer();
            int ub = (*subT_it).getSubTreeEndPointer();
            if( ub == 0 ){
                ub = data_bit8.size();
            }
            Origin* orig = (*subT_it).getOrigin();
            data_uchar.clear();

            for(std::vector<bitset<8> >::iterator data_bit8_it = data_bit8.begin()+lb; data_bit8_it != data_bit8.begin()+lb+(ub-lb); ++data_bit8_it) {
                data_uchar.push_back(static_cast<u_int8_t> ((*data_bit8_it).to_ulong() ));
            }

            std::stringstream datastream_uchar;
            datastream_uchar.write((const char*) &data_uchar[0] ,data_uchar.size());
            octomap::OcTree* octree = new octomap::OcTree(0.05);
            octree->readBinaryData(datastream_uchar);
            std::stringstream name;
            name << "/tmp/octomaptest/octomapTest_" << subT_it - subtrees_1.begin()<< ".bt";
//            cout << name.str() << endl;
//            octree->writeBinary(name.str());


//            omtb->createMeshData(octree);



        }

//        omtb->testPython(argc,argv,std::string("createMesh"),std::string("run"));
        omtb->testPython(argc,argv,std::string("sys"),std::string("version_info"));
        std::cout <<"creating all octrees took " << float(clock()-cstart)/float(CLOCKS_PER_SEC) << " on average " << (float(clock()-cstart)/float(CLOCKS_PER_SEC))/float(subtrees_1.size()) << std::endl << std::endl;




        //            std::stringstream datastream;



        //            if (bioctree_1.getData()->size() > 0){
        //                long int lb =  (long int )subtrees_1[h].getSubTreePointer();
        //                long int ub = (long int )subtrees_1[h].getSubTreeEndPointer();
        //                vector<bitset<8> > tree(*bioctree_1.getData());
        //                datastream.write((const char*) &tree[lb],ub-lb);
        //                //                std::copy(tree.begin(),tree.end(), std::ostream_iterator<char>(datastream));


        ////                octree->readBinaryData(datastream);
        ////                getMeshData(octree);
        //            }


        //        }


        //        if (!msg.binary)
        //            return NULL;

        //            // exerpt to create an octree
        //            std::string id = bioctree_1.getID();
        //            double res = (double) bioctree_1.getResolution();
        //            unsigned size = (unsigned) bioctree_1.getSize();
        //            octomap::AbstractOcTree* octomap_tree = createTree(id, res);
        //            std::istream s;

        //            if (octomap_tree){
        //                if (size > 0)
        //                    tree->readData(s);

        //                OCTOMAP_DEBUG_STR("Done ("<< tree->size() << " nodes)");
        //            }

        //            return tree;

        //        tree.erase(tree.begin() + 2000,tree.end());
        //        int childCount = comp_bio.getChildCount_rec(&tree,0);
        //        std::cout <<"getChildCount took " << float(clock()-cstart)/float(CLOCKS_PER_SEC) << std::endl;
        //        std::cout << "childCount " << childCount << std::endl;
        std::cout <<"read the octomap took " << float(clock()-overall_start)/float(CLOCKS_PER_SEC) << std::endl;



    }
}




