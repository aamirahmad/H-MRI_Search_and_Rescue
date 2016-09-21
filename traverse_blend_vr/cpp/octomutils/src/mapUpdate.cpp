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

#include <sys/types.h>
#include <dirent.h> // readdir opendir closedir

#include <unistd.h> // getopt
#include <getopt.h>

#define no_argument 0
#define required_argument 1
#define optional_argument 2
#define VERSION 1.0

#define NOBLEND 0
#define save_bt_file 0


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

void readOcTreeFile(std::string path, BinaryOctree &bin_octree);
void getSubTrees(BinaryOctree* bin_octree,vector<Subtree>& subtrees);
std::vector<Subtree> compare(BinaryOctree* curr_bin_octree,std::vector<Subtree>* curr_sub_trees,BinaryOctree* prev_bin_octree,std::vector<Subtree>* prev_sub_trees);
void buildSubtree(std::vector<Subtree> , BinaryOctree*, string );

CompareOcTrees _comp_trees; // compare subtree object, can also get subtrees form octree
octomapToBlend *_omtb_complete = new octomapToBlend();

int main( int argc, char *argv[] ){

    BinaryOctree prev_bin_octree;
    std::vector<Subtree> prev_sub_trees;

    // init the python stuff
    _omtb_complete->init();



    std::string octree_path("/tmp/octomapToBlend/maps");
    // preamble
    std::string fst_filename;

    //main endless loop
    while(true){

        // check if there is a new octree to read
        BinaryOctree curr_bin_octree;
        readOcTreeFile(octree_path,curr_bin_octree);

        if(curr_bin_octree.getData()->size() > 0){ // make sure we don't use empty trees
            // get the subtrees of the current binary octree
            std::vector<Subtree> curr_sub_trees;

            getSubTrees(&curr_bin_octree,curr_sub_trees);
            // make a tempory copy of the current subtrees because compare might alter this vector
            std::vector<Subtree> to_save = curr_sub_trees;

            if(prev_sub_trees.size() > 0){ // can compare something
                // find the subtrees which have to be updated
                // this alters the current subtree vector and removes subtrees which can be found in prev_sub_trees
                // and are equal to their counterpart
                _comp_trees.compare(&prev_bin_octree,&prev_sub_trees,&curr_bin_octree,&curr_sub_trees);
            }

            if(curr_sub_trees.size() > 0){
                cout << "build blend" << endl;
                // build the subtrees which have been updated
                cout << curr_sub_trees.size()<< " "<< curr_bin_octree.getData()->size() << endl;
                buildSubtree(curr_sub_trees,&curr_bin_octree,"/tmp/blender");
            }

            // save the new tree state
            prev_bin_octree = curr_bin_octree;
            prev_sub_trees = to_save;


        }
        // just for debugging I added a sleep here
            sleep(1);
    }
        _omtb_complete->finalize();

}

bool hasEnding (std::string const &fullString, std::string const &ending) {
    if (fullString.length() >= ending.length()) {
        return (0 == fullString.compare (fullString.length() - ending.length(), ending.length(), ending));
    } else {
        return false;
    }
}

void readOcTreeFile(string path,BinaryOctree& bin_octree){

    DIR* dirp;
    dirent* dp;
    // make the readOcTreeFile() blocking
    // wait for a file to be within the folder
    while(true){
        dirp = opendir(path.c_str());
        while ((dp = readdir(dirp)) != NULL){
            cout << dp->d_name << endl;
            if (hasEnding(dp->d_name,".bt")) {
                std::cout << "found " << dp->d_name << std::endl;
                std::stringstream completefile;
                completefile << path << "/" << dp->d_name;
                std::ifstream bintree_file(completefile.str().c_str(),  std::ios_base::in | std::ios_base::binary);

                // check if file is open
                if (!bintree_file.is_open()){
                    cerr << bintree_file << " could not be opened" << endl;
                    break;
                }
                BinaryOctree bo(bintree_file);
                bin_octree = bo;
                (void)closedir(dirp);
                return;
            }
        }
        (void)closedir(dirp);
        // sleep to define the check intervall for new files
        std::cout << "waiting for file in " << path << std::endl;

        sleep(1);

    }
    // when we exit the function we need to make sure we close the dir
    (void)closedir(dirp);
}

void getSubTrees(BinaryOctree *bin_octree, vector<Subtree> &subtrees){
    // only if there is actual data in the tree other wise this ould cause a seg fault further down below
    if(bin_octree->getData()->size() > 0){
        subtrees = _comp_trees.getSubtrees_iter(bin_octree->getData(),bin_octree->getResolution());
    }
}

void buildSubtree(std::vector<Subtree> subtrees, BinaryOctree* bin_octree,std::string output_path){
    if(subtrees.size() < 1 || bin_octree->getData()->size() < 1){
        return;
    }




    // prepare the conversion from my binary octree to octomap bin octree
    vector<bitset<8> >data_bit8 = *bin_octree->getData();
    vector<unsigned char> data_uchar;
    data_uchar.reserve(data_bit8.size());

    /////////////////////////////////
    // BUILDING THE BLEND MESH FILES
    /////////////////////////////////

    vector<Origin> subtree_origs;
    cout << "number of subtrees " << subtrees.end() - subtrees.begin() << endl;

    for(std::vector<Subtree >::iterator subT_it = subtrees.begin(); subT_it != subtrees.end(); ++subT_it){

        int lb = (*subT_it).getSubTreePointer();
        int ub = (*subT_it).getSubTreeEndPointer();
        if( ub == 0 ){
            ub = data_bit8.size();
        }
        Origin* orig = (*subT_it).getOrigin();
        subtree_origs.push_back((*orig));
        // clear any old data out of the uchar vector
        data_uchar.clear();

        // convert the binary octree to octomap octree readable (conversion from bitset 8 to u int8)
        for(std::vector<bitset<8> >::iterator data_bit8_it = data_bit8.begin()+lb; data_bit8_it != data_bit8.begin()+lb+(ub-lb); ++data_bit8_it) {
            data_uchar.push_back(static_cast<u_int8_t> ((*data_bit8_it).to_ulong() ));
        }

        std::stringstream datastream_uchar;
        datastream_uchar.write((const char*) &data_uchar[0] ,data_uchar.size());
        // set the correct resolution for the octomap octree
        octomap::OcTree* octree = new octomap::OcTree(bin_octree->getResolution());

        // read the uint 8 data to the octomap octree
        octree->readBinaryData(datastream_uchar);
        // generate the name for the subtree
        std::stringstream subtree_name;
        subtree_name << output_path << "map_" << subT_it - subtrees.begin() << "_X" <<  (*subT_it).getOrigin()->x
                     << "Y" <<  (*subT_it).getOrigin()->y
                     << "Z" <<  (*subT_it).getOrigin()->z;
        // if this flag is set the .bt file will be written to disk
        if(save_bt_file){
            std::stringstream binary_tree_name;
            binary_tree_name << subtree_name << ".bt";
            octree->writeBinary(binary_tree_name.str());
        }

        // with this flag one can prevent the creation  of a blend file
        if(!NOBLEND){
            _omtb_complete->createMeshData(octree,16,subtree_name.str(),(*subT_it).getOrigin());
        }

    }




}
