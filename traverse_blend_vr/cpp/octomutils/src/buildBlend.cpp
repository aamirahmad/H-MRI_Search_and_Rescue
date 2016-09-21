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

#include <unistd.h> // getopt
#include <getopt.h>

#define no_argument 0
#define required_argument 1
#define optional_argument 2
#define VERSION 1.0

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

bool assert_origins(vector<Origin> origs1,vector<Origin> origs2){
    if (origs1.size() != origs2.size()){
        cout << "origins have not the same size" << endl;
        return false;
    }
    vector<Origin>::iterator it2 = origs2.begin();
    for (vector<Origin>::iterator it = origs1.begin(); it != origs1.end(); ++it,++it2){
        // compare Origins
        if((*it) != (*it2)){
            return false;
        }
    }
    return true;
}


static void show_usage(std::string name)
{
    std::cerr << "not in my house" << std::endl << "  usage: " << name << " <octomap.bt> [OPTIONS]\n"
              << "Options:\n"
              << "\t-h,--help\t\tShow this help message and exit\n"
              << "\t-v,--version\t\tShow the version and exit\n"
              << "\t-b,--savetree\t\tsave the binary tree to outpath\n"
              << "\t--noblend\t\tDon't convert the tree to a blend file\n"
              << "\t-o,--outpath DESTINATION\tSpecify the destination path"
              << std::endl;
}

static int noblend;
/**
 * @brief main mainloop Reading an octomap binary tree file and building blend files according to the found subtrees.
 * It is also possible to build the subtrees as binary octomaps
 * @param argc argument count
 * @param argv argument vector
 * @return int
 */
int main( int argc, char *argv[] ){

    // check the length of the argv
    if (argc < 2){
        show_usage(argv[0]);
        return 1;
    }

    /// the long version of the options for the command line usage
    const struct option longopts[] =
    {
        {"noblend", no_argument,        &noblend,1},
        {"version",   no_argument,        0, 'v'},
        {"help",      no_argument,        0, 'h'},
        {"savetree",  no_argument,        0, 'b'},
        {"outpath",   required_argument,  0, 'o'},
        {0,0,0,0},
    };


    int show_help_flag = 0;
    int show_version_flag = 0 ;
    int savetree_flag = 0;
    int oflag = 0;

    char *ovalue = NULL;

    int option_ind;
    int c;

    opterr = 0;
    cout << argc<< endl;
    // set the arguments
    while ((c = getopt_long (argc, argv, "o:vhbw",longopts, &option_ind)) != -1)
        switch (c)
        {
        case 0:
            /* If this option set a flag, do nothing else now. */
            if (longopts[option_ind].flag != 0)
                break;
            printf ("option %s", longopts[option_ind].name);
            if (optarg)
                printf (" with arg %s", optarg);
            printf ("\n");
            break;
        case 'h':
            show_help_flag = 1;
            break;
        case 'v':
            show_version_flag = 1;
            break;
        case 'b':
            savetree_flag = 1;
            break;
        case 'o':
            oflag = 1;
            ovalue = optarg;
            break;
        case '?':
            if (optopt == 'o')
                fprintf (stderr, "Option -%c requires an argument.\n", optopt);
            else if (isprint (optopt))
                fprintf (stderr, "Unknown option `-%c'.\n", optopt);
            else
                fprintf (stderr,
                         "Unknown option character `\\x%x'.\n",
                         optopt);
            return 1;
        default:
            abort ();
        }


    printf ("hflag = %d, vflag = %d,savetree_flag %d, noblend %d, ovalue = %s\n",
            show_help_flag, show_version_flag, savetree_flag, noblend, ovalue);

    if (show_help_flag){
        show_usage(argv[0]);
        return 0;
    }
    if (show_version_flag){
        std::cout << VERSION << std::endl;
        return 0;
    }

    // setting the output path to a default value
    std::string out_bt_filepath = "/tmp/octomaptest/";
    // if the corresponding commandline argument is set change the default accordingly
    if(oflag){
        out_bt_filepath = std::string(ovalue);
    }
    std::string file_name;
    // Print any remaining command line arguments (not options).
    // the last input which is not an option is considered to be the input file
    if (optind < argc){
        while (optind < argc){
            printf ("Non-option argument %s\n", argv[optind]);
            file_name = argv[optind++];
        }
    }else{ // if the filename is missing
        std::cerr << "need an input file" << endl;
        show_usage(argv[0]);
        return 1;

    }


    clock_t overall_start,cstart;
    overall_start = clock();
    cstart = clock();

    std::cout << file_name.c_str() << std::endl;
    std::ifstream tree_file(file_name.c_str(),  std::ios_base::in | std::ios_base::binary);

    // check if file is open
    if (!tree_file.is_open()){
        cerr << file_name << " could not be opened" << endl;
        return 1;
    }

    BinaryOctree bin_octree(tree_file);
    std::cout <<" reading file " << float(clock()- cstart)/float(CLOCKS_PER_SEC) << std::endl << std::endl;

    if(bin_octree.getSize() < 1){
        std::cerr << file_name << " is an empty tree" << std::endl;
        exit(0);
    }

    CompareOcTrees compTrees;
    cstart = clock();
    std::vector<Subtree> subtrees = compTrees.getSubtrees_iter(bin_octree.getData(),bin_octree.getResolution());
    std::cout << "get the first Subtree took " << float(clock()-cstart)/float(CLOCKS_PER_SEC) << std::endl << std::endl;

    std::cout << "number of subtrees found in tree " << subtrees.size()  << std::endl;


    cstart = clock();

    vector<bitset<8> >data_bit8 = *bin_octree.getData();
    std::cout <<"reading octomap file " << float(clock()-cstart)/float(CLOCKS_PER_SEC) << std::endl << std::endl;

    cstart = clock();

    vector<unsigned char> data_uchar;
    data_uchar.reserve(data_bit8.size());
    data_uchar.clear();



    std::stringstream name;


    octomapToBlend *omtb_complete = new octomapToBlend();
    std::cout << "we are now at " << float(clock()-overall_start)/float(CLOCKS_PER_SEC) << "s" << std::endl;
    clock_t start_py_init = clock();
    omtb_complete->init();
    std::cout <<"py init took " << float(clock()-start_py_init)/float(CLOCKS_PER_SEC) << std::endl << std::endl;

    int numOccNodes_inSubtrees = 0;


    /////////////////////////////////
    // BUILDING THE BLEND MESH FILES
    /////////////////////////////////

    vector<Origin> subtree_origs;
    cout << "number of subtrees " << subtrees.end() - subtrees.begin() << endl;
    for(std::vector<Subtree >::iterator subT_it = subtrees.begin(); subT_it != subtrees.end(); ++subT_it){

        (*subT_it).printSubTree();


        int lb = (*subT_it).getSubTreePointer();
        int ub = (*subT_it).getSubTreeEndPointer();
        if( ub == 0 ){
            ub = data_bit8.size();
        }
        Origin* orig = (*subT_it).getOrigin();
        subtree_origs.push_back((*orig));
        data_uchar.clear();

        for(std::vector<bitset<8> >::iterator data_bit8_it = data_bit8.begin()+lb; data_bit8_it != data_bit8.begin()+lb+(ub-lb); ++data_bit8_it) {
            data_uchar.push_back(static_cast<u_int8_t> ((*data_bit8_it).to_ulong() ));
        }

        std::stringstream datastream_uchar;
        datastream_uchar.write((const char*) &data_uchar[0] ,data_uchar.size());
        octomap::OcTree* octree = new octomap::OcTree(bin_octree.getResolution());

        octree->readBinaryData(datastream_uchar);
        std::stringstream subtree_name;
        subtree_name << out_bt_filepath << "map_" << subT_it - subtrees.begin() << "_X" <<  (*subT_it).getOrigin()->x
                                                                                << "Y" <<  (*subT_it).getOrigin()->y
                                                                                << "Z" <<  (*subT_it).getOrigin()->z;
        cout << name.str() << endl;
        // TEST THE SUBTREE INTEGRETY
        if(savetree_flag){
            std::stringstream binary_tree_name;
            binary_tree_name << subtree_name << ".bt";
            octree->writeBinary(binary_tree_name.str());
        }


        if(! noblend){
               numOccNodes_inSubtrees += omtb_complete->createMeshData(octree,16,subtree_name.str(),(*subT_it).getOrigin());
        }

    }
    omtb_complete->finalize();
    std::cout << " numOccNodes_inSubtrees " << numOccNodes_inSubtrees << endl;
    std::cout <<"creating all octrees took " << float(clock()-cstart)/float(CLOCKS_PER_SEC)<< endl << "\ton average " << (float(clock()-cstart)/float(CLOCKS_PER_SEC))/float(subtrees.size()) << std::endl << std::endl;


    std::cout <<"read the octomap took " << float(clock()-overall_start)/float(CLOCKS_PER_SEC) << std::endl;




}




