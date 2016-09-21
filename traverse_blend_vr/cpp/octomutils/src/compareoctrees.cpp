#include "compareoctrees.h"

using namespace std;
CompareOcTrees::CompareOcTrees(){}
CompareOcTrees::~CompareOcTrees(){ }


/** \brief calcOrigins calulates the 3D origins for a node, but only for the parts where the node has children
 *
* \param node: a pointer to two bitset<8>
* \param resolution: size of the smallest leaf node
* \param current_depth: the depth of the node
* \param origin: the origin of the node
* \param size: the volume of the node, because its a cube only one float is sufficient
*/
vector<Origin> CompareOcTrees::calcOrigin(std::bitset<8>  node1to4,std::bitset<8>  node5to8, float resolution, int current_depth, Origin origin){
    vector<Origin> origins;

    // quarter the size
    float r4 = this->calcSize(resolution,current_depth)*.25;
    // array defining the order in which children are sorted in octomap
    // this array describes the order of the child node origins
    // the origins are translated in a binary fashin where 0 corresponds to - and 1 to +
    // the list can be seen as tripple of xyz translations
    //    int XYZ_5to8[24] = { 1, 1, 1,-1, 1, 1, 1,-1, 1,-1,-1, 1};
    //    int XYZ_1to4[24] = {-1,-1,-1, 1,-1,-1,-1, 1,-1, 1, 1,-1,};

    int XYZ_1to4[24] = { -1,-1,-1       ,1,-1,-1,        -1,1,-1    ,1,1,-1};
    int XYZ_5to8[24] = {-1,-1,1,       1,-1,1,         -1,1,1,         1,1,1};



    //    int XYZ[24] = {1,1,1, -1,1,1, 1,-1,1, -1,-1,1, 1,1,-1, -1,1,-1, 1,-1,-1, -1,-1,-1};
    //    int XYZ[24] = {1,1,1, 1,1,-1, 1,-1,1, 1,-1,-1, -1,1,1, -1,1,-1, -1,-1,1, -1,-1,-1};
    // calc x y and z according to the order adding or subtracting res/4 depending on the childs position
    //    float child_position[24];
    // get the binary representation of the node


    int childCount_5to8 = 0;
    for(size_t i = 0; i < 4;i++){
        if (node5to8[2*i+1] == 1 && node5to8[2*i] == 1){
            //            std::cout << child5to8 <<  " " << occupied <<  std::endl;
            float x = origin.x+r4*XYZ_5to8[i*3+0];
            float y = origin.y+r4*XYZ_5to8[i*3+1];
            float z = origin.z+r4*XYZ_5to8[i*3+2];
            childCount_5to8 += 1;
            origins.push_back(Origin(x,y,z));
        }

    }
    int childCount_1to4 = 0;
    for(size_t i = 0; i < 4;i++){
        if (node1to4[2*i+1] == 1 && node1to4[2*i] == 1){
            //            std::cout << child1to4 <<  " " << occupied <<  std::endl;
            float x = origin.x+r4*XYZ_1to4[i*3+0];
            float y = origin.y+r4*XYZ_1to4[i*3+1];
            float z = origin.z+r4*XYZ_1to4[i*3+2];
            childCount_1to4 += 1;
            origins.push_back(Origin(x,y,z));
        }
    }

    // reverse the order of the origins because bits are read from left to right or LSB first

    std::reverse(origins.begin(),origins.begin()+childCount_5to8);
    std::reverse(origins.begin()+childCount_5to8,origins.end());

    // return the vector of origins
    return origins;
}
/** \biref calculate the size of the octree node at a given depth provided the resolution
 *  \param resolution: the size of the leafnodes at max depth
 * \param depth: the depth at which the size should be calculated
 * \param maxDepth: the maxDepth of the entire tree, defaults to 16
 */
float CompareOcTrees::calcSize(float resolution,int depth,int maxDepth){
    return (resolution*pow(2,(maxDepth-depth)));
}

/** \brief check if a node is occupied
 * \param node: the binary node representation
 *
 * \return bool: isOccupied
 */
bool CompareOcTrees::isOccupied(bitset<8> node1to4, bitset<8> node5to8){


    //    std::bitset<8> occupied;
    for(int i =0;i < 4; i++){
        //        occupied.reset();
        //        occupied = (1 << 2*i);

        /// See if the this node is a parent node
        if (node1to4[2*i+1] == 0 && node1to4[2*i] == 1){
            //            std::cout << child1to4 <<  " " << occupied <<  std::endl;
            return true;
        }
        if (node5to8[2*i+1] == 0 && node5to8[2*i] == 1){
            //            std::cout << child5to8 <<  " " << occupied <<  std::endl;
            return true;
        }
    }
    return false;
}


/** \brief get the number of children this nodes has
 * \param node: the binary node representation
 *
 * \return int: number of children or outgoing edges
 */
int CompareOcTrees::getBranchCount(bitset<8> node1to4, bitset<8> node5to8){
    int branchCount = 0;

    for(int i =0;i < 4; i++){
        /// See if the this node is a parent node
        if ((((node1to4) & (std::bitset<8>((3 << 2*i)))) == (std::bitset<8>((3 << 2*i))))){
            branchCount += 1;
        }
        if ((((node5to8) & (std::bitset<8>((3 << 2*i)))) == (std::bitset<8>((3 << 2*i))))){
            branchCount += 1;
        }
    }
    return branchCount;
}
/** \brief check the node for children. If it has one 11 field in its description it is a parent and therefore not a leafnode
 * \param node: vector with two bitset<8> describing the binary node
 *
 * \return bool: isLeafnode or not
 */
bool CompareOcTrees::isLeafNode(bitset<8> node1to4,bitset<8> node5to8){
    bool isLeafNode = true;

    for(int i =0; i < 4; i++){
        /// See if the this node is a parent node
        std::bitset<8>((3 << 2*i));
        if ((((node1to4) & (std::bitset<8>(3 << 2*i))) == (std::bitset<8>((3 << 2*i))))){
            isLeafNode = false;
            break;
        }
        if ((((node5to8) & (std::bitset<8>((3 << 2*i)))) == (std::bitset<8>((3 << 2*i))))){
            isLeafNode = false;
            break;
        }
    }
    return isLeafNode;

}
/** \brief loops through the whole tree finding all the subtrees
 *  the subtrees at the predefined level are returned as vector of Subtree
 *
 * \param tree: the tree storing the binary representation of an octree
 *
 * \return subtree: vector of Subtree objects
 */
vector<Subtree> CompareOcTrees::getSubtrees_iter(std::vector<std::bitset<8> > *tree,float resolution){

    int branchCounter = 0; // keep track of how many branches are "open"
    int subTree = 0; // to keep track in which of the eight tolevel subtrees we're in
    int depth = 0; // current depht within the tree
    int subTreePointer = 0; // index of the start of a subtree

    vector<Origin> origins; // set of origins for the subtrees
    vector<int> depthAtBranch; // depth at branch tells at which level the tree branched last
    vector<bitset<8> > node; // variable for the currently looked at node

    bitset<8> root_1to4((*tree)[0]);
    bitset<8> root_5to8((*tree)[1]);
    bitset<8> node_1to4;
    bitset<8> node_5to8;

    vector<Subtree> subTrees; // return container of subtree objects

    vector<Origin> subTreeOrigins = this->calcOrigin(root_1to4,root_5to8,resolution,0,Origin(0,0,0));

    //    for(vector<Origin>::iterator it = subTreeOrigins.begin(); it!= subTreeOrigins.end();it++){
    //        (*it).printOrigin();
    //    }
    // TODO check if the order of origins is correct
    //std::reverse(subTreeOrigins.begin(),subTreeOrigins.end());
    // loop through the tree once (without the root)
    //    return std::vector<Subtree>();
    int i = 0; // initialize the tree idx
    for(vector<bitset<8> >::iterator it=tree->begin()+2;it!=tree->end();++it){

        node_1to4 = (*it); // store the current node in the node varialble
        it++; // advance the iterator once more because one node is stored not in 8 but 16 bits
        node_5to8 = (*it); // store the current node in the node varialble
        i = it - tree->begin()-2;


        //        cout << node_1to4 << node_5to8 << " " << branchCounter << " "<<  depth << " i: " << i << endl;
        // at a depth of 10 we collect all subtrees
        if (depth == 10){
            // update the subTreePointer to the current index plus one
            subTreePointer = i+1;
            // set the end pointer of the previous subTree (for convinience later on)
            if(subTrees.size() > 0){
                subTrees.back().setLength(subTreePointer);
            }
            //append subtrees
            subTrees.push_back(Subtree(subTreePointer,depth,origins.back(),this->calcSize(resolution,depth)));
            // print the found subtree with its attributes nicely
        }
        // branchCounter at zero means that a new subtree is starting
        if (branchCounter == 0){
            // reset the origins vector
            origins.clear();
            // set the corresponding origin of the current (of the eight base) subtree

            origins.push_back(subTreeOrigins.back());
            subTreeOrigins.pop_back();
            depth = 1; // at the toplevel depth is eq to one
            // the depth at branch on the toplevel is one (one below the root)
            depthAtBranch.push_back(1);
            // reset the branch counter to one
            branchCounter = 1;
            // update the subTreePointer to the current index plus one node (=2)
            subTreePointer = i+1;
            //            subTrees.push_back(Subtree(subTreePointer,depth,Origin(0.,0.,0.),0.));
        }

        if(CompareOcTrees::isLeafNode(node_1to4,node_5to8)){
            // if this is a leaf node we close one branch
            branchCounter -= 1;
            // and set the depth back to that depth of the last branch
            depth = depthAtBranch.back();
            depthAtBranch.pop_back();
            // remove origins of leafnodes
            origins.pop_back();
        }else{
            // get the origins of the child nodes
            vector<Origin> origins_tmp = this->calcOrigin(node_1to4,node_5to8,resolution,depth,origins.back());


            // remove the last element
            origins.pop_back();
            // append the newly found origins to the set of origins
            origins.insert(origins.end(), origins_tmp.begin(), origins_tmp.end());
            int branchCount = getBranchCount(node_1to4,node_5to8)-1;
            //            if(i < 76){
            //                cout << "branch count " << branchCount  << " depth " << depth << endl;
            //                for (std::vector<Origin>::iterator origing_iter=origins.begin(); origing_iter!=origins.end(); ++origing_iter){
            //                    (*origing_iter).printOrigin();
            //                }
            //                cout << endl;
            //            }


            branchCounter += branchCount;
            if (branchCount > 0){
                for (size_t k = 0; k < branchCount; k++){
                    depthAtBranch.push_back(depth+1);
                }

            }

            depth += 1;
        }
    }
    return subTrees;
}

/** \brief Compare Two Octrees and find the subtrees which differ
 *
 *  Under the asumption that the second octree is an updated version of the fist one this function
 * tries to find the subtrees which differ
 *
 * \param orig_tree: first octree (the original)
 * \param up_tree: the updated octree
 */

void CompareOcTrees::compare(BinaryOctree *orig_tree, BinaryOctree *up_tree){
    clock_t cstart;
    cstart = clock();

    std::vector<Subtree> orig_subTrees = this->getSubtrees_iter(orig_tree->getData(),orig_tree->getResolution());
    std::cout <<"reading first subtree  " << float(clock()-cstart)/float(CLOCKS_PER_SEC) << std::endl;

    std::vector<Subtree> up_subTrees = this->getSubtrees_iter(up_tree->getData(),up_tree->getResolution());
    std::cout <<"reading both subtrees  " << float(clock()-cstart)/float(CLOCKS_PER_SEC) << std::endl;


    //    for(size_t h = 0; h < orig_subTrees.size(); h++){
    //        orig_subTrees[h].printSubTree();
    //    }
    this->compare(orig_tree,&orig_subTrees,up_tree,&up_subTrees);

}
/**
 * @brief CompareOcTrees::compare the subtrees within two vectors
 * @param orig_tree binary octree which holds the actual data to compare
 * @param orig_subTrees subtree container which holds the definitions of the subtrees (start,end point etc)
 * @param up_tree second binary octree to compare two
 * @param up_subTrees subtree container of second tree
 */
void CompareOcTrees::compare(BinaryOctree *tree_fst, vector<Subtree> *subtree_fst, BinaryOctree *tree_snd, vector<Subtree> *subtree_snd){
    clock_t cstart;
    cstart = clock();
    // remeber the size of the second subtree
    int snd_subtree_size = subtree_snd->size();
    std::vector<int> idx;
    int update_count = 0;
    for(std::vector<Subtree >::iterator it = subtree_fst->begin(); it != subtree_fst->end(); it++){
        Origin O_fst = (*(*it).getOrigin());
        int lower_fst = (*it).getSubTreePointer();
        int upper_fst = (*it).getSubTreeEndPointer() == 0 ? tree_fst->getData()->size() : (*it).getSubTreeEndPointer() ;
        for(std::vector<Subtree >::iterator jt = subtree_snd->begin(); jt != subtree_snd->end(); jt++){
            int lower_snd = (*jt).getSubTreePointer();
            int upper_snd = (*jt).getSubTreeEndPointer()== 0 ? tree_snd->getData()->size() : (*jt).getSubTreeEndPointer() ;;

            Origin O_snd = (*(*jt).getOrigin());
            // finding corresponding suptrees
            if(O_fst == O_snd){
                //                cout << compareSubTrees(tree_fst->getData(),lower_fst,upper_fst,tree_snd->getData(),lower_snd,upper_snd) << endl;
                if(subtrees_areEqual(tree_fst->getData(),lower_fst,upper_fst,tree_snd->getData(),lower_snd,upper_snd)){
                    // subtrees not to update
                    idx.push_back(subtree_snd->end() - jt);
                }else{
                    cout << "found subtree to update " << endl;

                    cout << "start index fst " << lower_fst << " " << lower_snd << endl << "start index snd " << upper_fst << " " << upper_snd << endl;
                    cout << "len subtree fst " << upper_fst-lower_fst << endl << "len subtree snd " <<  upper_snd - lower_snd << endl;
                    cout << ++update_count << endl;
                }

            }


        }
    }
    // making sure that the biggest index comes first and is ordered down to the smallest one
    // this comesin handy when removing from the subtree vector by index
    std::sort(idx.begin(),idx.end(),std::greater<int>());

    for(std::vector<int>::iterator kt = idx.begin(); kt != idx.end(); kt++){
        subtree_snd->erase(subtree_snd->begin() + (*kt));
      }

    std::cout <<"comparing subtrees  " << float(clock()-cstart)/float(CLOCKS_PER_SEC) << std::endl;
    cout << subtree_snd->size() << "/" << snd_subtree_size << " subtrees needs to be updated or replaced" << endl;
}

/**
 * @brief CompareOcTrees::compareSubTrees using iterators, this method compared to the second implementation is rather slow
 * @param it_fst start iterator of the first subtree
 * @param upper_fst end iterator of the first subtree
 * @param it_snd start iterator of the second subtree
 * @param upper_snd end iterator of the second subtree
 * @return boolean wheather the two subtrees are identical
 */
bool CompareOcTrees::subtrees_areEqual(vector<bitset<8> >::iterator it_fst, vector<bitset<8> >::iterator upper_fst, vector<bitset<8> >::iterator it_snd, vector<bitset<8> >::iterator upper_snd){

    if((upper_fst-it_fst) != upper_snd - it_snd){
        cout << " they don't match in size" << endl;
        return false;
    }
    while(it_fst != upper_fst && it_snd != upper_snd){
        if((*it_fst) != (*it_snd))
            return false;

        it_fst++;
        it_snd++;
    }
    return true;
}

/** @brief comparing two trees with its binary representation, first thing to test is the length of both
 *  if they differ then the two subtrees cannot be equal, after that the subtrees can be compared bitwise
 *
 * @param tree1:
 * @param tree2: the two trees which should be compared
 */

/**
 * @brief CompareOcTrees::compareSubTrees comparing two trees with its binary representation, first thing to test is the length of both
 *  if they differ then the two subtrees cannot be equal, after that the subtrees can be compared bitwis
 * @param data_tree1 pointer to the data vector of the first tree
 * @param lb1 lower bound of the first subtree
 * @param ub1 upper bound of the first subtree
 * @param data_tree2 pointer to the data of the second tree
 * @param lb2 lower_bound of the second subtree
 * @param ub2 uppber bound of the second subtree
 * @return bolean wheather the two subtrees are equal
 */
bool CompareOcTrees::subtrees_areEqual(std::vector<std::bitset<8> > *data_tree1, int lb1, int ub1, std::vector<std::bitset<8> > *data_tree2, int lb2, int ub2){


    // subtrees must have same size
    if(ub1-lb1 != ub2-lb2){
        cout << "\t they don't match in size" << endl;
        return false;
    }
    bitset<8> node1_1to4;
    bitset<8> node1_5to8;

    bitset<8> node2_1to4;
    bitset<8> node2_5to8;

    //    std::cout << ub1 << " " << lb1 << " " << ub2 << " " << lb2 << std::endl;

    //   std::cout << "node count " << ub1-lb1 << " lower bound " << lb1 << " upper bound " << ub1 << " " << tree1->size()  << std::endl;



    for(size_t i=0;i < (ub1-lb1);i++){

        node1_1to4 = (*data_tree1)[lb1+i];
        node2_1to4 = (*data_tree2)[lb2+i];

        //        std::cout << "is occupied? " << this->isOccupied(node2_1to4,node2_5to8) << std::endl;
        if(node1_1to4 != node2_1to4){
            return false;
        }

    }
    return true;
}



/** \brief get the number of children for the n to n+4 children
 *
 *  helper function for the two bitsets child1to4 and childto8 from getChildCount()
 *
 *  \param child_n_to_n_p4 bitset<8> describing four child nodes
 * \param *tree pointer to the octomap octree
 * \param depht current depht
 * \param maxDepth maximum depth to discover
 */
int CompareOcTrees::getSubTrees_ntonp4_rec(std::bitset<8> child_n_to_n_p4,std::vector<std::bitset<8> > *tree,int depth){
    int numChildNodes= 0;
    for(int i =0;i < 4; i++){
        /// Failsafe when passing an empty tree
        if(tree->size() ==0){  break;}
        /// See if the this node is a parent node
        if ((((child_n_to_n_p4) & (std::bitset<8>((3 << 2*i)))) == (std::bitset<8>((3 << 2*i))))){
            numChildNodes += 1;
            //recursion call to get the childcount in depth first manner
            tree->erase(tree->begin(),tree->begin()+1);
            numChildNodes += getChildCount_rec(tree,depth+1);
        }
    }
    return numChildNodes;
}
/**  \brief get the number of child nodes underlying
 *
 * \param tree a binary representatio of an octomap octree data
 * \param depth the current depth of the tree
 * \param maxDepth the maximum depth which should be explored
 *
 * \return the number of child nodes
 */
int CompareOcTrees::getChildCount_rec( std::vector<std::bitset<8> > *tree ,int depth, int maxDepth){
    std::cout << "treesize " << tree->size() << std::endl;
    int childCount1to4 = 0;
    int childCount5to8 = 0;
    if (tree->size() == 0){
        std::cout << "tree is finished" << std::endl;
        return 0;
    }


    std::bitset<8> child1to4 ((*tree)[0][0]);
    std::bitset<8> child5to8 ((*tree)[0][1]);
    childCount1to4 = this->getSubTrees_ntonp4_rec(child1to4,tree,depth);
    childCount5to8 =this->getSubTrees_ntonp4_rec(child5to8,tree,depth);

    return childCount1to4+childCount5to8;
}

