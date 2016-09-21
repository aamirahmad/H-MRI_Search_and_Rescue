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
vector<Origin> CompareOcTrees::calcOrigin(std::bitset<8>  child1to4,std::bitset<8>  child5to8, float resolution, int current_depth, Origin origin){
    vector<Origin> origins;

    // quarter the size
    float r4 = this->calcSize(resolution,current_depth)*.25;
    // array defining the order in which children are sorted in octomap
    // this array describes the order of the child node origins
    // the origins are translated in a binary fashin where 0 corresponds to - and 1 to +
    // the list can be seen as tripple of xyz translations
    int XYZ[24] = {-1,-1,-1, 1,-1,-1, -1,1,-1, 1,1,-1, -1,-1,1, 1,-1,1, -1,1,1, 1,1,1};
    //    int XYZ[24] = {1,1,1, -1,1,1, 1,-1,1, -1,-1,1, 1,1,-1, -1,1,-1, 1,-1,-1, -1,-1,-1};
    // calc x y and z according to the order adding or subtracting res/4 depending on the childs position
    //    float child_position[24];
    // get the binary representation of the node

    bitset<8> childntom;
    for(size_t i =0; i < 8;i++){
        // select the part of the node we are in
        if(i < 4){
            childntom = child1to4;
        }else{
            childntom = child5to8;
        }
        // only if therere is a child node add the origin to the list
        if ((((childntom) & (std::bitset<8>((3 << 3*2) >> i%4*2))) == (std::bitset<8>((3 << 3*2) >> i%4*2)))){
            // calculate the new child node origin according to the list above
            // adding or subtracting a quarter of the current node size accordingly
            //cout << r4*XYZ[i*3+0] << " " << r4*XYZ[i*3+1] << " " << r4*XYZ[i*3+2] << endl;
            float x = origin.x+r4*XYZ[i*3+0];
            float y = origin.y+r4*XYZ[i*3+1];
            float z = origin.z+r4*XYZ[i*3+2];
            origins.push_back(Origin(x,y,z));
        }
    }
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

    // TODO check if the order of origins is correct
    //std::reverse(subTreeOrigins.begin(),subTreeOrigins.end());
    // loop through the tree once (without the root)
    //    return std::vector<Subtree>();
    int i = 0; // initialize the tree idx
    for(vector<bitset<8> >::iterator it=tree->begin();it!=tree->end();++it){

        node_1to4 = (*it); // store the current node in the node varialble
        ++it; // advance the iterator once more because one node is stored not in 8 but 16 bits
        node_5to8 = (*it); // store the current node in the node varialble
        i = it - tree->begin();
//        cout << node_1to4 << " " << node_5to8 << " " << i << endl;


        //        cout << node_1to4 << node_5to8 << " " << branchCounter << " "<<  depth << " i: " << i << endl;
        // at a depth of 10 we collect all subtrees
        if (depth == 10){
            // update the subTreePointer to the current index plus one
            subTreePointer = i+2;
            // set the end pointer of the previous subTree (for convinience later on)
            if(subTrees.size() > 0){
                subTrees.back().setLength(i+2);
            }
            //append subtrees
            subTrees.push_back(Subtree(subTreePointer,depth,origins.back(),this->calcSize(resolution,depth)));
            // print the found subtree with its attributes nicely
        }
        // branchCounter at zero means that a new subtree is starting
        if (branchCounter == 0){
            // reset the origins vector
            origins.empty();
            // set the corresponding origin of the current (of the eight base) subtree
            origins.push_back(subTreeOrigins[subTree]);
            subTree += 1; // the nexttime around we consider the next subtree
            depth = 1; // at the toplevel depth is eq to one
            // the depth at branch on the toplevel is one (one below the root)
            depthAtBranch.push_back(1);
            // reset the branch counter to one
            branchCounter = 1;
            // update the subTreePointer to the current index plus one
            subTreePointer = i+1;
            //            subTrees.push_back(Subtree(subTreePointer,depth,Origin(0.,0.,0.),0.));
        }
        if(isLeafNode(node_1to4,node_5to8)){
            // if this is a leaf node we close one branch
            branchCounter -= 1;
            // and set the depth back to that depth of the last branch
            depth = depthAtBranch.back();
            depthAtBranch.pop_back();
        }else{
            // get the origins of the child nodes
            vector<Origin> origins_tmp = this->calcOrigin(node_1to4,node_5to8,resolution,depth,origins.back());
            // we need to reverse the order of the origins because the origins are accessed in a first in first out fashion
            // and the next origin is always taken from the back of the vector
            std::reverse(origins_tmp.begin(),origins_tmp.end());
            // remove the last element
            origins.pop_back();
            // append the newly found origins to the set of origins
            origins.insert(origins.end(), origins_tmp.begin(), origins_tmp.end());


            int branchCount = getBranchCount(node_1to4,node_5to8)-1;
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
void CompareOcTrees::compare(BinaryOctree *orig_tree, vector<Subtree> *orig_subTrees, BinaryOctree *up_tree, vector<Subtree> *up_subTrees){
    clock_t cstart;
    cstart = clock();
    for(size_t i = 0; i < orig_subTrees->size();i++){

        // under the assumption that no subtree vanishes only subtrees are added
        for(size_t j = 0; j < up_subTrees->size();j++){
            if ((*(*orig_subTrees)[i].getOrigin()) == (*(*up_subTrees)[j].getOrigin())){
                // find upper and lower bounds for the subtrees
                int orig_ub = i+1 < orig_subTrees->size() ? (*orig_subTrees)[i+1].getSubTreePointer() : orig_tree->getData()->size();
                int orig_lb = (*orig_subTrees)[i].getSubTreePointer();
                int up_ub = j+1 < up_subTrees->size() ? (*up_subTrees)[j+1].getSubTreePointer() : up_tree->getData()->size();
                int up_lb =  (*up_subTrees)[j].getSubTreePointer();
                // compare the two subtrees
                if(!this->compareSubTrees(orig_tree->getData(),orig_lb,orig_ub,up_tree->getData(),up_lb,up_ub)){
                    cout << "subTree " << j << " needs to be updated "<< endl;
                }else{
                    (*up_subTrees).erase(up_subTrees->begin() + j);
                }
                break;
            }
        }

    }
    std::cout <<"comparing subtrees  " << float(clock()-cstart)/float(CLOCKS_PER_SEC) << std::endl;
    cout << up_subTrees->size() << " subtrees needs to be updated or replaced" << endl;
}
/** \brief comparing two trees with its binary representation, first thing to test is the length of both
 *  if they differ then the two subtrees cannot be equal, after that the subtrees can be compared bitwise
 *
 * \param tree1:
 * \param tree2: the two trees which should be compared
 */
bool CompareOcTrees::compareSubTrees(std::vector<std::bitset<8> > *data_tree1, int lb1, int ub1, std::vector<std::bitset<8> > *data_tree2, int lb2, int ub2){


    // subtrees must have same size
    if(ub1-lb1 != ub2-lb2){
        return false;
    }
    bitset<8> node1_1to4;
    bitset<8> node1_5to8;

    bitset<8> node2_1to4;
    bitset<8> node2_5to8;

    //    std::cout << ub1 << " " << lb1 << " " << ub2 << " " << lb2 << std::endl;

    //   std::cout << "node count " << ub1-lb1 << " lower bound " << lb1 << " upper bound " << ub1 << " " << tree1->size()  << std::endl;


    for(size_t i=0;i < (ub1-lb1);i+=2){

        node1_1to4 = (*data_tree1)[lb1+i];
        node1_5to8 = (*data_tree1)[lb1+i+1];

        node2_1to4 = (*data_tree2)[lb2+i];
        node2_5to8 = (*data_tree2)[lb2+i+1];

        //        std::cout << "is occupied? " << this->isOccupied(node2_1to4,node2_5to8) << std::endl;
        if(node1_1to4 != node2_1to4 || node1_5to8 != node2_5to8){
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

