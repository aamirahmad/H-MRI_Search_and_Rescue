#include "binaryoctree.h"


const std::string BinaryOctree::fileHeader = "# Octomap OcTree file";
const std::string BinaryOctree::binaryFileHeader = "# Octomap OcTree binary file";


/** \brief deafult Constructor
 */
BinaryOctree::BinaryOctree(){
    std::vector<std::bitset<8>  > data;
    data_ =  data;
    this->setSize(0);
    this->setID("");
    this->setResolution(0.0);
}



/** \brief Constructor read octomap from istream
 */
BinaryOctree::BinaryOctree(std::istream &s){
    data_ = this->read(s);
}

/** \brief read the standardized header of the binary octree
 *  \param s: the stream of the octree
 *  \param id: used as output the id of the tree
 *  \param size: reference to the size of the tree
 *  \param res: reference to the size of the smalles leaf nodes
 */
bool BinaryOctree::readHeader(std::istream &s, std::string &id, int &size, float &res){
    id = "";
    size = 0;
    res = 0.0;

    std::string token;
    bool headerRead = false;
    while(s.good() && !headerRead) {
        s >> token;
        if (token == "data"){
            headerRead = true;
            // skip forward until end of line:
            char c;
            do {
                c = s.get();
            } while(s.good() && (c != '\n'));
        }
        else if (token.compare(0,1,"#") == 0){
            // comment line, skip forward until end of line:
            char c;
            do {
                c = s.get();
            } while(s.good() && (c != '\n'));
        }
        else if (token == "id")
            s >> id;
        else if (token == "res")
            s >> res;
        else if (token == "size")
            s >> size;
        else{
            std::cout << "Unknown keyword in OcTree header, skipping: "<< token << std::endl;
            char c;
            do {
                c = s.get();
            } while(s.good() && (c != '\n'));
        }
    }
    return true;
}

/** \brief read the binary data of an octomap octree from file
 *
 *  \param s inpustream for the binary octomap
 *  \return vector with binary denotation of octree nodes
 */
std::vector<std::bitset<8>  > BinaryOctree::readBinaryData(std::istream &s,unsigned size){

    char half_child_char;
    std::vector<std::bitset<8> > tree;
    tree.reserve(size);

    while (s.read((char*)&half_child_char, sizeof(char))){
        std::bitset<8> child1to4 ((unsigned long long) half_child_char);
        tree.push_back(child1to4);
    }
    std::cout << std::endl << " tree size " << tree.size() << std::endl;
    return tree;
}

/** \brief read binary octomap octree
 *  \param s input stream
 *  \return vector of bitsets describing the tree nodes
 */
std::vector<std::bitset<8> > BinaryOctree::read(std::istream &s){
    // check if first line valid:
    std::string line;
    std::getline(s, line);
    if (line.compare(0,BinaryOctree::binaryFileHeader.length(), BinaryOctree::binaryFileHeader) !=0){
        std::cout << "First line of OcTree file header does not start with " << BinaryOctree::binaryFileHeader << std::endl;
        return std::vector<std::bitset<8> >();
    }

    if (!this->readHeader(s,this->id_, this->size_, this->resolution_)){
        return std::vector<std::bitset<8> >();
    }else{
        std::cout << "id " << this->id_ << " size  " << this->getSize() << " res " << this->getResolution() << std::endl;
        //        this->setID(id);
        //        this->setResolution(res);
        //        this->setSize(size);
    }
    std::vector<std::bitset<8> > tree(this->readBinaryData(s,this->getSize()));

    return tree;
}

std::string BinaryOctree::getID(){
    return this->id_;
}

float BinaryOctree::getResolution(){
    return this->resolution_;
}

int BinaryOctree::getSize(){
    return this->size_;
}

void BinaryOctree::setID(std::string id){
    this->id_ = id;
}

void BinaryOctree::setResolution(float res){
    this->resolution_ = res;
}

void BinaryOctree::setSize(int size){
    this->size_ = size;
}

/** \brief return the binary octomap octree
 *
 * \return std::vector<std::bitset<8> > data
 */
std::vector<std::bitset<8> >*  BinaryOctree::getData(){
    return &this->data_;
}
