#ifndef BINARYOCTREE_H
#define BINARYOCTREE_H

#include <fstream>
#include <bitset>
#include <sys/time.h>
#include <vector>
#include <math.h>
#include <iostream>



class BinaryOctree
{
public:
    BinaryOctree();
    BinaryOctree(std::istream &s);

    std::vector<std::bitset<8> >    read(std::istream &s);
    float                           getResolution();
    int                             getSize();
    std::vector<std::bitset<8> >    *getData();
    std::string                     getID();


    void                            setSize(int size);
    void                            setResolution(float res);
    void                            setID(std::string id);

private:
    bool                            readHeader(std::istream& s, std::string& id, int& size, float& res);

    std::vector<std::bitset<8> >    readBinaryData(std::istream &s,unsigned size);

    static const std::string        fileHeader;
    static const std::string        binaryFileHeader;
    std::string                     id_;
    float                           resolution_;
    int                             size_;
    std::vector<std::bitset<8> >    data_;
};

#endif // BINARYOCTREE_H
