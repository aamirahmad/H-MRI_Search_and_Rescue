#ifndef OCTOMAPTOBLEND_H
#define OCTOMAPTOBLEND_H

#include <octomap/octomap.h>
#include <Python.h>

class octomapToBlend
{
public:
    octomapToBlend();
    void createMeshData(octomap::OcTree *octree);
    int testPython(int argc, char *argv[], std::string module_name, std::string fnc_name);
};

#endif // OCTOMAPTOBLEND_H
