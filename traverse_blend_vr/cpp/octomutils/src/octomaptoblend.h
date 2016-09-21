#ifndef OCTOMAPTOBLEND_H
#define OCTOMAPTOBLEND_H

#include <octomap/octomap.h>
#include <Python.h>
#include <numpy/arrayobject.h>
#include <typeinfo>
#include "compareoctrees.h"

class octomapToBlend
{
public:
    octomapToBlend();
    ~octomapToBlend();
    int createMeshData(octomap::OcTree *octree, int depth, std::string filename,Origin *origin);
    int createMesh();
    int callPythonClassFunction(double *c_arr_verts, size_t verts_len,std::string filename);
    void init();
    void finalize();
private:
    bool _init;
    bool _finalize;
    void* init_numpy();
    template <class T> PyArrayObject *makeNpArray(T c_array[], size_t size);
    template <class T> PyObject *makelist(T array[], size_t size);
};

#endif // OCTOMAPTOBLEND_H
