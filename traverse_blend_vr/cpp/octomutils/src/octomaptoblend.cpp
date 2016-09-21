#include "octomaptoblend.h"

using namespace octomap;
using namespace std;

octomapToBlend::octomapToBlend()
{
}


void octomapToBlend::createMeshData(octomap::OcTree *tree){
    //    tree->expand();
    cout << "number of leave nodes " <<  tree->getNumLeafNodes() << endl;

    double verticies [tree->getNumLeafNodes()*8][3];
    int leaf_counter = 0;
    for (OcTree::leaf_iterator it = tree->begin_leafs(), end = tree->end_leafs();  it != end; ++it)
    {
        if(tree->isNodeOccupied((*it)) ){
            if(it.getZ() > 0.1){
                double x = it.getX();
                double y = it.getY();
                double z = it.getZ();
                double s = it.getSize();


//                 verts.push_back(it.getCoordinate())
//                 static const double  arr[] = {x+s/2,y+s/2,z+s/2};
//                verticies[leaf_counter+0*8][0]= x+s/2;// = {x+s/2,y+s/2,z+s/2};
//                verticies[leaf_counter+0*8][1]= y+s/2;
//                verticies[leaf_counter+0*8][2]= z+s/2;

//                    verts.push_back((x+s/2,y+s/2,z-s/2))
//                    verts.push_back((x+s/2,y-s/2,z+s/2))
//                    verts.push_back((x+s/2,y-s/2,z-s/2))
//                    verts.push_back((x-s/2,y+s/2,z+s/2))
//                    verts.push_back((x-s/2,y+s/2,z-s/2))
//                    verts.push_back((x-s/2,y-s/2,z+s/2))
//                    verts.push_back((x-s/2,y-s/2,z-s/2))

//                    faces.push_back([0+o*8,1+o*8,3+o*8,2+o*8])
//                    faces.push_back([0+o*8,4+o*8,5+o*8,1+o*8])
//                    faces.push_back([0+o*8,2+o*8,6+o*8,4+o*8])
//                    faces.push_back([4+o*8,6+o*8,7+o*8,5+o*8])
//                    faces.push_back([1+o*8,3+o*8,7+o*8,5+o*8])
//                    faces.push_back([2+o*8,6+o*8,7+o*8,3+o*8])

//                    edges.push_back([0+o*8,1+o*8])
//                    edges.push_back([0+o*8,2+o*8])
//                    edges.push_back([0+o*8,4+o*8])
//                    edges.push_back([6+o*8,4+o*8])
//                    edges.push_back([6+o*8,2+o*8])
//                    edges.push_back([6+o*8,7+o*8])
//                    edges.push_back([5+o*8,1+o*8])
//                    edges.push_back([5+o*8,4+o*8])
//                    edges.push_back([5+o*8,7+o*8])
//                    edges.push_back([3+o*8,1+o*8])
//                    edges.push_back([3+o*8,2+o*8])
//                    edges.push_back([3+o*8,7+o*8])
                    leaf_counter+=1;
//                cout << "test" << endl;
            }
        }
        //        cout << "is Occupied?" << it->isNodeOccupied() << endl;
    }
}


int octomapToBlend::testPython(int argc, char *argv[], std::string module_name, std::string fnc_name){
    Py_SetProgramName(argv[0]);  /* optional but recommended */
    Py_Initialize();
    PySys_SetArgv(argc, argv); // must call this to get sys.argv and relative imports

    PyObject *pName, *pModule, *pDict, *pFunc;
    PyObject *pArgs, *pValue;
    int i;

//    if (argc < 3) {
//        fprintf(stderr,"Usage: call pythonfile funcname [args]\n");
//        return 1;
//    }

    pName = PyString_FromString(module_name.c_str());
    /* Error checking of pName left out */
    pModule = PyImport_Import(pName);
    Py_DECREF(pName);

    if (pModule != NULL) {
        pFunc = PyObject_GetAttrString(pModule, fnc_name.c_str());
        /* pFunc is a new reference */

        if (pFunc && PyCallable_Check(pFunc)) {
            cout << "now call py function" << endl;
//            pArgs = PyTuple_New(argc - 3);
//            for (i = 0; i < argc - 3; ++i) {
//                pValue = PyInt_FromLong(atoi(argv[i + 3]));
//                if (!pValue) {
//                    Py_DECREF(pArgs);
//                    Py_DECREF(pModule);
//                    fprintf(stderr, "Cannot convert argument\n");
//                    return 1;
//                }
//                /* pValue reference stolen here: */
//                PyTuple_SetItem(pArgs, i, pValue);
//            }
//            pValue = PyObject_CallObject(pFunc, pArgs);
//            Py_DECREF(pArgs);
//            if (pValue != NULL) {
//                printf("Result of call: %ld\n", PyInt_AsLong(pValue));
//                Py_DECREF(pValue);
//            }
//            else {
//                Py_DECREF(pFunc);
//                Py_DECREF(pModule);
//                PyErr_Print();
//                fprintf(stderr,"Call failed\n");
//                return 1;
//            }
        }
        else {
            if (PyErr_Occurred())
                PyErr_Print();
            fprintf(stderr, "Cannot find function \"%s\"\n", fnc_name.c_str());
        }
        Py_XDECREF(pFunc);
        Py_DECREF(pModule);
    }
    else {
        PyErr_Print();
        fprintf(stderr, "Failed to load \"%s\"\n", module_name.c_str());
        return 1;
    }
    Py_Finalize();
    return 0;

}
