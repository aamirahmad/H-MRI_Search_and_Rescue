#include "octomaptoblend.h"

using namespace octomap;
using namespace std;

octomapToBlend::octomapToBlend()
{
    this->_init = false;
    this->_finalize = false;
}

octomapToBlend::~octomapToBlend(){
    if(!this->_finalize){
        this->finalize();
    }
}

#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#define PY_ARRAY_UNIQUE_SYMBOL damaris_ARRAY_API

using namespace std;

/**
 * @brief init_numpy warpper function to import the numpy array
 * @return
 */

void* octomapToBlend::init_numpy() {
    PyOS_sighandler_t sighandler = PyOS_getsig(SIGINT);
    import_array();
    PyOS_setsig(SIGINT,sighandler);
    return NULL;
}
/** @brief makeNpArray<T> convert a c_array (1D) to an np array
 * @param c_array data buffer
 * @param size number of entries
 * @return PyArrayObject*
 */
template <class T>
PyArrayObject* octomapToBlend::makeNpArray(T c_array[], size_t size) {
    PyArrayObject *np_arg;

    npy_intp dims[1]{(const int) size};
    const int ND{ 1 };

    if (typeid(T) == typeid(int)){
        np_arg = reinterpret_cast<PyArrayObject*>(PyArray_SimpleNewFromData(ND, dims, NPY_INT,
                                                                            reinterpret_cast<void*>(c_array)));
    }else if (typeid(T) == typeid(float)){
        np_arg = reinterpret_cast<PyArrayObject*>(PyArray_SimpleNewFromData(ND, dims, NPY_FLOAT,
                                                                            reinterpret_cast<void*>(c_array)));
    }else if (typeid(T) == typeid(double)){
        np_arg = reinterpret_cast<PyArrayObject*>(PyArray_SimpleNewFromData(ND, dims, NPY_DOUBLE,
                                                                            reinterpret_cast<void*>(c_array)));
    }

    return np_arg;
}


/**
 * @brief makelist<T> convert a c array to a python array, currently implemented is the conversion for double and int
 * @param array c _style array of type double or int
 * @param size number of elements in the array
 * @return PyObject python array
 */

template <class T> PyObject *octomapToBlend::makelist(T array[], size_t size) {
    PyObject *l = PyList_New(size);
    for (size_t i = 0; i != size; ++i) {
        if (typeid(T) == typeid(int)){
            PyList_SET_ITEM(l, i, PyLong_FromLong(array[i]));
        }else if(typeid(T) == typeid(double)){
            PyList_SET_ITEM(l, i, PyFloat_FromDouble(array[i]));
        }

    }
    return l;
}
/**
 * @brief octomapToBlend::init init the python interpreter and import numpy
 */
void octomapToBlend::init(){
    Py_Initialize();
    this->init_numpy();
    this->_init = true;
}

/**
 * @brief octomapToBlend::finalize end the python interpreter
 */
void octomapToBlend::finalize(){
    Py_Finalize();
}

/**
 * @brief main test the embedded python interface
 * @param argc argument count
 * @param argv argument vector
 * @return int return value
 */
int octomapToBlend::callPythonClassFunction(double *c_arr_verts, size_t verts_len, string filename){

    if(!this->_init){
        std::perror("Did you forget to call octomap::init()?");
        return 1;
    }
    PyObject *pName, *pModule, *pDict, *pFunc,*pClass;
    PyObject *pArgs, *pValue, *pInstance, *pFileName,*pArgList;
    PyArrayObject *pNPArray;

    PyRun_SimpleString("from time import time,ctime\n"
                       "print('Today is', ctime(time()))\n");

    pName = PyUnicode_DecodeFSDefault("createMesh");
    pFileName = PyUnicode_DecodeFSDefault(filename.c_str());
    /* Error checking of pName left out */
    pModule = PyImport_Import(pName);
    Py_DECREF(pName);

    if (pModule != NULL) {
        // pDict is a borrowed reference
        pDict = PyModule_GetDict(pModule);


        // Build the name of a callable class
        pClass = PyDict_GetItemString(pDict, "MeshFromOcTree");
        //        Py_DECREF(pDict);
        // Create an instance of the class
        if (PyCallable_Check(pClass))
        {
            // get the Instance of the Class with call object
            pInstance = PyObject_CallObject(pClass, NULL);

            pFunc = PyObject_GetAttrString(pInstance, "meshDataToBlendFile");
            /* pFunc is a new reference */
            if (pFunc && PyCallable_Check(pFunc)) {




                // init the args tuple with three entries
                pArgs = PyTuple_New(2);
                // fill the args tuple with the list values
                pNPArray = this->makeNpArray(c_arr_verts,verts_len);
                PyTuple_SetItem(pArgs, 0,reinterpret_cast<PyObject*>(pNPArray));

                PyTuple_SetItem(pArgs, 1, pFileName);

                //                pNPArray = this->makeNpArray(c_arr_edges,edges_len);
                //                PyTuple_SetItem(pArgs, 1, reinterpret_cast<PyObject*>(pNPArray));

                //                pNPArray = this->makeNpArray(c_arr_faces,faces_len);
                //                PyTuple_SetItem(pArgs, 2, reinterpret_cast<PyObject*>(pNPArray));


                // call the cuntion with the prev set arguments
                pValue = PyObject_CallObject(pFunc, pArgs);

                //                                Py_DECREF(pArgs);
                //                                Py_DECREF(pNPArray);

                if (pValue != NULL) {
                    printf("Result of call: %ld\n", PyLong_AsLong(pValue));
                    Py_DECREF(pValue);
                }
                else {
                    Py_DECREF(pFunc);
                    Py_DECREF(pModule);
                    PyErr_Print();
                    fprintf(stderr,"Call failed\n");
                    return 1;
                }
            }
            else {
                if (PyErr_Occurred())
                    PyErr_Print();
                fprintf(stderr, "Cannot find function \"%s\"\n", "meshDataToBlendFile");
            }
            Py_XDECREF(pFunc);
            Py_DECREF(pModule);
        }
        else {
            PyErr_Print();
            fprintf(stderr, "Failed to load \"%s\"\n", "MeshFromOcTree");
            return 1;
        }
    }
    else {
        PyErr_Print();
        fprintf(stderr, "Failed to load \"%s\"\n", "createMesh");
        return 1;
    }

    return 0;

}
/**
 * @brief octomapToBlend::createMeshData provided with an octomap::OcTree this functions calls
 * a python module to convert the tree into a blend file
 * @param tree octomap::OcTree
 * @return bool wheather or not the tree has at least one occupied node
 */
int octomapToBlend::createMeshData(octomap::OcTree *tree, int depth, std::string filename, Origin *origin){
    //    tree->expand();
    // sanity check
    if (tree->getNumLeafNodes() < 1){
        return false;
    }
    clock_t cstart;
    cstart = clock();

    size_t tree_leaf_length = (size_t) tree->getNumLeafNodes();

    cout << "number of leave nodes " <<  tree_leaf_length << " wish to allocate " << tree->getNumLeafNodes()*8*3*sizeof(double) << " bytes" << endl;


    int leaf_counter = 0;

    // find out if there are occupied nodes in the tree
    int numOccupiedNodes = 0;

    cstart = clock();

    for (OcTree::tree_iterator it = tree->begin_tree();  it != tree->end_tree(); ++it)
    {
        if(it.getDepth()+(16-6) == depth){

            if(tree->isNodeOccupied((*it)) ){
                numOccupiedNodes += 1;
            }
        }
    }
    std::cout <<"\t getNumNodes took " << float(clock()-cstart)/float(CLOCKS_PER_SEC) << std::endl;
    if(numOccupiedNodes < 4){
        return numOccupiedNodes;
    }

    size_t len_verts = numOccupiedNodes*8*3;
    double *verticies = new double[len_verts];
    numOccupiedNodes = 0;


    double tree_size = tree->begin_tree().getSize();

    // BAD HACK GET THE SUBTREE DEPTH CORRECT
    int sub_tree_depth = 10;

    double sub_tree_size = tree_size/pow(2,sub_tree_depth);;
    double scaling = sub_tree_size/tree_size;

    for (OcTree::tree_iterator it = tree->begin_tree();  it != tree->end_tree(); ++it)
    {

        // this is a bad hack; 10 is the level in which the subtrees are rooted

        if(it.getDepth()+(16-6) == depth){

            if(tree->isNodeOccupied((*it)) ){


                numOccupiedNodes += 1;


                //                double x = origin->x+it.getX()/power;
                //                double y = origin->y+it.getY()/power;
                //                double z = origin->z+it.getZ()/power;

                double x = origin->x+it.getX()*scaling;
                double y = origin->y+it.getY()*scaling;
                double z = origin->z+it.getZ()*scaling;
                double s = it.getSize()*scaling;

                //                cout << "origin " << origin->x << " " << origin->y << " " << origin->z << endl;
                //                cout << origin->x-x << " " << origin->y-y << " " << origin->z-z << endl;
                //                cout << "x " << x << " y " << y << " z " << z <<  " s " << s << " " << it.getDepth()+10<< endl;

                //                 verts.push_back(it.getCoordinate())
                //                 static const double  arr[] = {x+s/2,y+s/2,z+s/2};
                verticies[(leaf_counter*(8*3))+0]= x+s/2;// = {x+s/2,y+s/2,z+s/2};
                verticies[(leaf_counter*(8*3))+1]= y+s/2;
                verticies[(leaf_counter*(8*3))+2]= z+s/2;

                verticies[(leaf_counter*(8*3))+3]= x+s/2;
                verticies[(leaf_counter*(8*3))+4]= y+s/2;
                verticies[(leaf_counter*(8*3))+5]= z-s/2;

                verticies[(leaf_counter*(8*3))+6]= x+s/2;
                verticies[(leaf_counter*(8*3))+7]= y-s/2;
                verticies[(leaf_counter*(8*3))+8]= z+s/2;

                verticies[(leaf_counter*(8*3))+9]= x+s/2;
                verticies[(leaf_counter*(8*3))+10]= y-s/2;
                verticies[(leaf_counter*(8*3))+11]= z-s/2;

                verticies[(leaf_counter*(8*3))+12]= x-s/2;
                verticies[(leaf_counter*(8*3))+13]= y+s/2;
                verticies[(leaf_counter*(8*3))+14]= z+s/2;

                verticies[(leaf_counter*(8*3))+15]= x-s/2;
                verticies[(leaf_counter*(8*3))+16]= y+s/2;
                verticies[(leaf_counter*(8*3))+17]= z-s/2;

                verticies[(leaf_counter*(8*3))+18]= x-s/2;
                verticies[(leaf_counter*(8*3))+19]= y-s/2;
                verticies[(leaf_counter*(8*3))+20]= z+s/2;

                verticies[(leaf_counter*(8*3))+21]= x-s/2;
                verticies[(leaf_counter*(8*3))+22]= y-s/2;
                verticies[(leaf_counter*(8*3))+23]= z-s/2;




                leaf_counter+=1;


            }

        }
        //        cout << "is Occupied?" << it->isNodeOccupied() << endl;
    } // end leaf iterator
    cout << "number of occupied nodes " << numOccupiedNodes <<endl;
    std::cout <<"\t generating mesh data took " << float(clock()-cstart)/float(CLOCKS_PER_SEC) << std::endl;

    cstart = clock();
    cout << "=========================================================================" << endl;
    this->callPythonClassFunction(verticies,len_verts,filename);
    cout << "+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++" << endl;
    std::cout <<"\t the python part took " << float(clock()-cstart)/float(CLOCKS_PER_SEC) << std::endl;

    delete verticies;

    return numOccupiedNodes;
}


