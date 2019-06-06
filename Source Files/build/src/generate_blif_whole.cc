/*
This example is reading a determinized controller (created by Ivan's tools)
, splitting it according to the control input value and generating the .blif file
*/


#include <iostream>
#include <array>

/* SCOTS header */
#include "scots.hh"

using namespace std;
using namespace scots;

int main(int argc, char* argv[]){

    if (argc < 4) {
        std::cerr << "Usage: " << argv[0] << " <source controller> <target blif> <state_space_dim>" << std::endl;
        return 1;
    }
    cout << "\n\nSplitting controller and generating blif file" << endl;
    /* Cudd manager */
    Cudd manager;
    char filename2[100];
    strcpy(filename2,argv[1]);
    strcat(filename2,".scs");
    string filename  = argv[2];
    int state_dim = atoi(argv[3]);
    // int input_dim = atoi(argv[4]);
    vector <int> readed_inputs = read_input_vars(state_dim, filename2);

    /* read controller from file */
    BDD C;
    scots::SymbolicSet controller;
    if(!read_from_file(manager,controller,C,argv[1])) {
        std::cout << "Could not read determinized from determinized.scs\n";
        return 0;
    }
    controller.print_info(1);


    FILE *outfile; // output file pointer for .bdd file
    DdNode *ddnodearray0[1];

    ddnodearray0[0] = C.getNode();
    outfile = fopen(filename.c_str(),"w");
    Cudd_DumpBlif(manager.getManager(), 1, ddnodearray0, NULL, NULL, NULL, outfile,0);
    fclose (outfile); // close the file */
    cout << filename << " file generated" << endl;

    return 0;
}
