/*
   Author:        Antonio Rueda
   Date:          26/06/2019
   University:    TUDelft
   Description:   This example is reading a determinized controller (created by Ivan's tools),
   and generating the complete .blif file without modifying the BDD
 */


#include <iostream>
#include <array>
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
    clock_t start, end_time;
    double cpu_time_used;

    //Read controller from file
    BDD C;
    scots::SymbolicSet controller;
    if(!read_from_file(manager,controller,C,argv[1])) {
        std::cout << "Could not read determinized from determinized.scs\n";
        return 0;
    }
    controller.print_info(1);

    //Profiling
    //////////////////////////////////////////////////////////////////////////
    start = clock();

    FILE *outfile; // output file pointer for .bdd file
    //Dump the BDD to a DdNode array
    DdNode *ddnodearray0[1];
    ddnodearray0[0] = C.getNode();
    //Dump the DdNode array to a .blif file
    outfile = fopen(filename.c_str(),"w");
    Cudd_DumpBlif(manager.getManager(), 1, ddnodearray0, NULL, NULL, NULL, outfile,0);
    fclose (outfile); // close the file */
    cout << filename << " file generated" << endl;
    end_time = clock();
    //////////////////////////////////////////////////////////////////////////

    cpu_time_used = ((double) (end_time - start)) / CLOCKS_PER_SEC;
    cout << "CPU_Time_used =  " << cpu_time_used << endl;
    return 0;
}
