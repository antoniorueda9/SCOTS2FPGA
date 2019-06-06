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

vector <int> read_input_vars(int state_dim, char* filename){
    static const int max_line = 65536;
    bool input_vars = false;
    int counter = 0;
    vector<int> u;
    ifstream file( filename );
    if (!file) {
        cerr << "Could not open file " << filename << endl;
        return {};
    }
    vector<string> tokens; // Storing all tokens
    string line;
    while (getline(file, line)){
        if (line.find("#END", 0) != string::npos) {
            input_vars = false;
        }
        if (input_vars){
            // cout << line << endl;
            u.push_back(stoi(line));
        }
        if (line.find("#VECTOR:BDD_VAR_ID_IN_DIM_" + to_string(state_dim+1+counter), 0) != string::npos) {
            input_vars = true;
            counter++;
            file.ignore(max_line, '\n');
        }

    }
    file.close();
    return u;
}

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
    const int size_of_inputs = readed_inputs.size();
    BDD s[size_of_inputs];

    int table_no = 0;
    //Remove the input and create two bdds for u=0 and for u=1
    for (int i = readed_inputs[0]; i < readed_inputs[0]+size_of_inputs; i++){
        //Create a cube to remove the desired input according to its value (u=1)
        DdNode *vars_to_remove[1] = {manager.ReadVars(i).getNode()};
        DdNode *cube = Cudd_bddComputeCube(manager.getManager(),vars_to_remove,NULL,1);
        BDD cube_bdd = BDD(manager,cube);
        vector<int> readed_inputs_cpy(size_of_inputs);
        copy(begin(readed_inputs), end(readed_inputs), begin(readed_inputs_cpy));
        remove(begin(readed_inputs_cpy), end(readed_inputs_cpy), i);

        //Create a cube to remove all the remaining bits of the input (u=X)
        DdNode *vars_to_remove2[size_of_inputs-1];
        for (int j = 0; j < size_of_inputs-1; j++)
             vars_to_remove2[j] = manager.ReadVars(readed_inputs_cpy[j]).getNode();
        DdNode *cube_all = Cudd_bddComputeCube(manager.getManager(),vars_to_remove2,NULL,size_of_inputs-1);
        BDD cube_bdd_all = BDD(manager,cube_all);

        //Restrict C to the value u1 = 1:
        s[table_no] = C & cube_bdd;
        //Delete variables from the originial bdd
        s[table_no] = s[table_no].ExistAbstract(cube_bdd_all);
        s[table_no] = s[table_no].ExistAbstract(cube_bdd);

        table_no++;
    }
    FILE *outfile; // output file pointer for .bdd file
    DdNode *ddnodearray0[size_of_inputs];
    for (int i=0; i<size_of_inputs; i++){
        ddnodearray0[i] = s[i].getNode();
    }
    // const char *inames[16] = {"x0","x1","x2","x3","x4","x5","x6","x7","x8","x9","x10","x11","x12","x13","x14","x15"};
    // const char *outnames[2] = {"y0","y1"};
    // char modname[30] = {"bdds"};
    outfile = fopen(filename.c_str(),"w");
    Cudd_DumpBlif(manager.getManager(), size_of_inputs, ddnodearray0, NULL, NULL, NULL, outfile,0); // dump the function to .dot file
    // free(ddnodearray0);
    fclose (outfile); // close the file */
    cout << filename << " file generated" << endl;

    // char filename3[30];
    // sprintf(filename3, "bdd.dot"); /*Write .dot filename to a string*/
    // FILE *outfile2; // output file pointer for .dot file
    // outfile2 = fopen(filename3,"w");
    // Cudd_DumpDot(manager.getManager(), size_of_inputs, ddnodearray0, NULL, NULL, outfile2); // dump the function to .dot file

    return 0;
}
