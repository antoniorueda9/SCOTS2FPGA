/*
   Author:        Antonio Rueda
   Date:          26/06/2019
   University:    TUDelft
   Description:   This example is reading a determinized controller (created by Ivan's tools),
   splitting it according to the control input value and generating the .blif file
 */



#include <iostream>
#include <array>
#include "scots.hh"
#include <time.h>
#include <sys/time.h>
#include <sys/resource.h>

using namespace std;
using namespace scots;

/*The read_input_vars fucntion look into the .scs file and search for the number
   of inputs, storing them and returning them in a vector */
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
	while (getline(file, line)) {
		if (line.find("#END", 0) != string::npos) {
			input_vars = false;
		}
		if (input_vars) {
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
	clock_t start, end_time;
	double cpu_time_used;
	vector <int> readed_inputs = read_input_vars(state_dim, filename2);

	//Read controller from file
	BDD C;
	scots::SymbolicSet controller;
	if(!read_from_file(manager,controller,C,argv[1])) {
		std::cout << "Could not read determinized from determinized.scs\n";
		return 0;
	}
	controller.print_info(1);
	const int size_of_inputs = readed_inputs.size();
	BDD s[size_of_inputs+1];

	//Profiling
    //////////////////////////////////////////////////////////////////////////
	start = clock();
	int table_no = 0;
	//Remove the input and create two bdds for u=0 and for u=1
	for (int i = readed_inputs[0]; i < readed_inputs[0]+size_of_inputs; i++) {
		//Create a cube to remove the desired input according to its value (u=1)
		DdNode *vars_to_remove[1] = {manager.ReadVars(i).getNode()};
		DdNode *cube = Cudd_bddComputeCube(manager.getManager(),vars_to_remove,NULL,1);
		BDD cube_bdd = BDD(manager,cube);

		//Create a copy of the input vector in order to modify it for the next iteration
		vector<int> readed_inputs_cpy(size_of_inputs);
		copy(begin(readed_inputs), end(readed_inputs), begin(readed_inputs_cpy));
		//Remove the i variable of the vector so we have a vector containing all vars
		//except for the i var.
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
	//Add BDD removing all inputs so we can check if we are in the domain of the controller
	DdNode *vars_to_remove3[size_of_inputs];
	for (int j = 0; j < size_of_inputs; j++)
		vars_to_remove3[j] = manager.ReadVars(readed_inputs[j]).getNode();

	DdNode *cube_complete = Cudd_bddComputeCube(manager.getManager(),vars_to_remove3,NULL,size_of_inputs);
	BDD cube_bdd_complete = BDD(manager,cube_complete);
	s[size_of_inputs] = C.ExistAbstract(cube_bdd_complete);;

	FILE *outfile; // output file pointer for .bdd file
	//Dump the BDD to a DdNode array
	DdNode *ddnodearray0[size_of_inputs+1];
	for (int i=0; i<size_of_inputs+1; i++) {
		ddnodearray0[i] = s[i].getNode();
	}

	//Dump the DdNode array to a .blif file
	outfile = fopen(filename.c_str(),"w");
	Cudd_DumpBlif(manager.getManager(), size_of_inputs+1, ddnodearray0, NULL, NULL, NULL, outfile,0); // dump the function to .dot file
	// free(ddnodearray0);
	fclose (outfile); // close the file */
	cout << filename << " file generated" << endl;
	end_time = clock();
	//////////////////////////////////////////////////////////////////////////

	cpu_time_used = ((double) (end_time - start)) / CLOCKS_PER_SEC;
	cout << "CPU_Time_used =  " << cpu_time_used << endl;

	return 0;
}
