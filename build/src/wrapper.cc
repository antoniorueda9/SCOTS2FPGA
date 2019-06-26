/*
   Author:        Antonio Rueda
   Date:          26/06/2019
   University:    TUDelft
   Description:   This wrapper is geterating the .vhd file necessary to import a
   controller in LabVIEW. Based on the mechanism to created text files from
   https://gitlab.lrz.de/hcs/BDD2Implement
 */

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <algorithm>
#include <vector>
#include "wrapper.hh"

using namespace std;

int main(int argc, char* argv[]){

    if (argc < 3) {
        std::cerr << "Usage: " << argv[0] << " <source verilog> <target vhd>" << std::endl;
        return 1;
    }

    cout << "\n\nGenerating VHDL wrapper for" << argv[1] << endl;
    const string template_file = "../build/src/templates/template.vhd";

	const char* filename   = argv[1];

	ifstream file( filename );
	if (!file) {
		cerr << "Could not open file " << filename << endl;
		return 1;
	}

    //Storing module, inputs and outputs names in vectors
    vector<string> vv_modname = Get_ModName(file);
    vector<string> vv_inputs = Get_Inputs(file);
    vector<string> vv_outputs = Get_Outputs(file);

    /*Using the functions from wrapper.hh we will find the next strings in a template
    and replace them with the names from the previous vectors*/
    stringstream modelname, inPorts, inPortsMap, outPorts, outPortsMap;
    for (vector<string>::const_iterator i = vv_modname.begin()+1; i != vv_modname.end()-1; ++i)
        modelname << *i;
    for (vector<string>::const_iterator i = vv_inputs.begin()+1; i != vv_inputs.end(); ++i){
        inPorts << *i << " : in STD_LOGIC;" << endl;
        inPortsMap << *i << " => " << *i << "," << endl;
    }
    for (vector<string>::const_iterator i = vv_outputs.begin()+1; i != vv_outputs.end(); ++i){
        outPorts << *i << " : out STD_LOGIC;" << endl;
        outPortsMap << *i << " => " << *i << "," << endl;
    }
    outPorts.seekp(outPorts.str().length()-2);
    outPorts << " ";
    outPortsMap.seekp(outPortsMap.str().length()-2);
    outPortsMap << " ";

    string templateText = ReadAllFileText(template_file);
    string OutText = ReplaceString(templateText,"#$ENTITY_MODEL_NAME$#",  modelname.str());
    OutText = ReplaceString(OutText,"#$ENTITY_INPUT_PORTS$#",  inPorts.str());
    OutText = ReplaceString(OutText,"#$ENTITY_INPUT_PORTS_MAP$#",  inPortsMap.str());
    OutText = ReplaceString(OutText,"#$ENTITY_OUTPUT_PORTS$#", outPorts.str());
    OutText = ReplaceString(OutText,"#$ENTITY_OUTPUT_PORTS_MAP$#", outPortsMap.str());
    OutText = ReplaceString(OutText, "#$DATES$#", GetCurrentDateTime());

    const string outFilename = argv[2] + modelname.str() + "_Wrapper.vhd";

    FileWriteAllText(outFilename, OutText);

    cout << outFilename << " created" << endl;

	return 0;

}
