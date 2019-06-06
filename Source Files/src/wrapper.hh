#include <cctype>
#include <iterator>
#include <cstring>
#include <time.h>

using namespace std;

struct tokens : ctype<char>
{
	tokens() : ctype<char>(get_table()) {
	}

	static ctype_base::mask const* get_table()
	{
		typedef ctype<char> cctype;
		static const cctype::mask *const_rc= cctype::classic_table();

		static cctype::mask rc[cctype::table_size];
		memcpy(rc, const_rc, cctype::table_size * sizeof(cctype::mask));

		rc[','] = ctype_base::space;
		rc[' '] = ctype_base::space;
		return &rc[0];
	}
};

std::string GetCurrentDateTime() {
    time_t     now = time(0);
    struct tm  tstruct;
    char       buf[80];
    tstruct = *localtime(&now);
    strftime(buf, sizeof(buf), "%Y-%m-%d.%X", &tstruct);
    return buf;
}

vector<string> Get_ModName(ifstream & file){
    string line;
    while ( getline( file, line, '\n' ) ) {
		if ( line.find( "module" ) != string::npos ){
			stringstream ss(line);
			ss.imbue(locale(locale(), new tokens()));
			istream_iterator<string> begin(ss);
			istream_iterator<string> end;
		    vector<string> v_modname(begin, end);
            return v_modname;
		}
    }
    return vector<string>();
}

vector<string> Get_Inputs(ifstream & file){
    string line;
    while ( getline( file, line, ';' ) ) {
		if ( line.find( "input" ) != string::npos ){
			stringstream ss(line);
			ss.imbue(locale(locale(), new tokens()));
			istream_iterator<string> begin(ss);
			istream_iterator<string> end;
		    vector<string> v_inputs(begin, end);
            return v_inputs;
		}
    }
    return vector<string>();
}

vector<string> Get_Outputs(ifstream & file){
    string line;
    while ( getline( file, line, ';' ) ) {
		if ( line.find( "output" ) != string::npos ){
			stringstream ss(line);
			ss.imbue(locale(locale(), new tokens()));
			istream_iterator<string> begin(ss);
			istream_iterator<string> end;
		    vector<string> v_outputs(begin, end);
            return v_outputs;
		}
    }
    return vector<string>();
}

string ReadAllFileText(const string& fname){

    ifstream inFile;
    inFile.open(fname);

    stringstream strStream;
    strStream << inFile.rdbuf();//read the file
    return strStream.str();//str holds the content of the file
}

string ReplaceString(string subject, const string& search,
                          const string& replace) {
    size_t pos = 0;
    while ((pos = subject.find(search, pos)) != string::npos) {
         subject.replace(pos, search.length(), replace);
         pos += replace.length();
    }
    return subject;
}

void FileWriteAllText(const std::string& fname, string data){
    std::ofstream out(fname);
    out << data;
    out.close();
}
