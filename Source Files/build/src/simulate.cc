/*
 * simulate.cc
 *
 *  created: Jan 2016
 *   author: Matthias Rungger
 */

/*
 * information about this example is given in
 * http://arxiv.org/abs/1503.03715
 * doi: 10.1109/TAC.2016.2593947
 */
/*
This example is reading a determinized controller (created by Ivan's tools)
and simulating the closed loop using SCOTSv0.2

- The control action obtained by the restriction function DOES match the
control action obtained from the splitted bdd

*/


#include <iostream>
#include <array>

/* SCOTS header */
#include "scots.hh"

/* ode solver */
#include "RungeKutta4.hh"
#include <stdlib.h> /* For EXIT_FAILURE, EXIT_SUCCESS */


/* state space dim */
const int state_dim=2;
const int input_dim=1;

/* sampling time */
const double tau = 0.5;

/* data type used by the ODE solver */
using state_type = std::array<double,state_dim>;
using input_type = std::array<double,input_dim>;

using namespace std;
using namespace scots;
/* parameters for system dynamics */
const double xc=70;
const double xl=3;
const double rc=0.005;
const double rl=0.05;
const double ro=1;
const double vs=1;

/* we integrate the dcdc ode by 0.5 sec (the result is stored in x)  */
auto system_post = [](state_type &x, const double &u) noexcept {
  /* the ode describing the dcdc converter */
  auto rhs =[](state_type& xx,  const state_type &x, const double &u) noexcept {
    if(u==1) {
      xx[0]=-rl/xl*x[0]+vs/xl;
      xx[1]=-1/(xc*(ro+rc))*x[1];
    } else {
      xx[0]=-(1/xl)*(rl+ro*rc/(ro+rc))*x[0]-(1/xl)*ro/(5*(ro+rc))*x[1]+vs/xl;
      xx[1]=(1/xc)*5*ro/(ro+rc)*x[0]-(1/xc)*(1/(ro+rc))*x[1];
    }
	};
  scots::runge_kutta_fixed4(rhs,x,u,state_dim,tau);
};

//Implement this function in Labview
unsigned long int translate_state(state_type &x) {
    unsigned long int id = 0;
    double d_id[2];
    int vars[2];
    unsigned long int vars2_abs;
    double m_first[2] ={0.65,4.95};
    int m_dim = 2;
    double m_eta[2] = {0.005,0.005};
    for(int k=0; k<m_dim; k++) {
        d_id[k] = x[k]-m_first[k];
        vars[k] = lround(d_id[k]/m_eta[k]);
    }
    vars2_abs = static_cast<unsigned long int>(vars[1]) << 8;
    id = static_cast<unsigned long int>(vars[0]) | vars2_abs;
    return id;
}

//Function obtained from BDD2Implement
void int_to_bool(unsigned long int in, int count, bool* out){
    unsigned long int mask = 1L;
    int i;
    for (i = 0; i < count; i++) {
        out[i] = (in & mask) ? true : false;
        in >>= 1;
    }
}

int main(int argc, char* argv[]){
    // NtrOptions *option;
    // option = mainInit();
    // FILE *fp1;
    // BnetNetwork *net1 = NULL;
    // fp1 = fopen("C17.blif","r");
    // net1 = Bnet_ReadNetwork(fp1,1);
    // fclose(fp1);
    //
    // DdManager *dd;
    // dd = Cudd_Init(0,0,CUDD_UNIQUE_SLOTS,CUDD_CACHE_SLOTS,0);
    // Cudd_AutodynEnable(dd,CUDD_REORDER_SIFT);
    // int result;
    // //result = Ntr_buildDDs(net1,dd,option,NULL);
    //
    // BnetNode *node;
    // DdNode **outputArray; /* output array to store BDD for each output */
    // outputArray = (DdNode**)malloc(6 * sizeof(DdNode*));
    // int outCount = 0;
    //   for (node = net1->nodes; outCount < 6; node = node->next) {
    //   if (node->type == BNET_OUTPUT_NODE){
    //           outputArray[outCount] = node->dd;
    //           outCount = outCount + 1;
    //       }
    //   }

    ofstream datafile;
    datafile.open(argv[2]);
    cout << "\n\nSplitting controller" << endl;
    /* Cudd manager */
    Cudd manager;
    /* read controller from file */
    BDD C;
    scots::SymbolicSet controller;
    if(!read_from_file(manager,controller,C,argv[1])) {
        std::cout << "Could not read determinized from determinized.scs\n";
        return 0;
    }
    int n_input_bits = atoi(argv[3]);
    int total_vars = Cudd_ReadSize(manager.getManager());

    //Remove the input and create two bdds for u=0 and for u=1
    BDD vars = manager.ReadVars(total_vars-n_input_bits);
    DdNode *vars_to_remove[1] = {vars.getNode()};
    DdNode *x1 = Cudd_bddComputeCube(manager.getManager(),vars_to_remove,NULL,1);
    BDD xx = BDD(manager,x1);
    BDD s0 = C & !xx;
    BDD s1 = C & xx;
    s0 = s0.ExistAbstract(xx);
    s1 = s1.ExistAbstract(xx);



    std::cout << "\nSimulation:\n " << std::endl;
    cout.precision(4);
    state_type x={{0.7, 5.4}};
    std::cout << "    ite |      x1 |      x2 |      s0 |      s1 | u_scots | u_splitted " << std::endl;
    for(int i=0; i<1000; i++) {
        /* returns a std vector with the valid control inputs */
        auto u = controller.restriction(manager,C,x);
        unsigned long int id = translate_state(x);
        BDD evals0,evals1,evals_c;
        bool stateBits1[16];
    	int_to_bool(id, 16, stateBits1);
        reverse(&stateBits1[0],&stateBits1[16]);

        int u2 = (int)(u[0]-1);
        int u3;

        int inputs[16] = {stateBits1[8], stateBits1[9], stateBits1[10], stateBits1[11], stateBits1[12], stateBits1[13], stateBits1[14], stateBits1[15], stateBits1[0], stateBits1[1], stateBits1[2], stateBits1[3], stateBits1[4], stateBits1[5], stateBits1[6], stateBits1[7]};
        int inputs2[17] = {stateBits1[8], stateBits1[9], stateBits1[10], stateBits1[11], stateBits1[12], stateBits1[13], stateBits1[14], stateBits1[15], stateBits1[0], stateBits1[1], stateBits1[2], stateBits1[3], stateBits1[4], stateBits1[5], stateBits1[6], stateBits1[7],u2};

        evals_c = C.Eval(inputs2);
        evals0 = s0.Eval(inputs);
        evals1 = s1.Eval(inputs);

        if (evals0.IsOne()&&!evals1.IsOne())
            u3 = 0;
        else if (evals1.IsOne()&&!evals0.IsOne())
            u3 = 1;
        else{
            std::cout << "Input not defined" << std::endl;
            return 1;
        }

        std::cout << setw(7) << right << i << " | " << setw(7) << right << x[0] << " | " << setw(7) << right << x[1]
        << " | " << setw(7) << right << evals0.IsOne() << " | " << setw(7) << right << evals1.IsOne()
        << " | " << setw(7) << right << u[0]-1 << " | " << setw(7) << right << u3 << endl;

        if (!evals_c.IsOne()){
            cout << "C not 1!" <<  endl;
            return 1;
        }
        if (u3 != (u[0]-1)){
            cout << "Different control input generated!" <<  endl;
            return 1;
        }
        datafile << i << "  " << x[0] << "  " << x[1] << "  " << u3 << endl;
        system_post(x,u3+1);
    }
    datafile.close();
    return 0;
}
