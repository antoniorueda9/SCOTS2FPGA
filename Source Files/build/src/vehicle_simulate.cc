/*
 * simulate.cc
 *
 *  created: Oct 2016
 *   author: Matthias Rungger
 */

/*
 * information about this example is given in
 * http://arxiv.org/abs/1503.03715
 * doi: 10.1109/TAC.2016.2593947
 */

#include <iostream>
#include <array>
#include <cmath>

/* SCOTS header */
#include "scots.hh"
/* ode solver */
#include "RungeKutta4.hh"
#include <stdlib.h> /* For EXIT_FAILURE, EXIT_SUCCESS */

/* sampling time */
const double tau = 0.3;

/* data types for the state space elements and input space
 * elements used in uniform grid and ode solvers */
using state_type = std::array<double,3>;
using input_type = std::vector<double>;

/* we integrate the vehicle ode by tau sec (the result is stored in x)  */
auto  vehicle_post = [](state_type& x, const input_type& u) {
  /* the ode describing the vehicle */
  auto rhs =[](state_type& xx,  const state_type& x, const input_type& u) {
    double alpha=std::atan(std::tan(u[1])/2.0);
    xx[0] = u[0]*std::cos(alpha+x[2])/std::cos(alpha);
    xx[1] = u[0]*std::sin(alpha+x[2])/std::cos(alpha);
    xx[2] = u[0]*std::tan(u[1]);
  };
  /* simulate (use 10 intermediate steps in the ode solver) */
  scots::runge_kutta_fixed4(rhs,x,u,3,tau,10);
};

//Implement this function in Labview
unsigned long int translate_state(state_type &x) {
    unsigned long int id = 0;
    double d_id[3];
    int vars[3];
    unsigned long int vars2_abs;
    unsigned long int vars3_abs;
    double m_first[3] ={0,0,-3.5};
    int m_dim = 3;
    double m_eta[3] = {0.2,0.2,0.2};
    for(int k=0; k<m_dim; k++) {
        d_id[k] = x[k]-m_first[k];
        vars[k] = lround(d_id[k]/m_eta[k]);
    }
    vars2_abs = static_cast<unsigned long int>(vars[2]) << 12;
    vars3_abs = static_cast<unsigned long int>(vars[1]) << 6;
    id = static_cast<unsigned long int>(vars[0]) | vars2_abs | vars3_abs;
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

int main() {

  /* Cudd manager */
  Cudd manager;

  /* define function to check if we are in target */
  auto target = [](const state_type& x) {
    if (9 <= x[0] && x[0] <= 9.5 && 0 <= x[1] && x[1] <= 0.5)
      return true;
    return false;
  };

  /* read controller from file */
  BDD C;
  scots::SymbolicSet con;
  if(!read_from_file(manager,con,C,"determinized")) {
    std::cout << "Could not read controller from determinized.scs\n";
    return 0;
  }


    //Remove the input and create two bdds for u=0 and for u=1
    DdNode *vars_to_remove[1] = {manager.ReadVars(18).getNode()};
    DdNode *u1 = Cudd_bddComputeCube(manager.getManager(),vars_to_remove,NULL,1);
    BDD uu1 = BDD(manager,u1);

    DdNode *vars_to_remove2[5] = {manager.ReadVars(19).getNode(),manager.ReadVars(20).getNode(),manager.ReadVars(21).getNode(),manager.ReadVars(22).getNode(),manager.ReadVars(23).getNode()};
    DdNode *u2 = Cudd_bddComputeCube(manager.getManager(),vars_to_remove2,NULL,5);
    BDD uu2 = BDD(manager,u2);

    //Restrict C to the value of u1=0 and u1=1:
    BDD s0 = C & !uu1;
    BDD s1 = C & uu1;

    s0 = s0.ExistAbstract(uu2);
    s0 = s0.ExistAbstract(uu1);
    s1 = s1.ExistAbstract(uu2);
    s1 = s1.ExistAbstract(uu1);

    FILE *outfile; // output file pointer for .bdd file
    char filename[30];
    DdNode *bdd;
    bdd = Cudd_BddToAdd(manager.getManager(), s0.getNode()); /*Convert BDD to ADD for display purpose*/
    DdNode *ddnodearray0[2];
    ddnodearray0[0] = bdd;
    sprintf(filename, "out.dot"); /*Write .dot filename to a string*/
    outfile = fopen(filename,"w");
    Cudd_DumpDot(manager.getManager(), 1, ddnodearray0, NULL, NULL, outfile); // dump the function to .dot file
    fclose (outfile); // close the file */





  std::cout << "\nSimulation:\n " << std::endl;

  state_type x={{0.6, 0.6, 0}};
  while(1) {
    /* returns a std vector with the valid control inputs */
    auto u = con.restriction(manager,C,x);
    std::cout << x[0] <<  " "  << x[1] << " " << x[2] << "\n";
    std::cout << u[0] <<  " "  << u[1] << "\n";

    unsigned long int id = translate_state(x);
    BDD evals0,evals1;
    bool stateBits1[18];
	int_to_bool(id, 18, stateBits1);
    std::reverse(&stateBits1[0],&stateBits1[18]);
    int inputs[18] = { stateBits1[13], stateBits1[14], stateBits1[15], stateBits1[16], stateBits1[17],
        stateBits1[7], stateBits1[8], stateBits1[9], stateBits1[10], stateBits1[11], stateBits1[12],
        stateBits1[0], stateBits1[1], stateBits1[2], stateBits1[3], stateBits1[4], stateBits1[5], stateBits1[6]};

    std::cout << "s0 = "  << s0.Eval(inputs) << std::endl;
    std::cout << "s1 = "  << s1.Eval(inputs) << std::endl;

    vehicle_post(x,u);
    if(target(x)) {
      std::cout << "Arrived: " << x[0] <<  " "  << x[1] << " " << x[2] << std::endl;
      break;
    }
  }

  return 1;
}
