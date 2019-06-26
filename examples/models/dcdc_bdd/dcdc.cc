/*
 * dcdc.cc
 *
 *  created: Jan 2017
 *   author: Matthias Rungger
 */

/* information about this example is given in the readme file */

#include <iostream>
#include <array>
#include <cmath>

/* SCOTS header */
#include "scots.hh"
/* ode solver */
#include "RungeKutta4.hh"


/* time profiling */
#include "TicToc.hh"
/* memory profiling */
#include <sys/time.h>
#include <sys/resource.h>
struct rusage usage;


/* state space dim */
const int state_dim=2;
/* input space dim */
const int input_dim=1;
/* sampling time */
const double tau = 0.5;

/*
 * data types for the elements of the state space
 * and input space used by the ODE solver
 */
using state_type = std::array<double,state_dim>;
using input_type = std::array<double,input_dim>;

/* abbrev of the type for abstract states and inputs */
using abs_type = scots::abs_type;

/* parameters for system dynamics */
const double xc=70;
const double xl=3;
const double rc=0.005;
const double rl=0.05;
const double ro=1;
const double vs=1;
/* parameters for radius calculation */
const double mu=std::sqrt(2);
/* we integrate the dcdc ode by 0.5 sec (the result is stored in x)  */
auto system_post = [](state_type &x, const input_type &u) noexcept {
  /* the ode describing the dcdc converter */
  auto rhs =[](state_type& xx,  const state_type &x, const input_type &u) noexcept {
    if(u[0]==1) {
      xx[0]=-rl/xl*x[0]+vs/xl;
      xx[1]=-1/(xc*(ro+rc))*x[1];
    } else {
      xx[0]=-(1/xl)*(rl+ro*rc/(ro+rc))*x[0]-(1/xl)*ro/(5*(ro+rc))*x[1]+vs/xl;
      xx[1]=(1/xc)*5*ro/(ro+rc)*x[0]-(1/xc)*(1/(ro+rc))*x[1];
    }
	};
  scots::runge_kutta_fixed4(rhs,x,u,state_dim,tau);
};
/* we integrate the growth bound by 0.5 sec (the result is stored in r)  */
auto radius_post = [](state_type &r, const state_type&, const input_type &u) noexcept {
  /* the ode for the growth bound */
  auto rhs =[](state_type& rr,  const state_type &r, const input_type &u) noexcept {
    if(u[0]==1) {
      rr[0]=-rl/xl*r[0];
      rr[1]=-1/(xc*(ro+rc))*r[1];
    } else {
      rr[0]=-(1/xl)*(rl+ro*rc/(ro+rc))*r[0]+(1/xl)*ro/(5*(ro+rc))*r[1];
      rr[1]=5*(1/xc)*ro/(ro+rc)*r[0]-(1/xc)*(1/(ro+rc))*r[1];
    }
	};
  scots::runge_kutta_fixed4(rhs,r,u,state_dim,tau);
};

int main() {
  /* to measure time */
  TicToc tt;
  /* BDD manager */
  Cudd manager;

  /* enable variable reordering */
  // manager.AutodynEnable();

  /* setup the workspace of the synthesis problem and the uniform grid */
  /* grid node distance diameter */
  state_type eta={{20.0/4e3,20.0/4e3}};
  /* lower bounds of the hyper-rectangle */
  state_type lb={{0.649,4.949}};
  /* upper bounds of the hyper-rectangle */
  state_type ub={{1.65,5.95}};
  scots::SymbolicSet ss_pre(manager,state_dim,lb,ub,eta);
  std::cout << "Unfiorm grid details:\n";
  ss_pre.print_info();

  /* construct grid for the input alphabet */
  /* hyper-rectangle [1,2] with grid node distance 1 */
  scots::SymbolicSet ss_input(manager,input_dim,input_type{{.99}},input_type{{2.1}},input_type{{1}});
  ss_input.print_info();
  scots::SymbolicSet controller(ss_pre,ss_input);
  scots::SymbolicSet ss_post(manager,state_dim,lb,ub,eta);

  /* compute transition function of symbolic model */
  std::cout << "Computing the transition function:\n";
  /* SymbolicModel class to compute the BDD encoding the transition function */
  scots::SymbolicModel<state_type,input_type> sym_model(ss_pre,ss_input,ss_post);

  tt.tic();
  size_t no_trans;
  BDD TF = sym_model.compute_gb(manager,system_post,radius_post,no_trans);
  tt.toc();

  std::cout << "No of Transitions " << no_trans  << "\n";
  if(!getrusage(RUSAGE_SELF, &usage)) {
    std::cout << "Memory pro Transition: " << usage.ru_maxrss/(double)no_trans<< "\n";
  }

  manager.DebugCheck();
  /* we continue with the controller synthesis for FG (target) */
  std::cout << "Synthesis: ";
  /* inner approximation of safe set */
  auto safe = [&ss_pre,&eta](const abs_type& idx) {
    double h[4] = {1.1,1.6,5.4, 5.9};
    state_type x;
    ss_pre.itox(idx,x);
    double c1= eta[0]/2.0+1e-10;
    double c2= eta[1]/2.0+1e-10;
    if ((h[0]+c1) <= x[0] && x[0] <= (h[1]-c1) &&
        (h[2]+c2) <= x[1] && x[1] <= (h[3]-c2)) {
      return true;
    }
    return false;
  };
  BDD S = ss_pre.ap_to_bdd(manager,safe);

  /*
   * we implement the nested fixed point algorithm
   *
   * mu Z. nu Y. ( pre(Y) & S ) | pre(Z)
   *
   */

  /* set up enf_pre computation */
  scots::EnfPre enf_pre(manager,TF,sym_model);
  tt.tic();
  size_t i,j;
  /* outer fp*/
  BDD Z=manager.bddOne();
  BDD ZZ=manager.bddZero();
  /* inner fp*/
  BDD Y=manager.bddZero();
  BDD YY=manager.bddOne();
  /* the controller */
  BDD C=manager.bddZero();
  /* helper */
  BDD U=ss_input.get_cube(manager);
  /* as long as not converged */
  for(i=1; ZZ != Z; i++) {
    Z=ZZ;
    BDD preZ=enf_pre(Z);
    /* init inner fp */
    YY = manager.bddOne();
    for(j=1; YY != Y; j++) {
      Y=YY;
      YY = ( enf_pre(Y) & S ) | preZ;
    }
    ZZ=YY;
    std::cout << "Inner: " << j << std::endl;
    /* remove all state-input pairs that have been added
     * to the controller already in the previous iterations */
    BDD N = ZZ & (!(C.ExistAbstract(U)));
    /* add the remaining pairs to the controller */
    C=C | N;
  }
  std::cout << "Outer: " << i << std::endl;
  tt.toc();

  std::cout << "Winning domain size: " << ss_pre.get_size(manager,C) << std::endl;

  /* symbolic set for the controller */
  // scots::SymbolicSet controller(ss_pre,ss_input);
  std::cout << "\nWrite controller to controller.scs \n";
  if(write_to_file(manager,controller,C,"controller"))
    std::cout << "Done. \n";


  return 1;
}
