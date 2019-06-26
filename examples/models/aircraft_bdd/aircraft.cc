/*
 * aircraft.cc
 *
 * created: Jan 2017
 *  author: Matthias Rungger
 *
 */

/*
 * information about this example is given in
 * http://arxiv.org/abs/1503.03715
 * doi: 10.1109/TAC.2016.2593947
 * doi: 10.1109/CDC.2015.7403185
 *
 */

#include <iostream>
#include <array>

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
const int state_dim=3;
/* input space dim */
const int input_dim=2;
/* sampling time */
const double tau = 0.25;

/* data types of the state space elements and input
 * space elements used in uniform grid and ode solver */
using state_type = std::array<double,state_dim>;
using input_type = std::array<double,input_dim>;

/* we integrate the aircraft ode by 0.25 sec (the result is stored in x)  */
auto aircraft_post = [] (state_type &x, const input_type &u) {
  /* the ode describing the aircraft */
  auto rhs =[] (state_type& xx,  const state_type &x, const input_type &u) {
    double mg = 60000.0*9.81;
    double mi = 1.0/60000;
    double c=(1.25+4.2*u[1]);
    xx[0] = mi*(u[0]*std::cos(u[1])-(2.7+3.08*c*c)*x[0]*x[0]-mg*std::sin(x[1]));
    xx[1] = (1.0/(60000*x[0]))*(u[0]*std::sin(u[1])+68.6*c*x[0]*x[0]-mg*std::cos(x[1]));
    xx[2] = x[0]*std::sin(x[1]);
  };
  /* use 10 intermediate steps */
  scots::runge_kutta_fixed4(rhs,x,u,state_dim,tau,5);
};

/* we integrate the growth bound by 0.25 sec (the result is stored in r)  */
auto radius_post = [] (state_type &r, const state_type &, const input_type &u) {
  /* the ode for the growth bound */
  auto rhs =[] (state_type& rr,  const state_type &r, const input_type &u) {
    /* lipschitz matrix computed with mupad/mathematica check the ./helper directory */
    double L[3][2];
    L[0][0]=-0.00191867*(2.7+3.08*(1.25+4.2*u[1])*(1.25+4.2*u[1]));
    L[0][1]=9.81;
    L[1][0]=0.002933+0.004802*u[1];
    L[1][1]=0.003623;
    L[2][0]=0.07483;
    L[2][1]=83.22;
    /* to account for input disturbances */
    const state_type w={{.108,0.002,0}};
    rr[0] = L[0][0]*r[0]+L[0][1]*r[1]+w[0]; /* L[0][2]=0 */
    rr[1] = L[1][0]*r[0]+L[1][1]*r[1]+w[1]; /* L[1][2]=0 */
    rr[2] = L[2][0]*r[0]+L[2][1]*r[1]+w[2]; /* L[2][2]=0 */
  };
  /* use 10 intermediate steps */
  scots::runge_kutta_fixed4(rhs,r,u,state_dim,tau,5);
};

int main() {
  /* to measure time */
  TicToc tt;

  /* cudd manager */
  Cudd mgr;
  mgr.AutodynEnable();


  /* construct grid for the state space */
  /* setup the workspace of the synthesis problem and the uniform grid */
  /* grid node distance diameter */
  /* optimized values computed according to doi: 10.1109/CDC.2015.7403185 */
  state_type s_eta={{25.0/362,3*M_PI/180/66,56.0/334}};
  /* lower bounds of the hyper rectangle */
  state_type s_lb={{58,-3*M_PI/180,0}};
  /* upper bounds of the hyper rectangle */
  state_type s_ub={{83,0,56}};
  /* construct SymbolicSet with the UniformGrid information for the state space
   * and BDD variable IDs for the pre */
  scots::SymbolicSet ss_pre(mgr, state_dim,s_lb,s_ub,s_eta);
  /* construct SymbolicSet with the UniformGrid information for the state space
   * and BDD variable IDs for the post */

  std::cout << "Unfiorm grid details:" << std::endl;
  ss_pre.print_info();

  /* construct grid for the input space */
  /* lower bounds of the hyper rectangle */
  input_type i_lb={{0,0}};
  /* upper bounds of the hyper rectangle */
  input_type i_ub={{32000,8*M_PI/180}};
  /* grid node distance diameter */
  input_type i_eta={{32000,8.0/9.0*M_PI/180}};
  scots::SymbolicSet ss_input(mgr,input_dim,i_lb,i_ub,i_eta);
  ss_input.print_info();
  scots::SymbolicSet controller(ss_pre,ss_input);

  scots::SymbolicSet ss_post(mgr, state_dim,s_lb,s_ub,s_eta);
  /* transition function of symbolic model */
  BDD TF;

  /* setup object to compute the transition function */
  scots::SymbolicModel<state_type,input_type> sym_model(ss_pre,ss_input,ss_post);
  /* measurement disturbances  */
  state_type z={{0.0125,0.0025/180*M_PI,0.05}};
  sym_model.set_measurement_error_bound(z);

  std::cout << "Computing the transition function: " << std::endl;
  tt.tic();
  size_t no_trans;
  TF = sym_model.compute_gb(mgr,aircraft_post,radius_post,no_trans);
  tt.toc();
  std::cout << "Number of transitions: " << no_trans << std::endl;
  if(!getrusage(RUSAGE_SELF, &usage))
    std::cout << "Memory per transition: " << usage.ru_maxrss/(double)no_trans << std::endl;

  scots::SymbolicSet  set(scots::SymbolicSet(ss_pre,ss_input),ss_post);
  scots::write_to_file(mgr,set,TF,"tf");

  /* define target set */
  auto target = [&s_eta, &z, &ss_pre](const scots::abs_type& abs_state) {
    state_type t_lb = {{63,-3*M_PI/180,0}};
    state_type t_ub = {{75,0,2.5}};
    state_type c_lb;
    state_type c_ub;
    /* center of cell associated with abs_state is stored in x */
    state_type x;
    ss_pre.itox(abs_state,x);
    /* hyper-interval of the quantizer symbol with perturbation */
    for(int i=0; i<state_dim; i++) {
      c_lb[i] = x[i]-s_eta[i]/2.0-z[i];
      c_ub[i] = x[i]+s_eta[i]/2.0+z[i];
    }
    if( t_lb[0]<=c_lb[0] && c_ub[0]<=t_ub[0] &&
        t_lb[1]<=c_lb[1] && c_ub[1]<=t_ub[1] &&
        t_lb[2]<=c_lb[2] && c_ub[2]<=t_ub[2]) {
      if(-0.91<=(x[0]*std::sin(x[1])-s_eta[0]/2.0-z[0]-(c_ub[0])*(s_eta[1]/2.0-z[1]))) {
        return true;
      }
    }
    return false;
  };

  BDD T = ss_pre.ap_to_bdd(mgr,target);

  std::cout << "\nSynthesis: " << std::endl;

  /*
   * we implement the fixed point algorithm
   *
   * mu X. ( pre(X) & T )
   *
   */

  /* setup enforcable predecessor */
  scots::EnfPre enf_pre(mgr,TF,sym_model);
  tt.tic();
  BDD X = mgr.bddOne();
  BDD XX =mgr.bddZero();
  /* the controller */
  BDD C = mgr.bddZero();
  /* BDD cube for existential abstract inputs */
  BDD U = ss_input.get_cube(mgr);
  /* as long as not converged */
  size_t i;
  for(i=1; XX != X; i++ ) {
    X=XX;
    XX=enf_pre(X) | T;
    /* new (state/input) pairs */
    BDD N = XX & (!(C.ExistAbstract(U)));
    /* add new (state/input) pairs to the controller */
    C=C | N;
    /* print progress */
    scots::print_progress(i);
  }
  std::cout << "\nNumber of iterations: " << i << std::endl;
  tt.toc();

  std::cout << "Winning domain size: " << ss_pre.get_size(mgr,C) << std::endl;

  /* symbolic set for the controller */
  //scots::SymbolicSet controller(ss_pre,ss_input);
  std::cout << "\nWrite controller to controller.scs \n";
  if(write_to_file(mgr,controller,C,"controller"))
    std::cout << "Done. \n";

   if(!getrusage(RUSAGE_SELF, &usage))
    std::cout << "Total memory per transition: " << usage.ru_maxrss/(double)no_trans << std::endl;

 return 1;
}
