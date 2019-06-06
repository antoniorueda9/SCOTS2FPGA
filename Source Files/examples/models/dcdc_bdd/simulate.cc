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

#include <iostream>
#include <array>

/* SCOTS header */
#include "scots.hh"
/* ode solver */
#include "RungeKutta4.hh"

/* state space dim */
const int state_dim=2;
/* sampling time */
const double tau = 0.05;

/* data type used by the ODE solver */
using state_type = std::array<double,state_dim>;

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

int main() {

  /* Cudd manager */
  Cudd manager;

  /* read controller from file */
  BDD C;
  scots::SymbolicSet con;
  if(!read_from_file(manager,con,C,"controller")) {
    std::cout << "Could not read controller from controller.scs\n";
    return 0;
  }

  std::cout << "\nSimulation:\n " << std::endl;

  state_type x={{0.7, 5.4}};

  for(int i=0; i<100; i++) {
    /* returns a std vector with the valid control inputs */
    auto u = con.restriction(manager,C,x);
    std::cout << x[0] <<  " "  << x[1] << "\n";
    //std::cout << u[0] << "\n";
    system_post(x,u[0]);
  }

  return 1;
}
