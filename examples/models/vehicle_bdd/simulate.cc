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
  if(!read_from_file(manager,con,C,"controller")) {
    std::cout << "Could not read controller from controller.scs\n";
    return 0;
  }
  
  std::cout << "\nSimulation:\n " << std::endl;

  state_type x={{0.6, 0.6, 0}};
  while(1) {
    /* returns a std vector with the valid control inputs */
    auto u = con.restriction(manager,C,x);
    std::cout << x[0] <<  " "  << x[1] << " " << x[2] << "\n";
    //std::cout << u[0] <<  " "  << u[1] << "\n";
    vehicle_post(x,u);
    if(target(x)) {
      std::cout << "Arrived: " << x[0] <<  " "  << x[1] << " " << x[2] << std::endl;
      break;
    }
  }

  return 1;
}
