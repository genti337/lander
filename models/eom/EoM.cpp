#include <pybind11/pybind11.h>

#include "EoM.h"
#include <iostream>
#include <sstream>
#include <iomanip>
#include <cmath>

// Model Includes
#include "../math/vector_macros.h"
#include "../math/matrix_macros.h"

// Default Constructor
EoM::EoM(void) {
}

// Default Destructor
EoM::~EoM(void) {
}

// Initialization Routine
void EoM::Initialize() {
   // Default Module Update Rate
   this->dt = 0.1;

   // Lunar Constants
   this->planet_omega = 0.0000026617; // Planet Rotation Rate
   this->planet_rot = 0.0;            // Planet Rotation Angle
   this->planet_gm = 6.673e-11;       // Universal Gravitational Constant
   this->planet_mass = 7.35e22;       // Planet Mass
   this->planet_radius = 1.74e6;      // Planet Radius

   // Initialize the Planet-Fixed to Inertial to be Co-aligned
   M_IDENT(T_eci2pfix);

   return;
}

// Update Routine
void EoM::Update(DynBody &lander) {
   static bool first_pass = true;

   // Update the Planet Rotation Angle
   this->planet_rot += this->planet_omega * this->dt;
   if (this->planet_rot > 2.0 * M_PI) this->planet_rot = 0.0;

   // Inertial to Planet-Fixed Rotation
   T_eci2pfix[0][0] = cos(this->planet_rot);
   T_eci2pfix[0][1] = sin(this->planet_rot);
   T_eci2pfix[0][2] = 0.0;
   T_eci2pfix[1][0] = -sin(this->planet_rot);
   T_eci2pfix[1][1] = cos(this->planet_rot);
   T_eci2pfix[1][2] = 0.0;
   T_eci2pfix[2][0] = 0.0;
   T_eci2pfix[2][1] = 0.0;
   T_eci2pfix[2][2] = 1.0;
   M_TRANS(T_pfix2eci, T_eci2pfix);

   // Inertial to LVLH Rotation
   double hvec[3] = {0.0, 0.0, 0.0};
   V_SCALE(lander.T_eci2lvlh[2], lander.eciPos, -1.0);
   V_CROSS(hvec, lander.eciPos, lander.eciVel);
   V_SCALE(lander.T_eci2lvlh[1], hvec, -1.0);
   V_CROSS(lander.T_eci2lvlh[0], lander.T_eci2lvlh[1], lander.T_eci2lvlh[2]);
   V_NORM(lander.T_eci2lvlh[0], lander.T_eci2lvlh[0]);
   V_NORM(lander.T_eci2lvlh[1], lander.T_eci2lvlh[1]);
   V_NORM(lander.T_eci2lvlh[2], lander.T_eci2lvlh[2]);

   // Gravitaional Acceleration
   this->grav_accel = (this->planet_gm * this->planet_mass) / powf(V_MAG(lander.eciPos), 2);

   // Gravitational Acceleration wrt Inertial Frame
   double eciPosNorm[3] = {0.0, 0.0, 0.0};
   V_NORM(eciPosNorm, lander.eciPos);
   V_SCALE(this->grav_accel_eci, eciPosNorm, -this->grav_accel);

   // Inertial Acceleration
   V_INIT(lander.eciAcc);
   V_ADD(lander.eciAcc, lander.eciAcc, this->grav_accel_eci);

   // Initialize Previous Values of Inertial Acceleration
   if (first_pass) {
      for (int i=0; i<3; i++) {
         for (int j=0; j<4; j++) {
            eciVelPrev[i].push_back(lander.eciVel[i]);
            eciAccPrev[i].push_back(lander.eciAcc[i]);
         }
      }
   }

   // Update the Vehicle Position and Velocity
   for (int i=0; i<3; i++) {
      Integrate(lander.eciVel[i], lander.eciVel[i], lander.eciAcc[i], eciAccPrev[i]);
      Integrate(lander.eciPos[i], lander.eciPos[i], lander.eciVel[i], eciVelPrev[i]);
   }

   // Geocentric Altitude
   lander.geocentric_alt = V_MAG(lander.eciPos) - planet_radius;

   // Reset the First Pass Flag
   first_pass = false;

   return;
}


// Integration Routine
void EoM::Integrate(double& output, double input, double input_deriv, std::deque<double>& input_deriv_vec) {
   // Update the Input Derivative
   input_deriv_vec.push_front(input_deriv);
   input_deriv_vec.pop_back();

   // Integrate the Inertial Position/Velocity
   //output = input + (1/24.0) * this->dt * (55.0 * input_deriv_vec[0] - 59.0 * input_deriv_vec[1] + 37.0 * input_deriv_vec[2] - 9.0 * input_deriv_vec[3]);
   output = input + input_deriv * this->dt;
}
