#include <pybind11/pybind11.h>

#include "FaM.h"
#include <iostream>
#include <sstream>
#include <iomanip>
#include <cmath>

// Model Includes
#include "../math/vector_macros.h"
#include "../math/matrix_macros.h"

// Default Constructor
FaM::FaM(void) {
}

// Default Destructor
FaM::~FaM(void) {
}

// Initialization Routine
void FaM::Initialize() {
   // Default Module Update Rate
   this->dt = 0.1;

   // Lunar Constants
   this->planet_omega = 0.0000026617; // Planet Rotation Rate
   this->planet_rot = 0.0;            // Planet Rotation Angle
   this->planet_gm = 6.673e-11;       // Universal Gravitational Constant
   this->planet_mass = 7.35e22;       // Planet Mass
   this->planet_radius = 1.74e6;      // Planet Radius

   return;
}

// Update Routine
void FaM::Update(DynBody &lander) {
   // Gravitaional Acceleration
   this->grav_accel = (this->planet_gm * this->planet_mass) / powf(V_MAG(lander.eciPos), 2);

   // Gravitational Acceleration wrt Inertial Frame
   double eciPosNorm[3] = {0.0, 0.0, 0.0};
   V_NORM(eciPosNorm, lander.eciPos);
   V_SCALE(this->grav_accel_eci, eciPosNorm, -this->grav_accel);

   // Gravitational Acceleration in the Inertial Frame
   MxV(this->grav_accel_body, lander.T_eci2body, this->grav_accel_eci);

   // Forces
   for (int i=0; i<3; i++) {
      lander.force_eci[i] = lander.mass * this->grav_accel_eci[i];
      lander.force_body[i] = lander.mass * this->grav_accel_body[i];
   }

   return;
}
