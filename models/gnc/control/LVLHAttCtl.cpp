#include <pybind11/pybind11.h>

#include "LVLHAttCtl.h"
#include <iostream>
#include <sstream>
#include <iomanip>
#include <cmath>

// Model Includes
#include "../math/vector_macros.h"
#include "../math/matrix_macros.h"

// Default Constructor
LVLHAttCtl::LVLHAttCtl(void) {
}

// Default Destructor
LVLHAttCtl::~LVLHAttCtl(void) {
}

// Initialization Routine
void LVLHAttCtl::Initialize() {
   // Default Module Update Rate
   this->dt = 0.1;

   // Initialize the PID Controller
   for (int i=0; i<3; i++) {
      attCntrl[i] = new PIDController(0.15, 0.0, 0.25, 0.1 * M_PI / 180.0, -0.1 * M_PI / 180.0, this->dt, this->dt);
   }

   return;
}

// Update Routine
void LVLHAttCtl::Update(DynBody &lander) {
   eul_lvlh2body_cmd[0] = 25.0 * M_PI / 180.0;
   eul_lvlh2body_cmd[1] = 3.0 * M_PI / 180.0;
   eul_lvlh2body_cmd[2] = 7.0 * M_PI / 180.0;

   // Command LVLH to Body Rotation
   Euler2Mat(T_lvlh2body_cmd, eul_lvlh2body_cmd);

   // Commanded Body to Body Rotation
   MxMt(T_body2body_cmd, T_lvlh2body_cmd, lander.T_lvlh2body);
   Mat2Euler(T_body2body_cmd, eul_body2body_cmd);

   // Update the PID Controller
   for (int i=0; i<3; i++) {
      alpha_body_cmd[i] = attCntrl[i]->getOutput(eul_body2body_cmd[i]);
   }

   // Commanded Torque in the Body Frame
   MxV(lander.torque_body, lander.inertia, alpha_body_cmd);

   return;
}
