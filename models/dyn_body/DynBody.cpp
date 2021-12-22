// System Includes
#include <iostream>
#include <sstream>
#include <iomanip>
#include <cmath>

// Model Includes
#include "DynBody.h"

// Default Constructor
DynBody::DynBody(void) :
   mass(0.0),
   cg(),
   eciPos(),
   eciVel() {
   // Default Inertial to Body
//   M_IDENT(T_eci2body);
}

// Default Destructor
DynBody::~DynBody(void) {
}
