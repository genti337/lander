#include <pybind11/pybind11.h>

#include "Lander.h"
#include <iostream>
#include <sstream>
#include <iomanip>
#include <cmath>

// Model Includes
#include "../math/vector_macros.h"
#include "../math/matrix_macros.h"

// Default Constructor
Lander::Lander(void) {
}

// Default Destructor
Lander::~Lander(void) {
}

// Initialization Routine
void Lander::Initialize() {
   // Default Module Update Rate
   this->dt = 0.1;

   // Initialize the Forces and Moments
   fam.Initialize();

   // Initialize the Equations of Motion
   eom.Initialize();

   return;
}

// Update Routine
void Lander::Update(DynBody &lander) {
   // Update the Forces and Moments
   fam.Update(lander);

   // Update the Equations of Motion
   eom.Update(lander);

   return;
}
