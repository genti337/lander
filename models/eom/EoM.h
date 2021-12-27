#ifndef EOM_H
#define EOM_H

// C++ Includes
#include <iosfwd>
#include <string>
#include <deque>

// Model Includes
#include "../math/Vec3.h"
#include "../dyn_body/DynBody.h"

class EoM {
   friend class DynBody;

public:
   // Default Constructor
   EoM(void);

   // Default Destructor
   ~EoM(void);

   // Initialization
   void Initialize();

   // Update
   void Update(DynBody &lander);

   // Integration Routine
   void Integrate(double& output, double input, double input_deriv, std::deque<double>& input_deriv_vec);

private:
   double planet_gm;	// (m^3*kg) Planet Graviational Constant
   double planet_mass;  // (kg)     Planet Mass
   double planet_radius;// (m)      Planet Radius
   double planet_omega;	// (r/s)    Planet Rotation Rate
   double planet_rot;	// (r)      Planet Rotation Angle
   double dt;           // (s)      Module Update Rate

   double T_eci2pfix[3][3];  // (--) Inertial to Planet-Fixed Rotation Matrix
   double T_pfix2eci[3][3];  // (--) Planet-Fixed to Inertial Rotation Matrix
   double T_eci2lvlh[3][3];  // (--) Inertial to Planet-Fixed Rotation Matrix

   double accel_body[3];     // (m/s^2)  Acceleration in the Body Frame
   double accel_eci[3];      // (m/s^2)  Acceleration in the Inertial Frame

   std::deque<double> eciVelPrev[3];
   std::deque<double> eciAccPrev[3];
};	

#endif
