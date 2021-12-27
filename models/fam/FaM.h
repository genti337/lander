#ifndef FaM_H
#define FaM_H

// C++ Includes
#include <iosfwd>
#include <string>
#include <deque>

// Model Includes
#include "../dyn_body/DynBody.h"

class FaM {
   friend class DynBody;

public:
   // Default Constructor
   FaM(void);

   // Default Destructor
   ~FaM(void);

   // Initialization
   void Initialize();

   // Update
   void Update(DynBody &lander);

private:
   double planet_gm;	// (m^3*kg) Planet Graviational Constant
   double planet_mass;  // (kg)     Planet Mass
   double planet_radius;// (m)      Planet Radius
   double planet_omega;	// (r/s)    Planet Rotation Rate
   double planet_rot;	// (r)      Planet Rotation Angle
   double dt;           // (s)      Module Update Rate

   double grav_accel;          // (m/s^2)  Gravitational Acceleration Magnitude
   double grav_accel_eci[3];   // (m/s^2)  Gravitational Acceleration wrt Inertial
   double grav_accel_body[3];  // (m/s^2)  Gravitational Acceleration wrt Body
};	

#endif
