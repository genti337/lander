#ifndef EOM_H
#define EOM_H

// C++ Includes
#include <iosfwd>
#include <string>
#include <deque>

// Model Includes
#include "MathUtils.h"
#include "../dyn_body/DynBody.h"

class EoM : MathUtils {
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

   // Convert the Rotation Matrix to Euler Angles (Yaw-Pitch-Roll)
   //void Mat2Euler(double mat[3][3], double euler[3]);

   // Convert the Euler Angles to Rotation Matrix (Yaw-Pitch-Roll)
   //void Euler2Mat(double mat[3][3], double euler[3]);

private:
   double planet_gm;	// (m^3*kg) Planet Graviational Constant
   double planet_mass;  // (kg)     Planet Mass
   double planet_radius;// (m)      Planet Radius
   double planet_omega;	// (r/s)    Planet Rotation Rate
   double planet_rot;	// (r)      Planet Rotation Angle
   double dt;           // (s)      Module Update Rate

   double T_eci2pfix[3][3];    // (--) Inertial to Planet-Fixed Rotation Matrix
   double T_pfix2eci[3][3];    // (--) Planet-Fixed to Inertial Rotation Matrix
   double T_eci2lvlh[3][3];    // (--) Inertial to Planet-Fixed Rotation Matrix
   double T_pfix2lframe[3][3]; // (--) Planet-Fixed to Landing Frame

   double accel_body[3];     // (m/s^2)  Acceleration in the Body Frame
   double accel_eci[3];      // (m/s^2)  Acceleration in the Inertial Frame

   double pfix_lsite[3];     // (m)      Planet-Fixed Landing Site
   double down_range;        // (m)      Down Range from Landing Site
   double down_range_theta;  // (r)      Down Range Angle from Landing Site
   double cross_range;       // (m)      Cross Range from Landing Site
   double cross_range_theta; // (r)      Cross Range Angle from Landing Site

   std::deque<double> eciVelPrev[3];
   std::deque<double> eciAccPrev[3];
};	

#endif
