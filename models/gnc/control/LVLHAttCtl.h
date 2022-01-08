#ifndef LVLHAttCtl_H
#define LVLHAttCtl_H

// C++ Includes
#include <iosfwd>
#include <string>
#include <deque>

// Model Includes
#include "../dyn_body/DynBody.h"
#include "MathUtils.h"
#include "PIDController.hh"

class LVLHAttCtl : MathUtils {
   friend class DynBody;

public:
   // Default Constructor
   LVLHAttCtl(void);

   // Default Destructor
   ~LVLHAttCtl(void);

   // Initialization
   void Initialize();

   // Update
   void Update(DynBody &lander);

private:
   double dt;                    // (s)    Module Update Rate
   double eul_lvlh2body_cmd[3];  // (r)    Commanded LVLH to Body Euler Angles (Yaw-Pitch-Roll)
   double eul_body2body_cmd[3];  // (r)    Commanded LVLH to Body Euler Angles (Yaw-Pitch-Roll)
   double alpha_body_cmd[3];     // (r/s2) Body Angular Acceleration Command (Yaw-Pitch-Roll)
   double T_lvlh2body_cmd[3][3]; // (--)   Commanded LVLH to Body Rotation
   double T_body2body_cmd[3][3]; // (--)   Commanded Body to Body Rotation

   PIDController* attCntrl[3];   // (--)   Attitude PID Controller

};	

#endif
