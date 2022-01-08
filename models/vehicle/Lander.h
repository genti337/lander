#ifndef Lander_H
#define Lander_H

// C++ Includes
#include <iosfwd>
#include <string>
#include <deque>

// Model Includes
#include "../dyn_body/DynBody.h"
#include "../eom/EoM.h"
#include "../fam/FaM.h"
#include "../gnc/control/LVLHAttCtl.h"

class Lander {
   friend class DynBody;

public:
   // Default Constructor
   Lander(void);

   // Default Destructor
   ~Lander(void);

   // Initialization
   void Initialize();

   // Update
   void Update(DynBody &lander);

private:
   double dt;           // (s)      Module Update Rate

   FaM fam;             // (--)     Forces and Moments
   EoM eom;             // (--)     Equations of Motion

   // Guidance Control Algorithms
   LVLHAttCtl lvlh_att_ctl;   // (--) LVLH Attitude Control Algorithm

};	

#endif
