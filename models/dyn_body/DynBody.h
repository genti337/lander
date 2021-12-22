#ifndef DYNBODY_H
#define DYNBODY_H

// C++ Includes
#include <iosfwd>
#include <string>

// Model Includes
#include "../math/vector_macros.h"
#include "../math/matrix_macros.h"

class DynBody {
   friend class EoM;
public:
   // Default Constructor
   DynBody(void);

   // Default Destructor
   ~DynBody(void);

   // Set the Body Mass
   void setMass(double mass_in);

   // Set the Body Center of Gravity
   void setCG(double cg_in);

   // Set the Inertial Position
   void setECIPos(double eciPosX, double eciPosY, double eciPosZ);

   // Get the Inertial Position
   double getECIPos(int element);

   // Set the Inertial Velocity
   void setECIVel(double eciVelX, double eciVelY, double eciVelZ);

   // Get the Inertial Velocity
   double getECIVel(int element);

   // Get the Geocentric Altitude
   double getAlt();

protected:
   double mass;			// (kg)   Mass
   double cg;			// (m)    Center of Gravity
   double eciPos[3];		// (m)    Inertial Position
   double eciVel[3];		// (m/s)  Inertial Velocity
   double eciAcc[3];		// (m/s2) Inertial Acceleration
   double T_eci2lvlh[3][3];	// (--)   Inertial to LVLH Rotation
   double T_eci2body[3][3];	// (--)   Inertial to Body Rotation
   double geocentric_alt;       // (m)    Geocentric Altitude
};	

// Set the Body Mass
inline void DynBody::setMass(double mass_in) {
   mass = mass_in;
}

// Set the Body Mass
inline void DynBody::setCG(double cg_in) {
   cg = cg_in;
}

// Set the Inertial Position
inline void DynBody::setECIPos(double eciPosX, double eciPosY, double eciPosZ) {
   eciPos[0] = eciPosX;
   eciPos[1] = eciPosY;
   eciPos[2] = eciPosZ;
}

// Get the Inertial Position
inline double DynBody::getECIPos(int element) {
   return eciPos[element];
}

// Set the Inertial Velocity
inline void DynBody::setECIVel(double eciVelX, double eciVelY, double eciVelZ) {
   eciVel[0] = eciVelX;
   eciVel[1] = eciVelY;
   eciVel[2] = eciVelZ;
}

// Get the Inertial Velocity
inline double DynBody::getECIVel(int element) {
   return eciVel[element];
}

// Get the Geocentric Altitude
inline double DynBody::getAlt() {
   return geocentric_alt;
}

#endif
