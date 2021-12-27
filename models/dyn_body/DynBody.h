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
   friend class FaM;
public:
   // Default Constructor
   DynBody(void);

   // Default Destructor
   ~DynBody(void);

   // Set the Body Mass
   void setMass(double mass_in);

   // Set the Body Center of Gravity
   void setCG(double cg_in[3]);

   // Set the Inertial Position
   void setECIPos(double eciPosX, double eciPosY, double eciPosZ);

   // Get the Inertial Position
   double getECIPos(int element);

   // Set the Inertial Velocity
   void setECIVel(double eciVelX, double eciVelY, double eciVelZ);

   // Get the Inertial Velocity
   double getECIVel(int element);

   // Get the LVLH Velocity
   double getLVLHVel(int element);

   // Get the Geocentric Altitude
   double getAlt();

protected:
   double mass;			// (kg)    Mass
   double cg[3];		// (m)     Center of Gravity
   double inertia[3][3];	// (m)     Moment of Inertia
   double eciPos[3];		// (m)     Inertial Position
   double eciVel[3];		// (m/s)   Inertial Velocity
   double eciAcc[3];		// (m/s2)  Inertial Acceleration
   double pfixPos[3];		// (m)     Planet-Fixed Position
   double pfixVel[3];		// (m/s)   Planet-Fixed Velocity
   double bodyAcc[3];		// (m/s2)  Body Acceleration
   double lvlhVel[3];		// (m/s)   Local Vertical-Local Horizontal Velocity
   double T_eci2body[3][3];	// (--)    Inertial to Body Rotation
   double T_body2eci[3][3];	// (--)    Body to Inertial Rotation
   double T_lvlh2body[3][3];    // (--)    LVLH to Body Rotation
   double geocentric_alt;       // (m)     Geocentric Altitude
   double force_body[3];        // (N)     Force in the Body Frame
   double torque_body[3];       // (N*m)   Torque in the Body Frame
   double moment_body[3];       // (N*m)   Moment in the Body Frame
   double alpha[3];             // (r/s^2) Body Angular Acceleration
   double omega_body[3];        // (r/s^2) Body Angular Velocity
   double omega_eci[3];         // (r/s^2) Body Angular Velocity
};	

// Set the Body Mass
inline void DynBody::setMass(double mass_in) {
   mass = mass_in;
}

// Set the Body Mass
inline void DynBody::setCG(double cg_in[3]) {
   V_COPY(cg, cg_in);
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

// Get the LVLH Velocity
inline double DynBody::getLVLHVel(int element) {
   return lvlhVel[element];
}

// Get the Geocentric Altitude
inline double DynBody::getAlt() {
   return geocentric_alt;
}

#endif
