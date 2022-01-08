#ifndef DYNBODY_H
#define DYNBODY_H

// C++ Includes
#include <iosfwd>
#include <string>

// Model Includes
#include "vector_macros.h"
#include "matrix_macros.h"
#include "MathUtils.h"

class DynBody : MathUtils {
   friend class EoM;
   friend class FaM;
   friend class LVLHAttCtl;
public:
   // Default Constructor
   DynBody(void);

   // Default Destructor
   ~DynBody(void);

   // Set the Body Mass
   void setMass(double mass_in);

   // Set the Body Center of Gravity
   void setCG(double cg_x, double cg_y, double cg_z);

   // Set the Body Moment of Inertia
   void setInertia(double ixx, double ixy, double ixz,
                   double iyx, double iyy, double iyz,
                   double izx, double izy, double izz);

   // Set the Inertial Position
   void setECIPos(double eciPosX, double eciPosY, double eciPosZ);

   // Get the Inertial Position
   double getECIPos(int element);

   // Set the Inertial Velocity
   void setECIVel(double eciVelX, double eciVelY, double eciVelZ);

   // Set the LVLH Atttiude
   void setLVLHAtt(double yaw, double pitch, double roll);

   // Get the Inertial Velocity
   double getECIVel(int element);

   // Get the LVLH Velocity
   double getLVLHVel(int element);

   // Get the LVLH Attitude
   double getLVLHAtt(int element);

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
   double eul_lvlh2body[3];     // (r)     LVLH to Body Euler Angles (Yaw-Pitch-Roll)
   double geocentric_alt;       // (m)     Geocentric Altitude
   double force_eci[3];         // (N)     Force in the Inertial Frame
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
inline void DynBody::setCG(double cg_x, double cg_y, double cg_z) {
   cg[0] = cg_x;
   cg[1] = cg_y;
   cg[2] = cg_z;
}

// Set the Body Moment of Inertia
inline void DynBody::setInertia(double ixx, double ixy, double ixz,
                                double iyx, double iyy, double iyz,
                                double izx, double izy, double izz) {
   inertia[0][0] = ixx;
   inertia[0][1] = ixy;
   inertia[0][2] = ixz;
   inertia[1][0] = iyx;
   inertia[1][1] = iyy;
   inertia[1][2] = iyz;
   inertia[2][0] = izx;
   inertia[2][1] = izy;
   inertia[2][2] = izz;
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

// Set the LVLH Attitude
inline void DynBody::setLVLHAtt(double yaw, double pitch, double roll) {
   double euler[3] = {yaw, pitch, roll};
   double hvec[3] = {0.0, 0.0, 0.0};
   double T_eci2lvlh[3][3];

   Euler2Mat(T_lvlh2body, euler);

   // Inertial to LVLH Rotation
   V_SCALE(T_eci2lvlh[2], eciPos, -1.0);
   V_CROSS(hvec, eciPos, eciVel);
   V_SCALE(T_eci2lvlh[1], hvec, -1.0);
   V_CROSS(T_eci2lvlh[0], T_eci2lvlh[1], T_eci2lvlh[2]);
   V_NORM(T_eci2lvlh[0], T_eci2lvlh[0]);
   V_NORM(T_eci2lvlh[1], T_eci2lvlh[1]);
   V_NORM(T_eci2lvlh[2], T_eci2lvlh[2]);

   // Inertial to Body
   MxM(T_eci2body, T_lvlh2body, T_eci2lvlh);

   M_PRINT(T_lvlh2body);
   M_PRINT(T_eci2body);
}

// Get the Inertial Velocity
inline double DynBody::getECIVel(int element) {
   return eciVel[element];
}

// Get the LVLH Velocity
inline double DynBody::getLVLHVel(int element) {
   return lvlhVel[element];
}

// Get the LVLH Attitude
inline double DynBody::getLVLHAtt(int element) {
   return eul_lvlh2body[element];
}

// Get the Geocentric Altitude
inline double DynBody::getAlt() {
   return geocentric_alt;
}

#endif
