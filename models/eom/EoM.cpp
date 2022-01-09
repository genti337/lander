#include <pybind11/pybind11.h>

#include "EoM.h"
#include <iostream>
#include <sstream>
#include <iomanip>
#include <cmath>
//#include <Eigen/Core>

// Trick Includes
//#include "include/trick/trick_math_proto.h"

// Model Includes
#include "vector_macros.h"
#include "matrix_macros.h"
#include "EoM.h"

// Default Constructor
EoM::EoM(void) {
}

// Default Destructor
EoM::~EoM(void) {
}

// Initialization Routine
void EoM::Initialize() {
   // Default Module Update Rate
   this->dt = 0.1;

   // Lunar Constants
   this->planet_omega = 0.0000026617; // Planet Rotation Rate
   this->planet_rot = 0.0;            // Planet Rotation Angle
   this->planet_gm = 6.673e-11;       // Universal Gravitational Constant
   this->planet_mass = 7.35e22;       // Planet Mass
   this->planet_radius = 1.74e6;      // Planet Radius

   // Initialize the Planet-Fixed to Inertial to be Co-aligned
   M_IDENT(T_eci2pfix);

   // Defautl Landing Site
   this->pfix_lsite[0] = 0.0;
   this->pfix_lsite[1] = 0.0;
   this->pfix_lsite[2] = this->planet_radius;

   return;
}

// Update Routine
void EoM::Update(DynBody &lander) {
   static bool first_pass = true;
   double vec1[3] = {0.0, 0.0, 0.0};
   double vec2[3] = {0.0, 0.0, 0.0};
   double vec3[3] = {0.0, 0.0, 0.0};
   double omega_vec[3] = {0.0, 0.0, planet_omega};

   // Update the Planet Rotation Angle
   this->planet_rot += this->planet_omega * this->dt;
   if (this->planet_rot > 2.0 * M_PI) this->planet_rot = 0.0;

   // Inertial to Planet-Fixed Rotation
   T_eci2pfix[0][0] = cos(this->planet_rot);
   T_eci2pfix[0][1] = sin(this->planet_rot);
   T_eci2pfix[0][2] = 0.0;
   T_eci2pfix[1][0] = -sin(this->planet_rot);
   T_eci2pfix[1][1] = cos(this->planet_rot);
   T_eci2pfix[1][2] = 0.0;
   T_eci2pfix[2][0] = 0.0;
   T_eci2pfix[2][1] = 0.0;
   T_eci2pfix[2][2] = 1.0;
   M_TRANS(T_pfix2eci, T_eci2pfix);

   // Inertial to LVLH Rotation
   double hvec[3] = {0.0, 0.0, 0.0};
   V_SCALE(T_eci2lvlh[2], lander.eciPos, -1.0);
   V_CROSS(hvec, lander.eciPos, lander.eciVel);
   V_SCALE(T_eci2lvlh[1], hvec, -1.0);
   V_CROSS(T_eci2lvlh[0], T_eci2lvlh[1], T_eci2lvlh[2]);
   V_NORM(T_eci2lvlh[0], T_eci2lvlh[0]);
   V_NORM(T_eci2lvlh[1], T_eci2lvlh[1]);
   V_NORM(T_eci2lvlh[2], T_eci2lvlh[2]);

   // Inertial Frame Acceleration
   for (int i=0; i<3; i++) {
      lander.eciAcc[i] = lander.force_eci[i] / lander.mass;
   }

   // Update the Vehicle Position and Velocity
   for (int i=0; i<3; i++) {
      Integrate(lander.eciVel[i], lander.eciVel[i], lander.eciAcc[i], eciAccPrev[i]);
      Integrate(lander.eciPos[i], lander.eciPos[i], lander.eciVel[i], eciVelPrev[i]);
   }

//   V_PRINT(lander.eciAcc);
//   printf("\n");

   // Integrate the Vehicle Moment
   for (int i=0; i<3; i++) {
      Integrate(lander.moment_body[i], lander.moment_body[i], lander.torque_body[i], eciVelPrev[i]);
   }

   // Body Angular Velocity
   double inertia_inverse[3][3];
   MatInvert(inertia_inverse, lander.inertia);
   MxV(lander.omega_body, inertia_inverse, lander.moment_body);

   // Update the Inertial to Body Rotation Matrix
   double T_eci2body_dot[3][3];
   double omega_star[3][3];
   V_SKEW(omega_star, lander.omega_body);
   MxM(T_eci2body_dot, omega_star, lander.T_eci2body);

   // Integrate the Rotation Matrix
   for (int i=0; i<3; i++) {
      for (int j=0; j<3; j++) {
         Integrate(lander.T_eci2body[i][j], lander.T_eci2body[i][j], T_eci2body_dot[i][j], eciVelPrev[i]);
      }
   }
   M_TRANS(lander.T_body2eci, lander.T_eci2body);

   // Planet-Fixed State
   MxV(lander.pfixPos, T_eci2pfix, lander.eciPos);
   V_CROSS(vec1, omega_vec, lander.eciPos);
   V_SUB(vec2, lander.eciVel, vec1);
   MxV(lander.pfixVel, T_eci2pfix, vec2);

   // LVLH State
   MxV(lander.lvlhVel, T_eci2lvlh, lander.eciVel);
   MxMt(lander.T_lvlh2body, lander.T_eci2body, T_eci2lvlh);
   Mat2Euler(lander.T_lvlh2body, lander.eul_lvlh2body); 

   // Geocentric Altitude
   lander.geocentric_alt = V_MAG(lander.eciPos) - this->planet_radius;

   // Calculate the Down Range and Cross Range
   if (first_pass) {
      V_SUB(T_pfix2lframe[0], lander.pfixPos, this->pfix_lsite); 
      V_NORM(T_pfix2lframe[0], T_pfix2lframe[0]);
      V_NORM(T_pfix2lframe[2], this->pfix_lsite);
      V_CROSS(T_pfix2lframe[1], T_pfix2lframe[2], T_pfix2lframe[0]);
      V_CROSS(T_pfix2lframe[0], T_pfix2lframe[1], T_pfix2lframe[2]);
   }

   // Calculate the Position and Velocity in the Landing Frame
   MxV(lander.lframe_pos, T_pfix2lframe, lander.pfixPos);
   MxV(lander.lframe_vel, T_pfix2lframe, lander.pfixVel);

   // Down Range
   this->down_range_theta = atan(lander.lframe_pos[0] / lander.lframe_pos[2]);
   this->down_range = V_MAG(lander.eciPos) * this->down_range_theta;

   // Cross Range
   this->cross_range_theta = atan2(lander.lframe_pos[1], lander.lframe_pos[0]);
   this->cross_range = V_MAG(lander.eciPos) * this->cross_range_theta;

   // Reset the First Pass Flag
   first_pass = false;

   return;
}

// Integration Routine
void EoM::Integrate(double& output, double input, double input_deriv, std::deque<double>& input_deriv_vec) {
   // Update the Input Derivative
   input_deriv_vec.push_front(input_deriv);
   input_deriv_vec.pop_back();

   // Integrate the Inertial Position/Velocity
   //output = input + (1/24.0) * this->dt * (55.0 * input_deriv_vec[0] - 59.0 * input_deriv_vec[1] + 37.0 * input_deriv_vec[2] - 9.0 * input_deriv_vec[3]);
   output = input + input_deriv * this->dt;
}

// Convert the Rotation Matrix to Euler Angles
//void EoM::Mat2Euler(double mat[3][3], double euler[3]){
//   if ((mat[2][0] > -1.0) && (mat[2][0] < 1.0)) {
//      euler[1] = -asin(mat[2][0]);
//      euler[2] = atan2(mat[1][1] / cos(euler[1]), mat[1][2] / cos(euler[1]));
//      euler[0] = atan2(mat[1][0] / cos(euler[1]), mat[0][0] / cos(euler[1]));
//   } else {
//      euler[0] = 0.0;
//      if (mat[2][0] == -1.0) {
//         euler[1] = M_PI / 2.0;
//         euler[2] = euler[0] + atan2(mat[0][1], mat[0][2]);
//      } else {
//         euler[1] = -M_PI / 2.0;
//         euler[2] = -euler[0] + atan2(-mat[0][1], -mat[0][2]);
//      }
//   }
//
//   return;
//}

// Convert the Euler Angles to Rotation Matrix
//void EoM::Euler2Mat(double mat[3][3], double euler[3]) {
//   double mat1[3][3], mat2[3][3], mat3[3][3];
//   double temp_mat[3][3];
//
//   // Rotation about the X-Axis
//   mat1[0][0] = 1.0;
//   mat1[0][1] = 0.0;
//   mat1[0][2] = 0.0;
//   mat1[1][0] = 0.0;
//   mat1[1][1] = cos(euler[0]);
//   mat1[1][2] = -sin(euler[0]);
//   mat1[2][0] = 0.0;
//   mat1[2][1] = sin(euler[0]);
//   mat1[2][2] = cos(euler[0]);
//
//   // Rotation about the Y-Axis
//   mat2[0][0] = cos(euler[1]);
//   mat2[0][1] = 0.0;
//   mat2[0][2] = sin(euler[1]);
//   mat2[1][0] = 0.0;
//   mat2[1][1] = 1.0;
//   mat2[1][2] = 0.0;
//   mat2[2][0] = -sin(euler[1]);
//   mat2[2][1] = 0.0;
//   mat2[2][2] = cos(euler[1]);
//
//   // Rotation about the Z-Axis
//   mat3[0][0] = cos(euler[2]);
//   mat3[0][1] = -sin(euler[2]);
//   mat3[0][2] = 0.0;
//   mat3[1][0] = sin(euler[2]);
//   mat3[1][1] = cos(euler[2]);
//   mat3[1][2] = 0.0;
//   mat3[2][0] = 0.0;
//   mat3[2][1] = 0.0;
//   mat3[2][2] = 1.0;
//  
//   // Multiply Rotations to get Matrix
//   MxM(temp_mat, mat1, mat2);
//   MxM(mat, temp_mat, mat3);
//}
