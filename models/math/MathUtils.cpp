#ifndef _MATRIX_FUNC_H_
#define _MATRIX_FUNC_H_

// System Includes
#include <stdio.h>
#include <cmath>

// Model Includes
#include "matrix_macros.h"
#include "MathUtils.h"

// Default Constructor
MathUtils::MathUtils(void) {
}

// Default Destructor
MathUtils::~MathUtils(void) {
}

// Convert Rotation Matrix to Euler Angles (Yaw-Pitch-Roll)
void MathUtils::Mat2Euler(double mat[3][3], double euler[3]) {
   if ((mat[2][0] > -1.0) && (mat[2][0] < 1.0)) {
      euler[1] = -asin(mat[2][0]);
      euler[0] = atan2(mat[2][1] / cos(euler[1]), mat[2][2] / cos(euler[1]));
      euler[2] = atan2(mat[1][0] / cos(euler[1]), mat[0][0] / cos(euler[1]));
   } else {
      euler[2] = 0.0;
      if (mat[2][0] == -1.0) {
         euler[1] = M_PI / 2.0;
         euler[0] = euler[0] + atan2(mat[0][1], mat[0][2]);
      } else {
         euler[1] = -M_PI / 2.0;
         euler[0] = -euler[0] + atan2(-mat[0][1], -mat[0][2]);
      }
   }

   return;
}

// Convert Euler Angles to Rotation Matrix (Yaw-Pitch-Roll)
void MathUtils::Euler2Mat(double mat[3][3], double euler[3]) {
   double mat1[3][3], mat2[3][3], mat3[3][3];
   double temp_mat[3][3];

   // Rotation about the X-Axis
   mat1[0][0] = 1.0;
   mat1[0][1] = 0.0;
   mat1[0][2] = 0.0;
   mat1[1][0] = 0.0;
   mat1[1][1] = cos(euler[0]);
   mat1[1][2] = -sin(euler[0]);
   mat1[2][0] = 0.0;
   mat1[2][1] = sin(euler[0]);
   mat1[2][2] = cos(euler[0]);

   // Rotation about the Y-Axis
   mat2[0][0] = cos(euler[1]);
   mat2[0][1] = 0.0;
   mat2[0][2] = sin(euler[1]);
   mat2[1][0] = 0.0;
   mat2[1][1] = 1.0;
   mat2[1][2] = 0.0;
   mat2[2][0] = -sin(euler[1]);
   mat2[2][1] = 0.0;
   mat2[2][2] = cos(euler[1]);

   // Rotation about the Z-Axis
   mat3[0][0] = cos(euler[2]);
   mat3[0][1] = -sin(euler[2]);
   mat3[0][2] = 0.0;
   mat3[1][0] = sin(euler[2]);
   mat3[1][1] = cos(euler[2]);
   mat3[1][2] = 0.0;
   mat3[2][0] = 0.0;
   mat3[2][1] = 0.0;
   mat3[2][2] = 1.0;
  
   // Multiply Rotations to get Matrix
   MxM(temp_mat, mat3, mat2);
   MxM(mat, temp_mat, mat1);
}

#endif
