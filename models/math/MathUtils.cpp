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

void MathUtils::MatInvert(double inv[3][3], /* Out: The 3X3 inverse of matrix 'm' */
                          double m[3][3])
{                                      /* In: A 3X3 matrix */
    double determinant;         /* The determinant of the input 'm' matrix */

    /* Calculate the determinant and test it */
    /* (Save the intermediate calculations in 'inv' for later use) */
    inv[0][0] = m[1][1] * m[2][2] - m[2][1] * m[1][2];
    inv[0][1] = m[2][1] * m[0][2] - m[0][1] * m[2][2];
    inv[0][2] = m[0][1] * m[1][2] - m[1][1] * m[0][2];

    determinant = m[0][0] * inv[0][0] + m[1][0] * inv[0][1] + m[2][0] * inv[0][2];
    if (determinant == 0.0) {
        fprintf(stderr, "trick_utils/math/dm_invert.c: 3x3 matrix has zero determinant\n");
        return;
    }

    /* Calculate the inverse matrix */
    inv[0][0] /= determinant;
    inv[0][1] /= determinant;
    inv[0][2] /= determinant;
    inv[1][0] = (m[2][0] * m[1][2] - m[1][0] * m[2][2]) / determinant;
    inv[1][1] = (m[0][0] * m[2][2] - m[2][0] * m[0][2]) / determinant;
    inv[1][2] = (m[1][0] * m[0][2] - m[0][0] * m[1][2]) / determinant;
    inv[2][0] = (m[1][0] * m[2][1] - m[2][0] * m[1][1]) / determinant;
    inv[2][1] = (m[2][0] * m[0][1] - m[0][0] * m[2][1]) / determinant;
    inv[2][2] = (m[0][0] * m[1][1] - m[1][0] * m[0][1]) / determinant;

    return;
}

#endif
