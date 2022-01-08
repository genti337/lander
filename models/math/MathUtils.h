#ifndef _MATH_UTILS_H_
#define _MATH_UTILS_H_

// System Includes
#include <stdio.h>

// Model Includes
#include "matrix_macros.h"

class MathUtils {
public:
   // Default Constructor
   MathUtils(void);

   // Default Destructor
   ~MathUtils(void);

   // Convert Rotation Matrix to Euler Angles (Yaw-Pitch-Roll)
   void Mat2Euler(double mat[3][3], double euler[3]);
   
   // Convert Euler Angles to Rotation Matrix (Yaw-Pitch-Roll)
   void Euler2Mat(double mat[3][3], double euler[3]);

   // Matrix Inverse
   void MatInvert(double inv[3][3], double m[3][3]);
};

#endif
