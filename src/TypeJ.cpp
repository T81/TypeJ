// Type J Thermocouple library per ITS-90

// *** BSD License ***
// ------------------------------------------------------------------------------------------
//
// Author: T81
//
// Redistribution and use in source and binary forms, with or without modification, are 
// permitted provided that the following conditions are met:
//
// Redistributions of source code must retain the above copyright notice, this list of 
// conditions and the following disclaimer.
//
// Redistributions in binary form must reproduce the above copyright notice, this list 
// of conditions and the following disclaimer in the documentation and/or other materials 
// provided with the distribution.
//
// Neither the name of the MLG Properties, LLC nor the names of its contributors may be 
// used to endorse or promote products derived from this software without specific prior 
// written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS 
// OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF 
// MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL 
// THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) 
// HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, 
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS 
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// 
// Original Author of Type K thermocouples library: Jim Gallt
// ------------------------------------------------------------------------------------------

#include "TypeJ.h"
#if ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif


// -------------------------------------
const int TypeJ::nranges_inv = 3;  // number of mV ranges for inverse lookup
const int TypeJ::ncoeff_inv = 9;  // number of coefficients for inverse lookup
const float TypeJ::mv_min = -8.095;
const float TypeJ::mv_max = 69.553;

// coefficients for inverse lookup (given mV, find C)
const double TypeJ::coeff_inv[9][3] = {
	 {  0.0000000E+00,  0.000000E+00, -3.11358187E+03 },
	 {  1.9528268E+01,  1.978425E+01,  3.00543684E+02 }, 
	 { -1.2286185E+00, -2.001204E-01, -9.94773230E+00 },
	 { -1.0752178E+00,  1.036969E-02,  1.70276630E-01 },
	 { -5.9086933E-01, -2.549687E-04, -1.43033468E-03 },
	 { -1.7256713E-01,  3.585153E-06,  4.73886084E-06 },
	 { -2.8131513E-02, -5.344285E-08,  0.00000000E+00 },
	 { -2.3963370E-03,  5.099890E-10,  0.00000000E+00 },
	 { -8.3823321E-05,  0.000000E+00,  0.00000000E+00 }
};

// mV ranges for inverse lookup coefficients
const float TypeJ::range_inv[2][3] = {
  { -8.095,          0.000,         42.919  },
  {  0.000,         42.919,         69.553  }
};

// coefficients for direct lookup (given C, find mV)
const double TypeJ::coeff_dir[9][2] = {
	 {  0.000000000000E+00,  0.296456256810E+03 },
	 {  0.503811878150E-01, -0.149761277860E+01 },
	 {  0.304758369300E-04,  0.317871039240E-02 },
	 { -0.856810657200E-07, -0.318476867010E-05 },
	 {  0.132281952950E-09,  0.157208190040E-08 },
	 { -0.170529583370E-12, -0.306913690560E-12 },
	 {  0.209480906970E-15,  0.000000000000E+00 },
	 { -0.125383953360E-18,  0.000000000000E+00 },
	 {  0.156317256970E-22,  0.000000000000E+00 }
};

// ranges for direct lookup
const double TypeJ::range_dir[2][2] = {
  { -210.000,  760 },
  {      760, 1200 }
};

const float TypeJ::C_max = 1200.0;
const float TypeJ::C_min = -210.0;

// -------------------------- constructor
TypeJ::TypeJ() {
  F_max = C_TO_F( C_max );
  F_min = C_TO_F( C_min );
}

// ------------------- given mv reading, returns absolute temp C
double TypeJ::Temp_C( float mv ) {
  double x = 1.0;
  double sum = 0.0;
  int i,j,ind;
  ind = 0;
  if ( ! inrange_mV( mv ) ) return TC_RANGE_ERR;
  // first figure out which range of values
  for( j = 0; j < nranges_inv; j++ ) {
    if((mv >= range_inv[0][j]) && (mv <= range_inv[1][j]))
      ind = j;
  };
//  Serial.println(ind);
  for( i = 0; i < ncoeff_inv; i++ ) {
    sum += x * coeff_inv[i][ind];
    x *= mv;
  }
  return sum;  
}

// --------- given mv reading and ambient temp, returns compensated (true)
//           temperature at tip of sensor
double TypeJ::Temp_C( float mv, float amb ) {
  float mv_amb;
  mv_amb = mV_C( amb );
  return Temp_C( mv + mv_amb );
};

// --------------------- returns compensated temperature in F units
double TypeJ::Temp_F( float mv, float amb ) {
  return C_TO_F( Temp_C( mv, F_TO_C( amb ) ) );
};

// --------------------- returns absolute temperature in F units
double TypeJ::Temp_F( float mv ) {
  float temp = Temp_C( mv );
  if( temp == TC_RANGE_ERR ) return TC_RANGE_ERR;
  return C_TO_F( temp );  
}

// --------------------- checks to make sure mv signal in range
boolean TypeJ::inrange_mV( float mv ) {
  return ( mv >= mv_min ) & ( mv <= mv_max );
};

// ---------------------- checks to make sure temperature in range
boolean TypeJ::inrange_C( float ambC ) {
  return ( ambC >= C_min ) & ( ambC <= C_max );
};

// ----------------------- checks to make sure temperature in range
boolean TypeJ::inrange_F( float ambF ) {
  return ( ambF >= F_min ) & ( ambF <= F_max );
};

// ---------------- returns mV corresponding to temp reading
//                  used for cold junction compensation
double TypeJ::mV_C( float ambC ) {
  double sum = 0.0;
  double x = 1.0;
  int i;
  if( !inrange_C( ambC ) ) return TC_RANGE_ERR;

  if( (ambC >= range_dir[0][0]) && ( ambC <= range_dir[1][0] ) ) {
    for( i = 0; i < 9; i++ ) {
      sum += x * coeff_dir[i][0];
      x *= ambC;
    } 
  }
  else {
    for( i = 0; i < 9; i++ ) {
      sum += x * coeff_dir[i][1];
      x *= ambC;    
    };
//    Serial.print( sum ); Serial.print(" , ");
  };  
  return sum;
};

// -------------------- cold junction compensation in F units
double TypeJ::mV_F( float ambF ) {
  if( inrange_F( ambF ) )
    return mV_C( F_TO_C( ambF ) );
  else
    return TC_RANGE_ERR;
};
