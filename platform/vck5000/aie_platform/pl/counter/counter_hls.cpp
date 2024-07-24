/*
Copyright (C) 2024, Advanced Micro Devices, Inc. All rights reserved.
SPDX-License-Identifier: MIT
*/

/*

Creating a AXIS IP. The functionality of this IP has not been
tested as its pure function is to be used to instantiate the 
PLIO interfaces on the AIE IP.

*/				      

#include "utils/x_hls_utils.h"
#include "assert.h"
#include <stdio.h>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <string>
#include <math.h>
#include <hls_stream.h>
#include "ap_int.h"
#include <stdint.h>
#include <cstdlib>
#include "ap_axi_sdata.h"

#define keepsize(n)   (1<<n)
#define keepval(n)   (keepsize(n) - 1)

#define SIZE 32

const int DW        = 64;  
const int UW        = 8; 
const int KW        = DW/8;

using namespace hls;

void counter(hls::stream<ap_axis<DW,0,0,0> > &strm_out)
{
 ap_axis<DW,0,0,0>   	  s;
 ap_uint<64>					  mm_data;

 s.last = 0;
 s.keep = 0;

  for(int i = 0; ; i++) {
    s.data = mm_data;
    s.keep = keepval(KW);
    if (i == SIZE-1)
      s.last = 1;
    else
      s.last = 0;
    strm_out.write(s);
  }

}

// Just want to be able to see the data coming out
void axis_stub(hls::stream<ap_axis<DW,0,0,0> > &strm_in)
{
 ap_axis<DW,0,0,0>   	  s;
 
 for(int i = 0; ; i++)
 {
	s = strm_in.read();
 }
}

extern "C" {

void counter_hls(hls::stream<ap_axis<DW,0,0,0> > &strm_out, hls::stream<ap_axis<DW,0,0,0> > &strm_in)
{
  
#pragma HLS INTERFACE axis port=strm_out
#pragma HLS INTERFACE axis port=strm_in
  counter (strm_out);
  axis_stub(strm_in);
}

}
