#include "gnss.h"

namespace {
#define DIM 11
#define EDIM 11
#define MEDIM 11
typedef void (*Hfun)(double *, double *, double *);
const static double MAHA_THRESH_6 = 3.8414588206941227;
const static double MAHA_THRESH_20 = 3.8414588206941227;
const static double MAHA_THRESH_7 = 3.8414588206941227;
const static double MAHA_THRESH_21 = 3.8414588206941227;

/******************************************************************************
 *                      Code generated with SymPy 1.11.1                      *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_7981808544055197912) {
   out_7981808544055197912[0] = delta_x[0] + nom_x[0];
   out_7981808544055197912[1] = delta_x[1] + nom_x[1];
   out_7981808544055197912[2] = delta_x[2] + nom_x[2];
   out_7981808544055197912[3] = delta_x[3] + nom_x[3];
   out_7981808544055197912[4] = delta_x[4] + nom_x[4];
   out_7981808544055197912[5] = delta_x[5] + nom_x[5];
   out_7981808544055197912[6] = delta_x[6] + nom_x[6];
   out_7981808544055197912[7] = delta_x[7] + nom_x[7];
   out_7981808544055197912[8] = delta_x[8] + nom_x[8];
   out_7981808544055197912[9] = delta_x[9] + nom_x[9];
   out_7981808544055197912[10] = delta_x[10] + nom_x[10];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_6481617288261504021) {
   out_6481617288261504021[0] = -nom_x[0] + true_x[0];
   out_6481617288261504021[1] = -nom_x[1] + true_x[1];
   out_6481617288261504021[2] = -nom_x[2] + true_x[2];
   out_6481617288261504021[3] = -nom_x[3] + true_x[3];
   out_6481617288261504021[4] = -nom_x[4] + true_x[4];
   out_6481617288261504021[5] = -nom_x[5] + true_x[5];
   out_6481617288261504021[6] = -nom_x[6] + true_x[6];
   out_6481617288261504021[7] = -nom_x[7] + true_x[7];
   out_6481617288261504021[8] = -nom_x[8] + true_x[8];
   out_6481617288261504021[9] = -nom_x[9] + true_x[9];
   out_6481617288261504021[10] = -nom_x[10] + true_x[10];
}
void H_mod_fun(double *state, double *out_3823724573421204829) {
   out_3823724573421204829[0] = 1.0;
   out_3823724573421204829[1] = 0;
   out_3823724573421204829[2] = 0;
   out_3823724573421204829[3] = 0;
   out_3823724573421204829[4] = 0;
   out_3823724573421204829[5] = 0;
   out_3823724573421204829[6] = 0;
   out_3823724573421204829[7] = 0;
   out_3823724573421204829[8] = 0;
   out_3823724573421204829[9] = 0;
   out_3823724573421204829[10] = 0;
   out_3823724573421204829[11] = 0;
   out_3823724573421204829[12] = 1.0;
   out_3823724573421204829[13] = 0;
   out_3823724573421204829[14] = 0;
   out_3823724573421204829[15] = 0;
   out_3823724573421204829[16] = 0;
   out_3823724573421204829[17] = 0;
   out_3823724573421204829[18] = 0;
   out_3823724573421204829[19] = 0;
   out_3823724573421204829[20] = 0;
   out_3823724573421204829[21] = 0;
   out_3823724573421204829[22] = 0;
   out_3823724573421204829[23] = 0;
   out_3823724573421204829[24] = 1.0;
   out_3823724573421204829[25] = 0;
   out_3823724573421204829[26] = 0;
   out_3823724573421204829[27] = 0;
   out_3823724573421204829[28] = 0;
   out_3823724573421204829[29] = 0;
   out_3823724573421204829[30] = 0;
   out_3823724573421204829[31] = 0;
   out_3823724573421204829[32] = 0;
   out_3823724573421204829[33] = 0;
   out_3823724573421204829[34] = 0;
   out_3823724573421204829[35] = 0;
   out_3823724573421204829[36] = 1.0;
   out_3823724573421204829[37] = 0;
   out_3823724573421204829[38] = 0;
   out_3823724573421204829[39] = 0;
   out_3823724573421204829[40] = 0;
   out_3823724573421204829[41] = 0;
   out_3823724573421204829[42] = 0;
   out_3823724573421204829[43] = 0;
   out_3823724573421204829[44] = 0;
   out_3823724573421204829[45] = 0;
   out_3823724573421204829[46] = 0;
   out_3823724573421204829[47] = 0;
   out_3823724573421204829[48] = 1.0;
   out_3823724573421204829[49] = 0;
   out_3823724573421204829[50] = 0;
   out_3823724573421204829[51] = 0;
   out_3823724573421204829[52] = 0;
   out_3823724573421204829[53] = 0;
   out_3823724573421204829[54] = 0;
   out_3823724573421204829[55] = 0;
   out_3823724573421204829[56] = 0;
   out_3823724573421204829[57] = 0;
   out_3823724573421204829[58] = 0;
   out_3823724573421204829[59] = 0;
   out_3823724573421204829[60] = 1.0;
   out_3823724573421204829[61] = 0;
   out_3823724573421204829[62] = 0;
   out_3823724573421204829[63] = 0;
   out_3823724573421204829[64] = 0;
   out_3823724573421204829[65] = 0;
   out_3823724573421204829[66] = 0;
   out_3823724573421204829[67] = 0;
   out_3823724573421204829[68] = 0;
   out_3823724573421204829[69] = 0;
   out_3823724573421204829[70] = 0;
   out_3823724573421204829[71] = 0;
   out_3823724573421204829[72] = 1.0;
   out_3823724573421204829[73] = 0;
   out_3823724573421204829[74] = 0;
   out_3823724573421204829[75] = 0;
   out_3823724573421204829[76] = 0;
   out_3823724573421204829[77] = 0;
   out_3823724573421204829[78] = 0;
   out_3823724573421204829[79] = 0;
   out_3823724573421204829[80] = 0;
   out_3823724573421204829[81] = 0;
   out_3823724573421204829[82] = 0;
   out_3823724573421204829[83] = 0;
   out_3823724573421204829[84] = 1.0;
   out_3823724573421204829[85] = 0;
   out_3823724573421204829[86] = 0;
   out_3823724573421204829[87] = 0;
   out_3823724573421204829[88] = 0;
   out_3823724573421204829[89] = 0;
   out_3823724573421204829[90] = 0;
   out_3823724573421204829[91] = 0;
   out_3823724573421204829[92] = 0;
   out_3823724573421204829[93] = 0;
   out_3823724573421204829[94] = 0;
   out_3823724573421204829[95] = 0;
   out_3823724573421204829[96] = 1.0;
   out_3823724573421204829[97] = 0;
   out_3823724573421204829[98] = 0;
   out_3823724573421204829[99] = 0;
   out_3823724573421204829[100] = 0;
   out_3823724573421204829[101] = 0;
   out_3823724573421204829[102] = 0;
   out_3823724573421204829[103] = 0;
   out_3823724573421204829[104] = 0;
   out_3823724573421204829[105] = 0;
   out_3823724573421204829[106] = 0;
   out_3823724573421204829[107] = 0;
   out_3823724573421204829[108] = 1.0;
   out_3823724573421204829[109] = 0;
   out_3823724573421204829[110] = 0;
   out_3823724573421204829[111] = 0;
   out_3823724573421204829[112] = 0;
   out_3823724573421204829[113] = 0;
   out_3823724573421204829[114] = 0;
   out_3823724573421204829[115] = 0;
   out_3823724573421204829[116] = 0;
   out_3823724573421204829[117] = 0;
   out_3823724573421204829[118] = 0;
   out_3823724573421204829[119] = 0;
   out_3823724573421204829[120] = 1.0;
}
void f_fun(double *state, double dt, double *out_5052117925629841371) {
   out_5052117925629841371[0] = dt*state[3] + state[0];
   out_5052117925629841371[1] = dt*state[4] + state[1];
   out_5052117925629841371[2] = dt*state[5] + state[2];
   out_5052117925629841371[3] = state[3];
   out_5052117925629841371[4] = state[4];
   out_5052117925629841371[5] = state[5];
   out_5052117925629841371[6] = dt*state[7] + state[6];
   out_5052117925629841371[7] = dt*state[8] + state[7];
   out_5052117925629841371[8] = state[8];
   out_5052117925629841371[9] = state[9];
   out_5052117925629841371[10] = state[10];
}
void F_fun(double *state, double dt, double *out_261666031851937057) {
   out_261666031851937057[0] = 1;
   out_261666031851937057[1] = 0;
   out_261666031851937057[2] = 0;
   out_261666031851937057[3] = dt;
   out_261666031851937057[4] = 0;
   out_261666031851937057[5] = 0;
   out_261666031851937057[6] = 0;
   out_261666031851937057[7] = 0;
   out_261666031851937057[8] = 0;
   out_261666031851937057[9] = 0;
   out_261666031851937057[10] = 0;
   out_261666031851937057[11] = 0;
   out_261666031851937057[12] = 1;
   out_261666031851937057[13] = 0;
   out_261666031851937057[14] = 0;
   out_261666031851937057[15] = dt;
   out_261666031851937057[16] = 0;
   out_261666031851937057[17] = 0;
   out_261666031851937057[18] = 0;
   out_261666031851937057[19] = 0;
   out_261666031851937057[20] = 0;
   out_261666031851937057[21] = 0;
   out_261666031851937057[22] = 0;
   out_261666031851937057[23] = 0;
   out_261666031851937057[24] = 1;
   out_261666031851937057[25] = 0;
   out_261666031851937057[26] = 0;
   out_261666031851937057[27] = dt;
   out_261666031851937057[28] = 0;
   out_261666031851937057[29] = 0;
   out_261666031851937057[30] = 0;
   out_261666031851937057[31] = 0;
   out_261666031851937057[32] = 0;
   out_261666031851937057[33] = 0;
   out_261666031851937057[34] = 0;
   out_261666031851937057[35] = 0;
   out_261666031851937057[36] = 1;
   out_261666031851937057[37] = 0;
   out_261666031851937057[38] = 0;
   out_261666031851937057[39] = 0;
   out_261666031851937057[40] = 0;
   out_261666031851937057[41] = 0;
   out_261666031851937057[42] = 0;
   out_261666031851937057[43] = 0;
   out_261666031851937057[44] = 0;
   out_261666031851937057[45] = 0;
   out_261666031851937057[46] = 0;
   out_261666031851937057[47] = 0;
   out_261666031851937057[48] = 1;
   out_261666031851937057[49] = 0;
   out_261666031851937057[50] = 0;
   out_261666031851937057[51] = 0;
   out_261666031851937057[52] = 0;
   out_261666031851937057[53] = 0;
   out_261666031851937057[54] = 0;
   out_261666031851937057[55] = 0;
   out_261666031851937057[56] = 0;
   out_261666031851937057[57] = 0;
   out_261666031851937057[58] = 0;
   out_261666031851937057[59] = 0;
   out_261666031851937057[60] = 1;
   out_261666031851937057[61] = 0;
   out_261666031851937057[62] = 0;
   out_261666031851937057[63] = 0;
   out_261666031851937057[64] = 0;
   out_261666031851937057[65] = 0;
   out_261666031851937057[66] = 0;
   out_261666031851937057[67] = 0;
   out_261666031851937057[68] = 0;
   out_261666031851937057[69] = 0;
   out_261666031851937057[70] = 0;
   out_261666031851937057[71] = 0;
   out_261666031851937057[72] = 1;
   out_261666031851937057[73] = dt;
   out_261666031851937057[74] = 0;
   out_261666031851937057[75] = 0;
   out_261666031851937057[76] = 0;
   out_261666031851937057[77] = 0;
   out_261666031851937057[78] = 0;
   out_261666031851937057[79] = 0;
   out_261666031851937057[80] = 0;
   out_261666031851937057[81] = 0;
   out_261666031851937057[82] = 0;
   out_261666031851937057[83] = 0;
   out_261666031851937057[84] = 1;
   out_261666031851937057[85] = dt;
   out_261666031851937057[86] = 0;
   out_261666031851937057[87] = 0;
   out_261666031851937057[88] = 0;
   out_261666031851937057[89] = 0;
   out_261666031851937057[90] = 0;
   out_261666031851937057[91] = 0;
   out_261666031851937057[92] = 0;
   out_261666031851937057[93] = 0;
   out_261666031851937057[94] = 0;
   out_261666031851937057[95] = 0;
   out_261666031851937057[96] = 1;
   out_261666031851937057[97] = 0;
   out_261666031851937057[98] = 0;
   out_261666031851937057[99] = 0;
   out_261666031851937057[100] = 0;
   out_261666031851937057[101] = 0;
   out_261666031851937057[102] = 0;
   out_261666031851937057[103] = 0;
   out_261666031851937057[104] = 0;
   out_261666031851937057[105] = 0;
   out_261666031851937057[106] = 0;
   out_261666031851937057[107] = 0;
   out_261666031851937057[108] = 1;
   out_261666031851937057[109] = 0;
   out_261666031851937057[110] = 0;
   out_261666031851937057[111] = 0;
   out_261666031851937057[112] = 0;
   out_261666031851937057[113] = 0;
   out_261666031851937057[114] = 0;
   out_261666031851937057[115] = 0;
   out_261666031851937057[116] = 0;
   out_261666031851937057[117] = 0;
   out_261666031851937057[118] = 0;
   out_261666031851937057[119] = 0;
   out_261666031851937057[120] = 1;
}
void h_6(double *state, double *sat_pos, double *out_7527303059176548152) {
   out_7527303059176548152[0] = sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2)) + state[6];
}
void H_6(double *state, double *sat_pos, double *out_455354385166966437) {
   out_455354385166966437[0] = (-sat_pos[0] + state[0])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_455354385166966437[1] = (-sat_pos[1] + state[1])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_455354385166966437[2] = (-sat_pos[2] + state[2])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_455354385166966437[3] = 0;
   out_455354385166966437[4] = 0;
   out_455354385166966437[5] = 0;
   out_455354385166966437[6] = 1;
   out_455354385166966437[7] = 0;
   out_455354385166966437[8] = 0;
   out_455354385166966437[9] = 0;
   out_455354385166966437[10] = 0;
}
void h_20(double *state, double *sat_pos, double *out_4786908733115082343) {
   out_4786908733115082343[0] = sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2)) + sat_pos[3]*state[10] + state[6] + state[9];
}
void H_20(double *state, double *sat_pos, double *out_2222481325922094845) {
   out_2222481325922094845[0] = (-sat_pos[0] + state[0])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_2222481325922094845[1] = (-sat_pos[1] + state[1])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_2222481325922094845[2] = (-sat_pos[2] + state[2])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_2222481325922094845[3] = 0;
   out_2222481325922094845[4] = 0;
   out_2222481325922094845[5] = 0;
   out_2222481325922094845[6] = 1;
   out_2222481325922094845[7] = 0;
   out_2222481325922094845[8] = 0;
   out_2222481325922094845[9] = 1;
   out_2222481325922094845[10] = sat_pos[3];
}
void h_7(double *state, double *sat_pos_vel, double *out_7736441670111324321) {
   out_7736441670111324321[0] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + state[7];
}
void H_7(double *state, double *sat_pos_vel, double *out_6741617175241692902) {
   out_6741617175241692902[0] = pow(sat_pos_vel[0] - state[0], 2)*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_6741617175241692902[1] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[1] - state[1], 2)*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_6741617175241692902[2] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[2] - state[2], 2)*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_6741617175241692902[3] = -(sat_pos_vel[0] - state[0])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_6741617175241692902[4] = -(sat_pos_vel[1] - state[1])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_6741617175241692902[5] = -(sat_pos_vel[2] - state[2])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_6741617175241692902[6] = 0;
   out_6741617175241692902[7] = 1;
   out_6741617175241692902[8] = 0;
   out_6741617175241692902[9] = 0;
   out_6741617175241692902[10] = 0;
}
void h_21(double *state, double *sat_pos_vel, double *out_7736441670111324321) {
   out_7736441670111324321[0] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + state[7];
}
void H_21(double *state, double *sat_pos_vel, double *out_6741617175241692902) {
   out_6741617175241692902[0] = pow(sat_pos_vel[0] - state[0], 2)*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_6741617175241692902[1] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[1] - state[1], 2)*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_6741617175241692902[2] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[2] - state[2], 2)*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_6741617175241692902[3] = -(sat_pos_vel[0] - state[0])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_6741617175241692902[4] = -(sat_pos_vel[1] - state[1])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_6741617175241692902[5] = -(sat_pos_vel[2] - state[2])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_6741617175241692902[6] = 0;
   out_6741617175241692902[7] = 1;
   out_6741617175241692902[8] = 0;
   out_6741617175241692902[9] = 0;
   out_6741617175241692902[10] = 0;
}
#include <eigen3/Eigen/Dense>
#include <iostream>

typedef Eigen::Matrix<double, DIM, DIM, Eigen::RowMajor> DDM;
typedef Eigen::Matrix<double, EDIM, EDIM, Eigen::RowMajor> EEM;
typedef Eigen::Matrix<double, DIM, EDIM, Eigen::RowMajor> DEM;

void predict(double *in_x, double *in_P, double *in_Q, double dt) {
  typedef Eigen::Matrix<double, MEDIM, MEDIM, Eigen::RowMajor> RRM;

  double nx[DIM] = {0};
  double in_F[EDIM*EDIM] = {0};

  // functions from sympy
  f_fun(in_x, dt, nx);
  F_fun(in_x, dt, in_F);


  EEM F(in_F);
  EEM P(in_P);
  EEM Q(in_Q);

  RRM F_main = F.topLeftCorner(MEDIM, MEDIM);
  P.topLeftCorner(MEDIM, MEDIM) = (F_main * P.topLeftCorner(MEDIM, MEDIM)) * F_main.transpose();
  P.topRightCorner(MEDIM, EDIM - MEDIM) = F_main * P.topRightCorner(MEDIM, EDIM - MEDIM);
  P.bottomLeftCorner(EDIM - MEDIM, MEDIM) = P.bottomLeftCorner(EDIM - MEDIM, MEDIM) * F_main.transpose();

  P = P + dt*Q;

  // copy out state
  memcpy(in_x, nx, DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
}

// note: extra_args dim only correct when null space projecting
// otherwise 1
template <int ZDIM, int EADIM, bool MAHA_TEST>
void update(double *in_x, double *in_P, Hfun h_fun, Hfun H_fun, Hfun Hea_fun, double *in_z, double *in_R, double *in_ea, double MAHA_THRESHOLD) {
  typedef Eigen::Matrix<double, ZDIM, ZDIM, Eigen::RowMajor> ZZM;
  typedef Eigen::Matrix<double, ZDIM, DIM, Eigen::RowMajor> ZDM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, EDIM, Eigen::RowMajor> XEM;
  //typedef Eigen::Matrix<double, EDIM, ZDIM, Eigen::RowMajor> EZM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, 1> X1M;
  typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> XXM;

  double in_hx[ZDIM] = {0};
  double in_H[ZDIM * DIM] = {0};
  double in_H_mod[EDIM * DIM] = {0};
  double delta_x[EDIM] = {0};
  double x_new[DIM] = {0};


  // state x, P
  Eigen::Matrix<double, ZDIM, 1> z(in_z);
  EEM P(in_P);
  ZZM pre_R(in_R);

  // functions from sympy
  h_fun(in_x, in_ea, in_hx);
  H_fun(in_x, in_ea, in_H);
  ZDM pre_H(in_H);

  // get y (y = z - hx)
  Eigen::Matrix<double, ZDIM, 1> pre_y(in_hx); pre_y = z - pre_y;
  X1M y; XXM H; XXM R;
  if (Hea_fun){
    typedef Eigen::Matrix<double, ZDIM, EADIM, Eigen::RowMajor> ZAM;
    double in_Hea[ZDIM * EADIM] = {0};
    Hea_fun(in_x, in_ea, in_Hea);
    ZAM Hea(in_Hea);
    XXM A = Hea.transpose().fullPivLu().kernel();


    y = A.transpose() * pre_y;
    H = A.transpose() * pre_H;
    R = A.transpose() * pre_R * A;
  } else {
    y = pre_y;
    H = pre_H;
    R = pre_R;
  }
  // get modified H
  H_mod_fun(in_x, in_H_mod);
  DEM H_mod(in_H_mod);
  XEM H_err = H * H_mod;

  // Do mahalobis distance test
  if (MAHA_TEST){
    XXM a = (H_err * P * H_err.transpose() + R).inverse();
    double maha_dist = y.transpose() * a * y;
    if (maha_dist > MAHA_THRESHOLD){
      R = 1.0e16 * R;
    }
  }

  // Outlier resilient weighting
  double weight = 1;//(1.5)/(1 + y.squaredNorm()/R.sum());

  // kalman gains and I_KH
  XXM S = ((H_err * P) * H_err.transpose()) + R/weight;
  XEM KT = S.fullPivLu().solve(H_err * P.transpose());
  //EZM K = KT.transpose(); TODO: WHY DOES THIS NOT COMPILE?
  //EZM K = S.fullPivLu().solve(H_err * P.transpose()).transpose();
  //std::cout << "Here is the matrix rot:\n" << K << std::endl;
  EEM I_KH = Eigen::Matrix<double, EDIM, EDIM>::Identity() - (KT.transpose() * H_err);

  // update state by injecting dx
  Eigen::Matrix<double, EDIM, 1> dx(delta_x);
  dx  = (KT.transpose() * y);
  memcpy(delta_x, dx.data(), EDIM * sizeof(double));
  err_fun(in_x, delta_x, x_new);
  Eigen::Matrix<double, DIM, 1> x(x_new);

  // update cov
  P = ((I_KH * P) * I_KH.transpose()) + ((KT.transpose() * R) * KT);

  // copy out state
  memcpy(in_x, x.data(), DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
  memcpy(in_z, y.data(), y.rows() * sizeof(double));
}




}
extern "C" {

void gnss_update_6(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_6, H_6, NULL, in_z, in_R, in_ea, MAHA_THRESH_6);
}
void gnss_update_20(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_20, H_20, NULL, in_z, in_R, in_ea, MAHA_THRESH_20);
}
void gnss_update_7(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_7, H_7, NULL, in_z, in_R, in_ea, MAHA_THRESH_7);
}
void gnss_update_21(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_21, H_21, NULL, in_z, in_R, in_ea, MAHA_THRESH_21);
}
void gnss_err_fun(double *nom_x, double *delta_x, double *out_7981808544055197912) {
  err_fun(nom_x, delta_x, out_7981808544055197912);
}
void gnss_inv_err_fun(double *nom_x, double *true_x, double *out_6481617288261504021) {
  inv_err_fun(nom_x, true_x, out_6481617288261504021);
}
void gnss_H_mod_fun(double *state, double *out_3823724573421204829) {
  H_mod_fun(state, out_3823724573421204829);
}
void gnss_f_fun(double *state, double dt, double *out_5052117925629841371) {
  f_fun(state,  dt, out_5052117925629841371);
}
void gnss_F_fun(double *state, double dt, double *out_261666031851937057) {
  F_fun(state,  dt, out_261666031851937057);
}
void gnss_h_6(double *state, double *sat_pos, double *out_7527303059176548152) {
  h_6(state, sat_pos, out_7527303059176548152);
}
void gnss_H_6(double *state, double *sat_pos, double *out_455354385166966437) {
  H_6(state, sat_pos, out_455354385166966437);
}
void gnss_h_20(double *state, double *sat_pos, double *out_4786908733115082343) {
  h_20(state, sat_pos, out_4786908733115082343);
}
void gnss_H_20(double *state, double *sat_pos, double *out_2222481325922094845) {
  H_20(state, sat_pos, out_2222481325922094845);
}
void gnss_h_7(double *state, double *sat_pos_vel, double *out_7736441670111324321) {
  h_7(state, sat_pos_vel, out_7736441670111324321);
}
void gnss_H_7(double *state, double *sat_pos_vel, double *out_6741617175241692902) {
  H_7(state, sat_pos_vel, out_6741617175241692902);
}
void gnss_h_21(double *state, double *sat_pos_vel, double *out_7736441670111324321) {
  h_21(state, sat_pos_vel, out_7736441670111324321);
}
void gnss_H_21(double *state, double *sat_pos_vel, double *out_6741617175241692902) {
  H_21(state, sat_pos_vel, out_6741617175241692902);
}
void gnss_predict(double *in_x, double *in_P, double *in_Q, double dt) {
  predict(in_x, in_P, in_Q, dt);
}
}

const EKF gnss = {
  .name = "gnss",
  .kinds = { 6, 20, 7, 21 },
  .feature_kinds = {  },
  .f_fun = gnss_f_fun,
  .F_fun = gnss_F_fun,
  .err_fun = gnss_err_fun,
  .inv_err_fun = gnss_inv_err_fun,
  .H_mod_fun = gnss_H_mod_fun,
  .predict = gnss_predict,
  .hs = {
    { 6, gnss_h_6 },
    { 20, gnss_h_20 },
    { 7, gnss_h_7 },
    { 21, gnss_h_21 },
  },
  .Hs = {
    { 6, gnss_H_6 },
    { 20, gnss_H_20 },
    { 7, gnss_H_7 },
    { 21, gnss_H_21 },
  },
  .updates = {
    { 6, gnss_update_6 },
    { 20, gnss_update_20 },
    { 7, gnss_update_7 },
    { 21, gnss_update_21 },
  },
  .Hes = {
  },
  .sets = {
  },
  .extra_routines = {
  },
};

ekf_init(gnss);
