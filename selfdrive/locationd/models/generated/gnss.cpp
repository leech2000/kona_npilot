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
void err_fun(double *nom_x, double *delta_x, double *out_309521215273340661) {
   out_309521215273340661[0] = delta_x[0] + nom_x[0];
   out_309521215273340661[1] = delta_x[1] + nom_x[1];
   out_309521215273340661[2] = delta_x[2] + nom_x[2];
   out_309521215273340661[3] = delta_x[3] + nom_x[3];
   out_309521215273340661[4] = delta_x[4] + nom_x[4];
   out_309521215273340661[5] = delta_x[5] + nom_x[5];
   out_309521215273340661[6] = delta_x[6] + nom_x[6];
   out_309521215273340661[7] = delta_x[7] + nom_x[7];
   out_309521215273340661[8] = delta_x[8] + nom_x[8];
   out_309521215273340661[9] = delta_x[9] + nom_x[9];
   out_309521215273340661[10] = delta_x[10] + nom_x[10];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_6124729898382051530) {
   out_6124729898382051530[0] = -nom_x[0] + true_x[0];
   out_6124729898382051530[1] = -nom_x[1] + true_x[1];
   out_6124729898382051530[2] = -nom_x[2] + true_x[2];
   out_6124729898382051530[3] = -nom_x[3] + true_x[3];
   out_6124729898382051530[4] = -nom_x[4] + true_x[4];
   out_6124729898382051530[5] = -nom_x[5] + true_x[5];
   out_6124729898382051530[6] = -nom_x[6] + true_x[6];
   out_6124729898382051530[7] = -nom_x[7] + true_x[7];
   out_6124729898382051530[8] = -nom_x[8] + true_x[8];
   out_6124729898382051530[9] = -nom_x[9] + true_x[9];
   out_6124729898382051530[10] = -nom_x[10] + true_x[10];
}
void H_mod_fun(double *state, double *out_3138109506697689559) {
   out_3138109506697689559[0] = 1.0;
   out_3138109506697689559[1] = 0;
   out_3138109506697689559[2] = 0;
   out_3138109506697689559[3] = 0;
   out_3138109506697689559[4] = 0;
   out_3138109506697689559[5] = 0;
   out_3138109506697689559[6] = 0;
   out_3138109506697689559[7] = 0;
   out_3138109506697689559[8] = 0;
   out_3138109506697689559[9] = 0;
   out_3138109506697689559[10] = 0;
   out_3138109506697689559[11] = 0;
   out_3138109506697689559[12] = 1.0;
   out_3138109506697689559[13] = 0;
   out_3138109506697689559[14] = 0;
   out_3138109506697689559[15] = 0;
   out_3138109506697689559[16] = 0;
   out_3138109506697689559[17] = 0;
   out_3138109506697689559[18] = 0;
   out_3138109506697689559[19] = 0;
   out_3138109506697689559[20] = 0;
   out_3138109506697689559[21] = 0;
   out_3138109506697689559[22] = 0;
   out_3138109506697689559[23] = 0;
   out_3138109506697689559[24] = 1.0;
   out_3138109506697689559[25] = 0;
   out_3138109506697689559[26] = 0;
   out_3138109506697689559[27] = 0;
   out_3138109506697689559[28] = 0;
   out_3138109506697689559[29] = 0;
   out_3138109506697689559[30] = 0;
   out_3138109506697689559[31] = 0;
   out_3138109506697689559[32] = 0;
   out_3138109506697689559[33] = 0;
   out_3138109506697689559[34] = 0;
   out_3138109506697689559[35] = 0;
   out_3138109506697689559[36] = 1.0;
   out_3138109506697689559[37] = 0;
   out_3138109506697689559[38] = 0;
   out_3138109506697689559[39] = 0;
   out_3138109506697689559[40] = 0;
   out_3138109506697689559[41] = 0;
   out_3138109506697689559[42] = 0;
   out_3138109506697689559[43] = 0;
   out_3138109506697689559[44] = 0;
   out_3138109506697689559[45] = 0;
   out_3138109506697689559[46] = 0;
   out_3138109506697689559[47] = 0;
   out_3138109506697689559[48] = 1.0;
   out_3138109506697689559[49] = 0;
   out_3138109506697689559[50] = 0;
   out_3138109506697689559[51] = 0;
   out_3138109506697689559[52] = 0;
   out_3138109506697689559[53] = 0;
   out_3138109506697689559[54] = 0;
   out_3138109506697689559[55] = 0;
   out_3138109506697689559[56] = 0;
   out_3138109506697689559[57] = 0;
   out_3138109506697689559[58] = 0;
   out_3138109506697689559[59] = 0;
   out_3138109506697689559[60] = 1.0;
   out_3138109506697689559[61] = 0;
   out_3138109506697689559[62] = 0;
   out_3138109506697689559[63] = 0;
   out_3138109506697689559[64] = 0;
   out_3138109506697689559[65] = 0;
   out_3138109506697689559[66] = 0;
   out_3138109506697689559[67] = 0;
   out_3138109506697689559[68] = 0;
   out_3138109506697689559[69] = 0;
   out_3138109506697689559[70] = 0;
   out_3138109506697689559[71] = 0;
   out_3138109506697689559[72] = 1.0;
   out_3138109506697689559[73] = 0;
   out_3138109506697689559[74] = 0;
   out_3138109506697689559[75] = 0;
   out_3138109506697689559[76] = 0;
   out_3138109506697689559[77] = 0;
   out_3138109506697689559[78] = 0;
   out_3138109506697689559[79] = 0;
   out_3138109506697689559[80] = 0;
   out_3138109506697689559[81] = 0;
   out_3138109506697689559[82] = 0;
   out_3138109506697689559[83] = 0;
   out_3138109506697689559[84] = 1.0;
   out_3138109506697689559[85] = 0;
   out_3138109506697689559[86] = 0;
   out_3138109506697689559[87] = 0;
   out_3138109506697689559[88] = 0;
   out_3138109506697689559[89] = 0;
   out_3138109506697689559[90] = 0;
   out_3138109506697689559[91] = 0;
   out_3138109506697689559[92] = 0;
   out_3138109506697689559[93] = 0;
   out_3138109506697689559[94] = 0;
   out_3138109506697689559[95] = 0;
   out_3138109506697689559[96] = 1.0;
   out_3138109506697689559[97] = 0;
   out_3138109506697689559[98] = 0;
   out_3138109506697689559[99] = 0;
   out_3138109506697689559[100] = 0;
   out_3138109506697689559[101] = 0;
   out_3138109506697689559[102] = 0;
   out_3138109506697689559[103] = 0;
   out_3138109506697689559[104] = 0;
   out_3138109506697689559[105] = 0;
   out_3138109506697689559[106] = 0;
   out_3138109506697689559[107] = 0;
   out_3138109506697689559[108] = 1.0;
   out_3138109506697689559[109] = 0;
   out_3138109506697689559[110] = 0;
   out_3138109506697689559[111] = 0;
   out_3138109506697689559[112] = 0;
   out_3138109506697689559[113] = 0;
   out_3138109506697689559[114] = 0;
   out_3138109506697689559[115] = 0;
   out_3138109506697689559[116] = 0;
   out_3138109506697689559[117] = 0;
   out_3138109506697689559[118] = 0;
   out_3138109506697689559[119] = 0;
   out_3138109506697689559[120] = 1.0;
}
void f_fun(double *state, double dt, double *out_7355617933517656719) {
   out_7355617933517656719[0] = dt*state[3] + state[0];
   out_7355617933517656719[1] = dt*state[4] + state[1];
   out_7355617933517656719[2] = dt*state[5] + state[2];
   out_7355617933517656719[3] = state[3];
   out_7355617933517656719[4] = state[4];
   out_7355617933517656719[5] = state[5];
   out_7355617933517656719[6] = dt*state[7] + state[6];
   out_7355617933517656719[7] = dt*state[8] + state[7];
   out_7355617933517656719[8] = state[8];
   out_7355617933517656719[9] = state[9];
   out_7355617933517656719[10] = state[10];
}
void F_fun(double *state, double dt, double *out_3794844325727046215) {
   out_3794844325727046215[0] = 1;
   out_3794844325727046215[1] = 0;
   out_3794844325727046215[2] = 0;
   out_3794844325727046215[3] = dt;
   out_3794844325727046215[4] = 0;
   out_3794844325727046215[5] = 0;
   out_3794844325727046215[6] = 0;
   out_3794844325727046215[7] = 0;
   out_3794844325727046215[8] = 0;
   out_3794844325727046215[9] = 0;
   out_3794844325727046215[10] = 0;
   out_3794844325727046215[11] = 0;
   out_3794844325727046215[12] = 1;
   out_3794844325727046215[13] = 0;
   out_3794844325727046215[14] = 0;
   out_3794844325727046215[15] = dt;
   out_3794844325727046215[16] = 0;
   out_3794844325727046215[17] = 0;
   out_3794844325727046215[18] = 0;
   out_3794844325727046215[19] = 0;
   out_3794844325727046215[20] = 0;
   out_3794844325727046215[21] = 0;
   out_3794844325727046215[22] = 0;
   out_3794844325727046215[23] = 0;
   out_3794844325727046215[24] = 1;
   out_3794844325727046215[25] = 0;
   out_3794844325727046215[26] = 0;
   out_3794844325727046215[27] = dt;
   out_3794844325727046215[28] = 0;
   out_3794844325727046215[29] = 0;
   out_3794844325727046215[30] = 0;
   out_3794844325727046215[31] = 0;
   out_3794844325727046215[32] = 0;
   out_3794844325727046215[33] = 0;
   out_3794844325727046215[34] = 0;
   out_3794844325727046215[35] = 0;
   out_3794844325727046215[36] = 1;
   out_3794844325727046215[37] = 0;
   out_3794844325727046215[38] = 0;
   out_3794844325727046215[39] = 0;
   out_3794844325727046215[40] = 0;
   out_3794844325727046215[41] = 0;
   out_3794844325727046215[42] = 0;
   out_3794844325727046215[43] = 0;
   out_3794844325727046215[44] = 0;
   out_3794844325727046215[45] = 0;
   out_3794844325727046215[46] = 0;
   out_3794844325727046215[47] = 0;
   out_3794844325727046215[48] = 1;
   out_3794844325727046215[49] = 0;
   out_3794844325727046215[50] = 0;
   out_3794844325727046215[51] = 0;
   out_3794844325727046215[52] = 0;
   out_3794844325727046215[53] = 0;
   out_3794844325727046215[54] = 0;
   out_3794844325727046215[55] = 0;
   out_3794844325727046215[56] = 0;
   out_3794844325727046215[57] = 0;
   out_3794844325727046215[58] = 0;
   out_3794844325727046215[59] = 0;
   out_3794844325727046215[60] = 1;
   out_3794844325727046215[61] = 0;
   out_3794844325727046215[62] = 0;
   out_3794844325727046215[63] = 0;
   out_3794844325727046215[64] = 0;
   out_3794844325727046215[65] = 0;
   out_3794844325727046215[66] = 0;
   out_3794844325727046215[67] = 0;
   out_3794844325727046215[68] = 0;
   out_3794844325727046215[69] = 0;
   out_3794844325727046215[70] = 0;
   out_3794844325727046215[71] = 0;
   out_3794844325727046215[72] = 1;
   out_3794844325727046215[73] = dt;
   out_3794844325727046215[74] = 0;
   out_3794844325727046215[75] = 0;
   out_3794844325727046215[76] = 0;
   out_3794844325727046215[77] = 0;
   out_3794844325727046215[78] = 0;
   out_3794844325727046215[79] = 0;
   out_3794844325727046215[80] = 0;
   out_3794844325727046215[81] = 0;
   out_3794844325727046215[82] = 0;
   out_3794844325727046215[83] = 0;
   out_3794844325727046215[84] = 1;
   out_3794844325727046215[85] = dt;
   out_3794844325727046215[86] = 0;
   out_3794844325727046215[87] = 0;
   out_3794844325727046215[88] = 0;
   out_3794844325727046215[89] = 0;
   out_3794844325727046215[90] = 0;
   out_3794844325727046215[91] = 0;
   out_3794844325727046215[92] = 0;
   out_3794844325727046215[93] = 0;
   out_3794844325727046215[94] = 0;
   out_3794844325727046215[95] = 0;
   out_3794844325727046215[96] = 1;
   out_3794844325727046215[97] = 0;
   out_3794844325727046215[98] = 0;
   out_3794844325727046215[99] = 0;
   out_3794844325727046215[100] = 0;
   out_3794844325727046215[101] = 0;
   out_3794844325727046215[102] = 0;
   out_3794844325727046215[103] = 0;
   out_3794844325727046215[104] = 0;
   out_3794844325727046215[105] = 0;
   out_3794844325727046215[106] = 0;
   out_3794844325727046215[107] = 0;
   out_3794844325727046215[108] = 1;
   out_3794844325727046215[109] = 0;
   out_3794844325727046215[110] = 0;
   out_3794844325727046215[111] = 0;
   out_3794844325727046215[112] = 0;
   out_3794844325727046215[113] = 0;
   out_3794844325727046215[114] = 0;
   out_3794844325727046215[115] = 0;
   out_3794844325727046215[116] = 0;
   out_3794844325727046215[117] = 0;
   out_3794844325727046215[118] = 0;
   out_3794844325727046215[119] = 0;
   out_3794844325727046215[120] = 1;
}
void h_6(double *state, double *sat_pos, double *out_8872459805762957153) {
   out_8872459805762957153[0] = sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2)) + state[6];
}
void H_6(double *state, double *sat_pos, double *out_8665813077433040777) {
   out_8665813077433040777[0] = (-sat_pos[0] + state[0])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_8665813077433040777[1] = (-sat_pos[1] + state[1])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_8665813077433040777[2] = (-sat_pos[2] + state[2])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_8665813077433040777[3] = 0;
   out_8665813077433040777[4] = 0;
   out_8665813077433040777[5] = 0;
   out_8665813077433040777[6] = 1;
   out_8665813077433040777[7] = 0;
   out_8665813077433040777[8] = 0;
   out_8665813077433040777[9] = 0;
   out_8665813077433040777[10] = 0;
}
void h_20(double *state, double *sat_pos, double *out_4040060993076324013) {
   out_4040060993076324013[0] = sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2)) + sat_pos[3]*state[10] + state[6] + state[9];
}
void H_20(double *state, double *sat_pos, double *out_8911946568174512483) {
   out_8911946568174512483[0] = (-sat_pos[0] + state[0])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_8911946568174512483[1] = (-sat_pos[1] + state[1])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_8911946568174512483[2] = (-sat_pos[2] + state[2])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_8911946568174512483[3] = 0;
   out_8911946568174512483[4] = 0;
   out_8911946568174512483[5] = 0;
   out_8911946568174512483[6] = 1;
   out_8911946568174512483[7] = 0;
   out_8911946568174512483[8] = 0;
   out_8911946568174512483[9] = 1;
   out_8911946568174512483[10] = sat_pos[3];
}
void h_7(double *state, double *sat_pos_vel, double *out_5571405682832922983) {
   out_5571405682832922983[0] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + state[7];
}
void H_7(double *state, double *sat_pos_vel, double *out_4115633887760809029) {
   out_4115633887760809029[0] = pow(sat_pos_vel[0] - state[0], 2)*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_4115633887760809029[1] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[1] - state[1], 2)*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_4115633887760809029[2] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[2] - state[2], 2)*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_4115633887760809029[3] = -(sat_pos_vel[0] - state[0])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_4115633887760809029[4] = -(sat_pos_vel[1] - state[1])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_4115633887760809029[5] = -(sat_pos_vel[2] - state[2])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_4115633887760809029[6] = 0;
   out_4115633887760809029[7] = 1;
   out_4115633887760809029[8] = 0;
   out_4115633887760809029[9] = 0;
   out_4115633887760809029[10] = 0;
}
void h_21(double *state, double *sat_pos_vel, double *out_5571405682832922983) {
   out_5571405682832922983[0] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + state[7];
}
void H_21(double *state, double *sat_pos_vel, double *out_4115633887760809029) {
   out_4115633887760809029[0] = pow(sat_pos_vel[0] - state[0], 2)*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_4115633887760809029[1] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[1] - state[1], 2)*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_4115633887760809029[2] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[2] - state[2], 2)*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_4115633887760809029[3] = -(sat_pos_vel[0] - state[0])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_4115633887760809029[4] = -(sat_pos_vel[1] - state[1])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_4115633887760809029[5] = -(sat_pos_vel[2] - state[2])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_4115633887760809029[6] = 0;
   out_4115633887760809029[7] = 1;
   out_4115633887760809029[8] = 0;
   out_4115633887760809029[9] = 0;
   out_4115633887760809029[10] = 0;
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
void gnss_err_fun(double *nom_x, double *delta_x, double *out_309521215273340661) {
  err_fun(nom_x, delta_x, out_309521215273340661);
}
void gnss_inv_err_fun(double *nom_x, double *true_x, double *out_6124729898382051530) {
  inv_err_fun(nom_x, true_x, out_6124729898382051530);
}
void gnss_H_mod_fun(double *state, double *out_3138109506697689559) {
  H_mod_fun(state, out_3138109506697689559);
}
void gnss_f_fun(double *state, double dt, double *out_7355617933517656719) {
  f_fun(state,  dt, out_7355617933517656719);
}
void gnss_F_fun(double *state, double dt, double *out_3794844325727046215) {
  F_fun(state,  dt, out_3794844325727046215);
}
void gnss_h_6(double *state, double *sat_pos, double *out_8872459805762957153) {
  h_6(state, sat_pos, out_8872459805762957153);
}
void gnss_H_6(double *state, double *sat_pos, double *out_8665813077433040777) {
  H_6(state, sat_pos, out_8665813077433040777);
}
void gnss_h_20(double *state, double *sat_pos, double *out_4040060993076324013) {
  h_20(state, sat_pos, out_4040060993076324013);
}
void gnss_H_20(double *state, double *sat_pos, double *out_8911946568174512483) {
  H_20(state, sat_pos, out_8911946568174512483);
}
void gnss_h_7(double *state, double *sat_pos_vel, double *out_5571405682832922983) {
  h_7(state, sat_pos_vel, out_5571405682832922983);
}
void gnss_H_7(double *state, double *sat_pos_vel, double *out_4115633887760809029) {
  H_7(state, sat_pos_vel, out_4115633887760809029);
}
void gnss_h_21(double *state, double *sat_pos_vel, double *out_5571405682832922983) {
  h_21(state, sat_pos_vel, out_5571405682832922983);
}
void gnss_H_21(double *state, double *sat_pos_vel, double *out_4115633887760809029) {
  H_21(state, sat_pos_vel, out_4115633887760809029);
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
