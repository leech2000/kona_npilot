#include "car.h"

namespace {
#define DIM 9
#define EDIM 9
#define MEDIM 9
typedef void (*Hfun)(double *, double *, double *);

double mass;

void set_mass(double x){ mass = x;}

double rotational_inertia;

void set_rotational_inertia(double x){ rotational_inertia = x;}

double center_to_front;

void set_center_to_front(double x){ center_to_front = x;}

double center_to_rear;

void set_center_to_rear(double x){ center_to_rear = x;}

double stiffness_front;

void set_stiffness_front(double x){ stiffness_front = x;}

double stiffness_rear;

void set_stiffness_rear(double x){ stiffness_rear = x;}
const static double MAHA_THRESH_25 = 3.8414588206941227;
const static double MAHA_THRESH_24 = 5.991464547107981;
const static double MAHA_THRESH_30 = 3.8414588206941227;
const static double MAHA_THRESH_26 = 3.8414588206941227;
const static double MAHA_THRESH_27 = 3.8414588206941227;
const static double MAHA_THRESH_29 = 3.8414588206941227;
const static double MAHA_THRESH_28 = 3.8414588206941227;
const static double MAHA_THRESH_31 = 3.8414588206941227;

/******************************************************************************
 *                      Code generated with SymPy 1.11.1                      *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_2397847969433383157) {
   out_2397847969433383157[0] = delta_x[0] + nom_x[0];
   out_2397847969433383157[1] = delta_x[1] + nom_x[1];
   out_2397847969433383157[2] = delta_x[2] + nom_x[2];
   out_2397847969433383157[3] = delta_x[3] + nom_x[3];
   out_2397847969433383157[4] = delta_x[4] + nom_x[4];
   out_2397847969433383157[5] = delta_x[5] + nom_x[5];
   out_2397847969433383157[6] = delta_x[6] + nom_x[6];
   out_2397847969433383157[7] = delta_x[7] + nom_x[7];
   out_2397847969433383157[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_8621666753450333961) {
   out_8621666753450333961[0] = -nom_x[0] + true_x[0];
   out_8621666753450333961[1] = -nom_x[1] + true_x[1];
   out_8621666753450333961[2] = -nom_x[2] + true_x[2];
   out_8621666753450333961[3] = -nom_x[3] + true_x[3];
   out_8621666753450333961[4] = -nom_x[4] + true_x[4];
   out_8621666753450333961[5] = -nom_x[5] + true_x[5];
   out_8621666753450333961[6] = -nom_x[6] + true_x[6];
   out_8621666753450333961[7] = -nom_x[7] + true_x[7];
   out_8621666753450333961[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_2589147812390356385) {
   out_2589147812390356385[0] = 1.0;
   out_2589147812390356385[1] = 0;
   out_2589147812390356385[2] = 0;
   out_2589147812390356385[3] = 0;
   out_2589147812390356385[4] = 0;
   out_2589147812390356385[5] = 0;
   out_2589147812390356385[6] = 0;
   out_2589147812390356385[7] = 0;
   out_2589147812390356385[8] = 0;
   out_2589147812390356385[9] = 0;
   out_2589147812390356385[10] = 1.0;
   out_2589147812390356385[11] = 0;
   out_2589147812390356385[12] = 0;
   out_2589147812390356385[13] = 0;
   out_2589147812390356385[14] = 0;
   out_2589147812390356385[15] = 0;
   out_2589147812390356385[16] = 0;
   out_2589147812390356385[17] = 0;
   out_2589147812390356385[18] = 0;
   out_2589147812390356385[19] = 0;
   out_2589147812390356385[20] = 1.0;
   out_2589147812390356385[21] = 0;
   out_2589147812390356385[22] = 0;
   out_2589147812390356385[23] = 0;
   out_2589147812390356385[24] = 0;
   out_2589147812390356385[25] = 0;
   out_2589147812390356385[26] = 0;
   out_2589147812390356385[27] = 0;
   out_2589147812390356385[28] = 0;
   out_2589147812390356385[29] = 0;
   out_2589147812390356385[30] = 1.0;
   out_2589147812390356385[31] = 0;
   out_2589147812390356385[32] = 0;
   out_2589147812390356385[33] = 0;
   out_2589147812390356385[34] = 0;
   out_2589147812390356385[35] = 0;
   out_2589147812390356385[36] = 0;
   out_2589147812390356385[37] = 0;
   out_2589147812390356385[38] = 0;
   out_2589147812390356385[39] = 0;
   out_2589147812390356385[40] = 1.0;
   out_2589147812390356385[41] = 0;
   out_2589147812390356385[42] = 0;
   out_2589147812390356385[43] = 0;
   out_2589147812390356385[44] = 0;
   out_2589147812390356385[45] = 0;
   out_2589147812390356385[46] = 0;
   out_2589147812390356385[47] = 0;
   out_2589147812390356385[48] = 0;
   out_2589147812390356385[49] = 0;
   out_2589147812390356385[50] = 1.0;
   out_2589147812390356385[51] = 0;
   out_2589147812390356385[52] = 0;
   out_2589147812390356385[53] = 0;
   out_2589147812390356385[54] = 0;
   out_2589147812390356385[55] = 0;
   out_2589147812390356385[56] = 0;
   out_2589147812390356385[57] = 0;
   out_2589147812390356385[58] = 0;
   out_2589147812390356385[59] = 0;
   out_2589147812390356385[60] = 1.0;
   out_2589147812390356385[61] = 0;
   out_2589147812390356385[62] = 0;
   out_2589147812390356385[63] = 0;
   out_2589147812390356385[64] = 0;
   out_2589147812390356385[65] = 0;
   out_2589147812390356385[66] = 0;
   out_2589147812390356385[67] = 0;
   out_2589147812390356385[68] = 0;
   out_2589147812390356385[69] = 0;
   out_2589147812390356385[70] = 1.0;
   out_2589147812390356385[71] = 0;
   out_2589147812390356385[72] = 0;
   out_2589147812390356385[73] = 0;
   out_2589147812390356385[74] = 0;
   out_2589147812390356385[75] = 0;
   out_2589147812390356385[76] = 0;
   out_2589147812390356385[77] = 0;
   out_2589147812390356385[78] = 0;
   out_2589147812390356385[79] = 0;
   out_2589147812390356385[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_9011514071817120336) {
   out_9011514071817120336[0] = state[0];
   out_9011514071817120336[1] = state[1];
   out_9011514071817120336[2] = state[2];
   out_9011514071817120336[3] = state[3];
   out_9011514071817120336[4] = state[4];
   out_9011514071817120336[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_9011514071817120336[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_9011514071817120336[7] = state[7];
   out_9011514071817120336[8] = state[8];
}
void F_fun(double *state, double dt, double *out_5945821314829573806) {
   out_5945821314829573806[0] = 1;
   out_5945821314829573806[1] = 0;
   out_5945821314829573806[2] = 0;
   out_5945821314829573806[3] = 0;
   out_5945821314829573806[4] = 0;
   out_5945821314829573806[5] = 0;
   out_5945821314829573806[6] = 0;
   out_5945821314829573806[7] = 0;
   out_5945821314829573806[8] = 0;
   out_5945821314829573806[9] = 0;
   out_5945821314829573806[10] = 1;
   out_5945821314829573806[11] = 0;
   out_5945821314829573806[12] = 0;
   out_5945821314829573806[13] = 0;
   out_5945821314829573806[14] = 0;
   out_5945821314829573806[15] = 0;
   out_5945821314829573806[16] = 0;
   out_5945821314829573806[17] = 0;
   out_5945821314829573806[18] = 0;
   out_5945821314829573806[19] = 0;
   out_5945821314829573806[20] = 1;
   out_5945821314829573806[21] = 0;
   out_5945821314829573806[22] = 0;
   out_5945821314829573806[23] = 0;
   out_5945821314829573806[24] = 0;
   out_5945821314829573806[25] = 0;
   out_5945821314829573806[26] = 0;
   out_5945821314829573806[27] = 0;
   out_5945821314829573806[28] = 0;
   out_5945821314829573806[29] = 0;
   out_5945821314829573806[30] = 1;
   out_5945821314829573806[31] = 0;
   out_5945821314829573806[32] = 0;
   out_5945821314829573806[33] = 0;
   out_5945821314829573806[34] = 0;
   out_5945821314829573806[35] = 0;
   out_5945821314829573806[36] = 0;
   out_5945821314829573806[37] = 0;
   out_5945821314829573806[38] = 0;
   out_5945821314829573806[39] = 0;
   out_5945821314829573806[40] = 1;
   out_5945821314829573806[41] = 0;
   out_5945821314829573806[42] = 0;
   out_5945821314829573806[43] = 0;
   out_5945821314829573806[44] = 0;
   out_5945821314829573806[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_5945821314829573806[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_5945821314829573806[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_5945821314829573806[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_5945821314829573806[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_5945821314829573806[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_5945821314829573806[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_5945821314829573806[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_5945821314829573806[53] = -9.8000000000000007*dt;
   out_5945821314829573806[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_5945821314829573806[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_5945821314829573806[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_5945821314829573806[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_5945821314829573806[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_5945821314829573806[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_5945821314829573806[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_5945821314829573806[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_5945821314829573806[62] = 0;
   out_5945821314829573806[63] = 0;
   out_5945821314829573806[64] = 0;
   out_5945821314829573806[65] = 0;
   out_5945821314829573806[66] = 0;
   out_5945821314829573806[67] = 0;
   out_5945821314829573806[68] = 0;
   out_5945821314829573806[69] = 0;
   out_5945821314829573806[70] = 1;
   out_5945821314829573806[71] = 0;
   out_5945821314829573806[72] = 0;
   out_5945821314829573806[73] = 0;
   out_5945821314829573806[74] = 0;
   out_5945821314829573806[75] = 0;
   out_5945821314829573806[76] = 0;
   out_5945821314829573806[77] = 0;
   out_5945821314829573806[78] = 0;
   out_5945821314829573806[79] = 0;
   out_5945821314829573806[80] = 1;
}
void h_25(double *state, double *unused, double *out_3193307014990671124) {
   out_3193307014990671124[0] = state[6];
}
void H_25(double *state, double *unused, double *out_3439607463114543200) {
   out_3439607463114543200[0] = 0;
   out_3439607463114543200[1] = 0;
   out_3439607463114543200[2] = 0;
   out_3439607463114543200[3] = 0;
   out_3439607463114543200[4] = 0;
   out_3439607463114543200[5] = 0;
   out_3439607463114543200[6] = 1;
   out_3439607463114543200[7] = 0;
   out_3439607463114543200[8] = 0;
}
void h_24(double *state, double *unused, double *out_1870912835991722903) {
   out_1870912835991722903[0] = state[4];
   out_1870912835991722903[1] = state[5];
}
void H_24(double *state, double *unused, double *out_7564462017606292096) {
   out_7564462017606292096[0] = 0;
   out_7564462017606292096[1] = 0;
   out_7564462017606292096[2] = 0;
   out_7564462017606292096[3] = 0;
   out_7564462017606292096[4] = 1;
   out_7564462017606292096[5] = 0;
   out_7564462017606292096[6] = 0;
   out_7564462017606292096[7] = 0;
   out_7564462017606292096[8] = 0;
   out_7564462017606292096[9] = 0;
   out_7564462017606292096[10] = 0;
   out_7564462017606292096[11] = 0;
   out_7564462017606292096[12] = 0;
   out_7564462017606292096[13] = 0;
   out_7564462017606292096[14] = 1;
   out_7564462017606292096[15] = 0;
   out_7564462017606292096[16] = 0;
   out_7564462017606292096[17] = 0;
}
void h_30(double *state, double *unused, double *out_6413933887999061586) {
   out_6413933887999061586[0] = state[4];
}
void H_30(double *state, double *unused, double *out_3568946410257783270) {
   out_3568946410257783270[0] = 0;
   out_3568946410257783270[1] = 0;
   out_3568946410257783270[2] = 0;
   out_3568946410257783270[3] = 0;
   out_3568946410257783270[4] = 1;
   out_3568946410257783270[5] = 0;
   out_3568946410257783270[6] = 0;
   out_3568946410257783270[7] = 0;
   out_3568946410257783270[8] = 0;
}
void h_26(double *state, double *unused, double *out_8794105260633279935) {
   out_8794105260633279935[0] = state[7];
}
void H_26(double *state, double *unused, double *out_7181110781988599424) {
   out_7181110781988599424[0] = 0;
   out_7181110781988599424[1] = 0;
   out_7181110781988599424[2] = 0;
   out_7181110781988599424[3] = 0;
   out_7181110781988599424[4] = 0;
   out_7181110781988599424[5] = 0;
   out_7181110781988599424[6] = 0;
   out_7181110781988599424[7] = 1;
   out_7181110781988599424[8] = 0;
}
void h_27(double *state, double *unused, double *out_6742055710391033214) {
   out_6742055710391033214[0] = state[3];
}
void H_27(double *state, double *unused, double *out_5743709722058208181) {
   out_5743709722058208181[0] = 0;
   out_5743709722058208181[1] = 0;
   out_5743709722058208181[2] = 0;
   out_5743709722058208181[3] = 1;
   out_5743709722058208181[4] = 0;
   out_5743709722058208181[5] = 0;
   out_5743709722058208181[6] = 0;
   out_5743709722058208181[7] = 0;
   out_5743709722058208181[8] = 0;
}
void h_29(double *state, double *unused, double *out_6390080314629533195) {
   out_6390080314629533195[0] = state[1];
}
void H_29(double *state, double *unused, double *out_7457072448927759214) {
   out_7457072448927759214[0] = 0;
   out_7457072448927759214[1] = 1;
   out_7457072448927759214[2] = 0;
   out_7457072448927759214[3] = 0;
   out_7457072448927759214[4] = 0;
   out_7457072448927759214[5] = 0;
   out_7457072448927759214[6] = 0;
   out_7457072448927759214[7] = 0;
   out_7457072448927759214[8] = 0;
}
void h_28(double *state, double *unused, double *out_5826402875759803314) {
   out_5826402875759803314[0] = state[0];
}
void H_28(double *state, double *unused, double *out_5493442177362432963) {
   out_5493442177362432963[0] = 1;
   out_5493442177362432963[1] = 0;
   out_5493442177362432963[2] = 0;
   out_5493442177362432963[3] = 0;
   out_5493442177362432963[4] = 0;
   out_5493442177362432963[5] = 0;
   out_5493442177362432963[6] = 0;
   out_5493442177362432963[7] = 0;
   out_5493442177362432963[8] = 0;
}
void h_31(double *state, double *unused, double *out_3191013835359070736) {
   out_3191013835359070736[0] = state[8];
}
void H_31(double *state, double *unused, double *out_3408961501237582772) {
   out_3408961501237582772[0] = 0;
   out_3408961501237582772[1] = 0;
   out_3408961501237582772[2] = 0;
   out_3408961501237582772[3] = 0;
   out_3408961501237582772[4] = 0;
   out_3408961501237582772[5] = 0;
   out_3408961501237582772[6] = 0;
   out_3408961501237582772[7] = 0;
   out_3408961501237582772[8] = 1;
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

void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_25, H_25, NULL, in_z, in_R, in_ea, MAHA_THRESH_25);
}
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<2, 3, 0>(in_x, in_P, h_24, H_24, NULL, in_z, in_R, in_ea, MAHA_THRESH_24);
}
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_30, H_30, NULL, in_z, in_R, in_ea, MAHA_THRESH_30);
}
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_26, H_26, NULL, in_z, in_R, in_ea, MAHA_THRESH_26);
}
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_27, H_27, NULL, in_z, in_R, in_ea, MAHA_THRESH_27);
}
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_29, H_29, NULL, in_z, in_R, in_ea, MAHA_THRESH_29);
}
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_28, H_28, NULL, in_z, in_R, in_ea, MAHA_THRESH_28);
}
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_31, H_31, NULL, in_z, in_R, in_ea, MAHA_THRESH_31);
}
void car_err_fun(double *nom_x, double *delta_x, double *out_2397847969433383157) {
  err_fun(nom_x, delta_x, out_2397847969433383157);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_8621666753450333961) {
  inv_err_fun(nom_x, true_x, out_8621666753450333961);
}
void car_H_mod_fun(double *state, double *out_2589147812390356385) {
  H_mod_fun(state, out_2589147812390356385);
}
void car_f_fun(double *state, double dt, double *out_9011514071817120336) {
  f_fun(state,  dt, out_9011514071817120336);
}
void car_F_fun(double *state, double dt, double *out_5945821314829573806) {
  F_fun(state,  dt, out_5945821314829573806);
}
void car_h_25(double *state, double *unused, double *out_3193307014990671124) {
  h_25(state, unused, out_3193307014990671124);
}
void car_H_25(double *state, double *unused, double *out_3439607463114543200) {
  H_25(state, unused, out_3439607463114543200);
}
void car_h_24(double *state, double *unused, double *out_1870912835991722903) {
  h_24(state, unused, out_1870912835991722903);
}
void car_H_24(double *state, double *unused, double *out_7564462017606292096) {
  H_24(state, unused, out_7564462017606292096);
}
void car_h_30(double *state, double *unused, double *out_6413933887999061586) {
  h_30(state, unused, out_6413933887999061586);
}
void car_H_30(double *state, double *unused, double *out_3568946410257783270) {
  H_30(state, unused, out_3568946410257783270);
}
void car_h_26(double *state, double *unused, double *out_8794105260633279935) {
  h_26(state, unused, out_8794105260633279935);
}
void car_H_26(double *state, double *unused, double *out_7181110781988599424) {
  H_26(state, unused, out_7181110781988599424);
}
void car_h_27(double *state, double *unused, double *out_6742055710391033214) {
  h_27(state, unused, out_6742055710391033214);
}
void car_H_27(double *state, double *unused, double *out_5743709722058208181) {
  H_27(state, unused, out_5743709722058208181);
}
void car_h_29(double *state, double *unused, double *out_6390080314629533195) {
  h_29(state, unused, out_6390080314629533195);
}
void car_H_29(double *state, double *unused, double *out_7457072448927759214) {
  H_29(state, unused, out_7457072448927759214);
}
void car_h_28(double *state, double *unused, double *out_5826402875759803314) {
  h_28(state, unused, out_5826402875759803314);
}
void car_H_28(double *state, double *unused, double *out_5493442177362432963) {
  H_28(state, unused, out_5493442177362432963);
}
void car_h_31(double *state, double *unused, double *out_3191013835359070736) {
  h_31(state, unused, out_3191013835359070736);
}
void car_H_31(double *state, double *unused, double *out_3408961501237582772) {
  H_31(state, unused, out_3408961501237582772);
}
void car_predict(double *in_x, double *in_P, double *in_Q, double dt) {
  predict(in_x, in_P, in_Q, dt);
}
void car_set_mass(double x) {
  set_mass(x);
}
void car_set_rotational_inertia(double x) {
  set_rotational_inertia(x);
}
void car_set_center_to_front(double x) {
  set_center_to_front(x);
}
void car_set_center_to_rear(double x) {
  set_center_to_rear(x);
}
void car_set_stiffness_front(double x) {
  set_stiffness_front(x);
}
void car_set_stiffness_rear(double x) {
  set_stiffness_rear(x);
}
}

const EKF car = {
  .name = "car",
  .kinds = { 25, 24, 30, 26, 27, 29, 28, 31 },
  .feature_kinds = {  },
  .f_fun = car_f_fun,
  .F_fun = car_F_fun,
  .err_fun = car_err_fun,
  .inv_err_fun = car_inv_err_fun,
  .H_mod_fun = car_H_mod_fun,
  .predict = car_predict,
  .hs = {
    { 25, car_h_25 },
    { 24, car_h_24 },
    { 30, car_h_30 },
    { 26, car_h_26 },
    { 27, car_h_27 },
    { 29, car_h_29 },
    { 28, car_h_28 },
    { 31, car_h_31 },
  },
  .Hs = {
    { 25, car_H_25 },
    { 24, car_H_24 },
    { 30, car_H_30 },
    { 26, car_H_26 },
    { 27, car_H_27 },
    { 29, car_H_29 },
    { 28, car_H_28 },
    { 31, car_H_31 },
  },
  .updates = {
    { 25, car_update_25 },
    { 24, car_update_24 },
    { 30, car_update_30 },
    { 26, car_update_26 },
    { 27, car_update_27 },
    { 29, car_update_29 },
    { 28, car_update_28 },
    { 31, car_update_31 },
  },
  .Hes = {
  },
  .sets = {
    { "mass", car_set_mass },
    { "rotational_inertia", car_set_rotational_inertia },
    { "center_to_front", car_set_center_to_front },
    { "center_to_rear", car_set_center_to_rear },
    { "stiffness_front", car_set_stiffness_front },
    { "stiffness_rear", car_set_stiffness_rear },
  },
  .extra_routines = {
  },
};

ekf_init(car);
