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
void err_fun(double *nom_x, double *delta_x, double *out_6747836662220731755) {
   out_6747836662220731755[0] = delta_x[0] + nom_x[0];
   out_6747836662220731755[1] = delta_x[1] + nom_x[1];
   out_6747836662220731755[2] = delta_x[2] + nom_x[2];
   out_6747836662220731755[3] = delta_x[3] + nom_x[3];
   out_6747836662220731755[4] = delta_x[4] + nom_x[4];
   out_6747836662220731755[5] = delta_x[5] + nom_x[5];
   out_6747836662220731755[6] = delta_x[6] + nom_x[6];
   out_6747836662220731755[7] = delta_x[7] + nom_x[7];
   out_6747836662220731755[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_8852891871922293725) {
   out_8852891871922293725[0] = -nom_x[0] + true_x[0];
   out_8852891871922293725[1] = -nom_x[1] + true_x[1];
   out_8852891871922293725[2] = -nom_x[2] + true_x[2];
   out_8852891871922293725[3] = -nom_x[3] + true_x[3];
   out_8852891871922293725[4] = -nom_x[4] + true_x[4];
   out_8852891871922293725[5] = -nom_x[5] + true_x[5];
   out_8852891871922293725[6] = -nom_x[6] + true_x[6];
   out_8852891871922293725[7] = -nom_x[7] + true_x[7];
   out_8852891871922293725[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_4144882642189588128) {
   out_4144882642189588128[0] = 1.0;
   out_4144882642189588128[1] = 0;
   out_4144882642189588128[2] = 0;
   out_4144882642189588128[3] = 0;
   out_4144882642189588128[4] = 0;
   out_4144882642189588128[5] = 0;
   out_4144882642189588128[6] = 0;
   out_4144882642189588128[7] = 0;
   out_4144882642189588128[8] = 0;
   out_4144882642189588128[9] = 0;
   out_4144882642189588128[10] = 1.0;
   out_4144882642189588128[11] = 0;
   out_4144882642189588128[12] = 0;
   out_4144882642189588128[13] = 0;
   out_4144882642189588128[14] = 0;
   out_4144882642189588128[15] = 0;
   out_4144882642189588128[16] = 0;
   out_4144882642189588128[17] = 0;
   out_4144882642189588128[18] = 0;
   out_4144882642189588128[19] = 0;
   out_4144882642189588128[20] = 1.0;
   out_4144882642189588128[21] = 0;
   out_4144882642189588128[22] = 0;
   out_4144882642189588128[23] = 0;
   out_4144882642189588128[24] = 0;
   out_4144882642189588128[25] = 0;
   out_4144882642189588128[26] = 0;
   out_4144882642189588128[27] = 0;
   out_4144882642189588128[28] = 0;
   out_4144882642189588128[29] = 0;
   out_4144882642189588128[30] = 1.0;
   out_4144882642189588128[31] = 0;
   out_4144882642189588128[32] = 0;
   out_4144882642189588128[33] = 0;
   out_4144882642189588128[34] = 0;
   out_4144882642189588128[35] = 0;
   out_4144882642189588128[36] = 0;
   out_4144882642189588128[37] = 0;
   out_4144882642189588128[38] = 0;
   out_4144882642189588128[39] = 0;
   out_4144882642189588128[40] = 1.0;
   out_4144882642189588128[41] = 0;
   out_4144882642189588128[42] = 0;
   out_4144882642189588128[43] = 0;
   out_4144882642189588128[44] = 0;
   out_4144882642189588128[45] = 0;
   out_4144882642189588128[46] = 0;
   out_4144882642189588128[47] = 0;
   out_4144882642189588128[48] = 0;
   out_4144882642189588128[49] = 0;
   out_4144882642189588128[50] = 1.0;
   out_4144882642189588128[51] = 0;
   out_4144882642189588128[52] = 0;
   out_4144882642189588128[53] = 0;
   out_4144882642189588128[54] = 0;
   out_4144882642189588128[55] = 0;
   out_4144882642189588128[56] = 0;
   out_4144882642189588128[57] = 0;
   out_4144882642189588128[58] = 0;
   out_4144882642189588128[59] = 0;
   out_4144882642189588128[60] = 1.0;
   out_4144882642189588128[61] = 0;
   out_4144882642189588128[62] = 0;
   out_4144882642189588128[63] = 0;
   out_4144882642189588128[64] = 0;
   out_4144882642189588128[65] = 0;
   out_4144882642189588128[66] = 0;
   out_4144882642189588128[67] = 0;
   out_4144882642189588128[68] = 0;
   out_4144882642189588128[69] = 0;
   out_4144882642189588128[70] = 1.0;
   out_4144882642189588128[71] = 0;
   out_4144882642189588128[72] = 0;
   out_4144882642189588128[73] = 0;
   out_4144882642189588128[74] = 0;
   out_4144882642189588128[75] = 0;
   out_4144882642189588128[76] = 0;
   out_4144882642189588128[77] = 0;
   out_4144882642189588128[78] = 0;
   out_4144882642189588128[79] = 0;
   out_4144882642189588128[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_3388687291422709070) {
   out_3388687291422709070[0] = state[0];
   out_3388687291422709070[1] = state[1];
   out_3388687291422709070[2] = state[2];
   out_3388687291422709070[3] = state[3];
   out_3388687291422709070[4] = state[4];
   out_3388687291422709070[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_3388687291422709070[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_3388687291422709070[7] = state[7];
   out_3388687291422709070[8] = state[8];
}
void F_fun(double *state, double dt, double *out_6630854580796602574) {
   out_6630854580796602574[0] = 1;
   out_6630854580796602574[1] = 0;
   out_6630854580796602574[2] = 0;
   out_6630854580796602574[3] = 0;
   out_6630854580796602574[4] = 0;
   out_6630854580796602574[5] = 0;
   out_6630854580796602574[6] = 0;
   out_6630854580796602574[7] = 0;
   out_6630854580796602574[8] = 0;
   out_6630854580796602574[9] = 0;
   out_6630854580796602574[10] = 1;
   out_6630854580796602574[11] = 0;
   out_6630854580796602574[12] = 0;
   out_6630854580796602574[13] = 0;
   out_6630854580796602574[14] = 0;
   out_6630854580796602574[15] = 0;
   out_6630854580796602574[16] = 0;
   out_6630854580796602574[17] = 0;
   out_6630854580796602574[18] = 0;
   out_6630854580796602574[19] = 0;
   out_6630854580796602574[20] = 1;
   out_6630854580796602574[21] = 0;
   out_6630854580796602574[22] = 0;
   out_6630854580796602574[23] = 0;
   out_6630854580796602574[24] = 0;
   out_6630854580796602574[25] = 0;
   out_6630854580796602574[26] = 0;
   out_6630854580796602574[27] = 0;
   out_6630854580796602574[28] = 0;
   out_6630854580796602574[29] = 0;
   out_6630854580796602574[30] = 1;
   out_6630854580796602574[31] = 0;
   out_6630854580796602574[32] = 0;
   out_6630854580796602574[33] = 0;
   out_6630854580796602574[34] = 0;
   out_6630854580796602574[35] = 0;
   out_6630854580796602574[36] = 0;
   out_6630854580796602574[37] = 0;
   out_6630854580796602574[38] = 0;
   out_6630854580796602574[39] = 0;
   out_6630854580796602574[40] = 1;
   out_6630854580796602574[41] = 0;
   out_6630854580796602574[42] = 0;
   out_6630854580796602574[43] = 0;
   out_6630854580796602574[44] = 0;
   out_6630854580796602574[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_6630854580796602574[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_6630854580796602574[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_6630854580796602574[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_6630854580796602574[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_6630854580796602574[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_6630854580796602574[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_6630854580796602574[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_6630854580796602574[53] = -9.8000000000000007*dt;
   out_6630854580796602574[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_6630854580796602574[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_6630854580796602574[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_6630854580796602574[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_6630854580796602574[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_6630854580796602574[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_6630854580796602574[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_6630854580796602574[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_6630854580796602574[62] = 0;
   out_6630854580796602574[63] = 0;
   out_6630854580796602574[64] = 0;
   out_6630854580796602574[65] = 0;
   out_6630854580796602574[66] = 0;
   out_6630854580796602574[67] = 0;
   out_6630854580796602574[68] = 0;
   out_6630854580796602574[69] = 0;
   out_6630854580796602574[70] = 1;
   out_6630854580796602574[71] = 0;
   out_6630854580796602574[72] = 0;
   out_6630854580796602574[73] = 0;
   out_6630854580796602574[74] = 0;
   out_6630854580796602574[75] = 0;
   out_6630854580796602574[76] = 0;
   out_6630854580796602574[77] = 0;
   out_6630854580796602574[78] = 0;
   out_6630854580796602574[79] = 0;
   out_6630854580796602574[80] = 1;
}
void h_25(double *state, double *unused, double *out_108763426625900206) {
   out_108763426625900206[0] = state[6];
}
void H_25(double *state, double *unused, double *out_1623245227139982541) {
   out_1623245227139982541[0] = 0;
   out_1623245227139982541[1] = 0;
   out_1623245227139982541[2] = 0;
   out_1623245227139982541[3] = 0;
   out_1623245227139982541[4] = 0;
   out_1623245227139982541[5] = 0;
   out_1623245227139982541[6] = 1;
   out_1623245227139982541[7] = 0;
   out_1623245227139982541[8] = 0;
}
void h_24(double *state, double *unused, double *out_3426007656625160364) {
   out_3426007656625160364[0] = state[4];
   out_3426007656625160364[1] = state[5];
}
void H_24(double *state, double *unused, double *out_3245569637887724311) {
   out_3245569637887724311[0] = 0;
   out_3245569637887724311[1] = 0;
   out_3245569637887724311[2] = 0;
   out_3245569637887724311[3] = 0;
   out_3245569637887724311[4] = 1;
   out_3245569637887724311[5] = 0;
   out_3245569637887724311[6] = 0;
   out_3245569637887724311[7] = 0;
   out_3245569637887724311[8] = 0;
   out_3245569637887724311[9] = 0;
   out_3245569637887724311[10] = 0;
   out_3245569637887724311[11] = 0;
   out_3245569637887724311[12] = 0;
   out_3245569637887724311[13] = 0;
   out_3245569637887724311[14] = 1;
   out_3245569637887724311[15] = 0;
   out_3245569637887724311[16] = 0;
   out_3245569637887724311[17] = 0;
}
void h_30(double *state, double *unused, double *out_3752920049802601446) {
   out_3752920049802601446[0] = state[4];
}
void H_30(double *state, double *unused, double *out_2904451102987625657) {
   out_2904451102987625657[0] = 0;
   out_2904451102987625657[1] = 0;
   out_2904451102987625657[2] = 0;
   out_2904451102987625657[3] = 0;
   out_2904451102987625657[4] = 1;
   out_2904451102987625657[5] = 0;
   out_2904451102987625657[6] = 0;
   out_2904451102987625657[7] = 0;
   out_2904451102987625657[8] = 0;
}
void h_26(double *state, double *unused, double *out_5443367884538282406) {
   out_5443367884538282406[0] = state[7];
}
void H_26(double *state, double *unused, double *out_2118258091734073683) {
   out_2118258091734073683[0] = 0;
   out_2118258091734073683[1] = 0;
   out_2118258091734073683[2] = 0;
   out_2118258091734073683[3] = 0;
   out_2118258091734073683[4] = 0;
   out_2118258091734073683[5] = 0;
   out_2118258091734073683[6] = 0;
   out_2118258091734073683[7] = 1;
   out_2118258091734073683[8] = 0;
}
void h_27(double *state, double *unused, double *out_3419155337861062887) {
   out_3419155337861062887[0] = state[3];
}
void H_27(double *state, double *unused, double *out_680857031803682440) {
   out_680857031803682440[0] = 0;
   out_680857031803682440[1] = 0;
   out_680857031803682440[2] = 0;
   out_680857031803682440[3] = 1;
   out_680857031803682440[4] = 0;
   out_680857031803682440[5] = 0;
   out_680857031803682440[6] = 0;
   out_680857031803682440[7] = 0;
   out_680857031803682440[8] = 0;
}
void h_29(double *state, double *unused, double *out_5103173916313097488) {
   out_5103173916313097488[0] = state[1];
}
void H_29(double *state, double *unused, double *out_2394219758673233473) {
   out_2394219758673233473[0] = 0;
   out_2394219758673233473[1] = 1;
   out_2394219758673233473[2] = 0;
   out_2394219758673233473[3] = 0;
   out_2394219758673233473[4] = 0;
   out_2394219758673233473[5] = 0;
   out_2394219758673233473[6] = 0;
   out_2394219758673233473[7] = 0;
   out_2394219758673233473[8] = 0;
}
void h_28(double *state, double *unused, double *out_3864825382797416024) {
   out_3864825382797416024[0] = state[0];
}
void H_28(double *state, double *unused, double *out_7476618775742764047) {
   out_7476618775742764047[0] = 1;
   out_7476618775742764047[1] = 0;
   out_7476618775742764047[2] = 0;
   out_7476618775742764047[3] = 0;
   out_7476618775742764047[4] = 0;
   out_7476618775742764047[5] = 0;
   out_7476618775742764047[6] = 0;
   out_7476618775742764047[7] = 0;
   out_7476618775742764047[8] = 0;
}
void h_31(double *state, double *unused, double *out_6329140609472914175) {
   out_6329140609472914175[0] = state[8];
}
void H_31(double *state, double *unused, double *out_2744466193967425159) {
   out_2744466193967425159[0] = 0;
   out_2744466193967425159[1] = 0;
   out_2744466193967425159[2] = 0;
   out_2744466193967425159[3] = 0;
   out_2744466193967425159[4] = 0;
   out_2744466193967425159[5] = 0;
   out_2744466193967425159[6] = 0;
   out_2744466193967425159[7] = 0;
   out_2744466193967425159[8] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_6747836662220731755) {
  err_fun(nom_x, delta_x, out_6747836662220731755);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_8852891871922293725) {
  inv_err_fun(nom_x, true_x, out_8852891871922293725);
}
void car_H_mod_fun(double *state, double *out_4144882642189588128) {
  H_mod_fun(state, out_4144882642189588128);
}
void car_f_fun(double *state, double dt, double *out_3388687291422709070) {
  f_fun(state,  dt, out_3388687291422709070);
}
void car_F_fun(double *state, double dt, double *out_6630854580796602574) {
  F_fun(state,  dt, out_6630854580796602574);
}
void car_h_25(double *state, double *unused, double *out_108763426625900206) {
  h_25(state, unused, out_108763426625900206);
}
void car_H_25(double *state, double *unused, double *out_1623245227139982541) {
  H_25(state, unused, out_1623245227139982541);
}
void car_h_24(double *state, double *unused, double *out_3426007656625160364) {
  h_24(state, unused, out_3426007656625160364);
}
void car_H_24(double *state, double *unused, double *out_3245569637887724311) {
  H_24(state, unused, out_3245569637887724311);
}
void car_h_30(double *state, double *unused, double *out_3752920049802601446) {
  h_30(state, unused, out_3752920049802601446);
}
void car_H_30(double *state, double *unused, double *out_2904451102987625657) {
  H_30(state, unused, out_2904451102987625657);
}
void car_h_26(double *state, double *unused, double *out_5443367884538282406) {
  h_26(state, unused, out_5443367884538282406);
}
void car_H_26(double *state, double *unused, double *out_2118258091734073683) {
  H_26(state, unused, out_2118258091734073683);
}
void car_h_27(double *state, double *unused, double *out_3419155337861062887) {
  h_27(state, unused, out_3419155337861062887);
}
void car_H_27(double *state, double *unused, double *out_680857031803682440) {
  H_27(state, unused, out_680857031803682440);
}
void car_h_29(double *state, double *unused, double *out_5103173916313097488) {
  h_29(state, unused, out_5103173916313097488);
}
void car_H_29(double *state, double *unused, double *out_2394219758673233473) {
  H_29(state, unused, out_2394219758673233473);
}
void car_h_28(double *state, double *unused, double *out_3864825382797416024) {
  h_28(state, unused, out_3864825382797416024);
}
void car_H_28(double *state, double *unused, double *out_7476618775742764047) {
  H_28(state, unused, out_7476618775742764047);
}
void car_h_31(double *state, double *unused, double *out_6329140609472914175) {
  h_31(state, unused, out_6329140609472914175);
}
void car_H_31(double *state, double *unused, double *out_2744466193967425159) {
  H_31(state, unused, out_2744466193967425159);
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
