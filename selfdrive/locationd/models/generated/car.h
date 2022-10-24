#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_err_fun(double *nom_x, double *delta_x, double *out_2397847969433383157);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_8621666753450333961);
void car_H_mod_fun(double *state, double *out_2589147812390356385);
void car_f_fun(double *state, double dt, double *out_9011514071817120336);
void car_F_fun(double *state, double dt, double *out_5945821314829573806);
void car_h_25(double *state, double *unused, double *out_3193307014990671124);
void car_H_25(double *state, double *unused, double *out_3439607463114543200);
void car_h_24(double *state, double *unused, double *out_1870912835991722903);
void car_H_24(double *state, double *unused, double *out_7564462017606292096);
void car_h_30(double *state, double *unused, double *out_6413933887999061586);
void car_H_30(double *state, double *unused, double *out_3568946410257783270);
void car_h_26(double *state, double *unused, double *out_8794105260633279935);
void car_H_26(double *state, double *unused, double *out_7181110781988599424);
void car_h_27(double *state, double *unused, double *out_6742055710391033214);
void car_H_27(double *state, double *unused, double *out_5743709722058208181);
void car_h_29(double *state, double *unused, double *out_6390080314629533195);
void car_H_29(double *state, double *unused, double *out_7457072448927759214);
void car_h_28(double *state, double *unused, double *out_5826402875759803314);
void car_H_28(double *state, double *unused, double *out_5493442177362432963);
void car_h_31(double *state, double *unused, double *out_3191013835359070736);
void car_H_31(double *state, double *unused, double *out_3408961501237582772);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}