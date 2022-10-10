#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void gnss_update_6(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_20(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_7(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_21(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_err_fun(double *nom_x, double *delta_x, double *out_7981808544055197912);
void gnss_inv_err_fun(double *nom_x, double *true_x, double *out_6481617288261504021);
void gnss_H_mod_fun(double *state, double *out_3823724573421204829);
void gnss_f_fun(double *state, double dt, double *out_5052117925629841371);
void gnss_F_fun(double *state, double dt, double *out_261666031851937057);
void gnss_h_6(double *state, double *sat_pos, double *out_7527303059176548152);
void gnss_H_6(double *state, double *sat_pos, double *out_455354385166966437);
void gnss_h_20(double *state, double *sat_pos, double *out_4786908733115082343);
void gnss_H_20(double *state, double *sat_pos, double *out_2222481325922094845);
void gnss_h_7(double *state, double *sat_pos_vel, double *out_7736441670111324321);
void gnss_H_7(double *state, double *sat_pos_vel, double *out_6741617175241692902);
void gnss_h_21(double *state, double *sat_pos_vel, double *out_7736441670111324321);
void gnss_H_21(double *state, double *sat_pos_vel, double *out_6741617175241692902);
void gnss_predict(double *in_x, double *in_P, double *in_Q, double dt);
}