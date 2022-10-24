#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void gnss_update_6(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_20(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_7(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_21(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_err_fun(double *nom_x, double *delta_x, double *out_309521215273340661);
void gnss_inv_err_fun(double *nom_x, double *true_x, double *out_6124729898382051530);
void gnss_H_mod_fun(double *state, double *out_3138109506697689559);
void gnss_f_fun(double *state, double dt, double *out_7355617933517656719);
void gnss_F_fun(double *state, double dt, double *out_3794844325727046215);
void gnss_h_6(double *state, double *sat_pos, double *out_8872459805762957153);
void gnss_H_6(double *state, double *sat_pos, double *out_8665813077433040777);
void gnss_h_20(double *state, double *sat_pos, double *out_4040060993076324013);
void gnss_H_20(double *state, double *sat_pos, double *out_8911946568174512483);
void gnss_h_7(double *state, double *sat_pos_vel, double *out_5571405682832922983);
void gnss_H_7(double *state, double *sat_pos_vel, double *out_4115633887760809029);
void gnss_h_21(double *state, double *sat_pos_vel, double *out_5571405682832922983);
void gnss_H_21(double *state, double *sat_pos_vel, double *out_4115633887760809029);
void gnss_predict(double *in_x, double *in_P, double *in_Q, double dt);
}