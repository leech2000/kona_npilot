#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void live_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_9(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_12(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_35(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_32(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_33(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_H(double *in_vec, double *out_5821450610139912524);
void live_err_fun(double *nom_x, double *delta_x, double *out_8199502743948441299);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_8974206346724261763);
void live_H_mod_fun(double *state, double *out_2259405359203808161);
void live_f_fun(double *state, double dt, double *out_7355999048988548228);
void live_F_fun(double *state, double dt, double *out_2793064135720303537);
void live_h_4(double *state, double *unused, double *out_8591242944923457004);
void live_H_4(double *state, double *unused, double *out_3172652086954443828);
void live_h_9(double *state, double *unused, double *out_1108062706504611441);
void live_H_9(double *state, double *unused, double *out_283790534674364486);
void live_h_10(double *state, double *unused, double *out_842376979376852950);
void live_H_10(double *state, double *unused, double *out_8552939557185869698);
void live_h_12(double *state, double *unused, double *out_7209938386305200071);
void live_H_12(double *state, double *unused, double *out_2551553061906850161);
void live_h_35(double *state, double *unused, double *out_1172154858295733565);
void live_H_35(double *state, double *unused, double *out_194009970418163548);
void live_h_32(double *state, double *unused, double *out_6894738059665748948);
void live_H_32(double *state, double *unused, double *out_118611724469281480);
void live_h_13(double *state, double *unused, double *out_5210003797918516289);
void live_H_13(double *state, double *unused, double *out_1664152559261893404);
void live_h_14(double *state, double *unused, double *out_1108062706504611441);
void live_H_14(double *state, double *unused, double *out_283790534674364486);
void live_h_33(double *state, double *unused, double *out_2862768964100890701);
void live_H_33(double *state, double *unused, double *out_3344566975057021152);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}