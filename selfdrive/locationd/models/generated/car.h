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
void car_err_fun(double *nom_x, double *delta_x, double *out_6747836662220731755);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_8852891871922293725);
void car_H_mod_fun(double *state, double *out_4144882642189588128);
void car_f_fun(double *state, double dt, double *out_3388687291422709070);
void car_F_fun(double *state, double dt, double *out_6630854580796602574);
void car_h_25(double *state, double *unused, double *out_108763426625900206);
void car_H_25(double *state, double *unused, double *out_1623245227139982541);
void car_h_24(double *state, double *unused, double *out_3426007656625160364);
void car_H_24(double *state, double *unused, double *out_3245569637887724311);
void car_h_30(double *state, double *unused, double *out_3752920049802601446);
void car_H_30(double *state, double *unused, double *out_2904451102987625657);
void car_h_26(double *state, double *unused, double *out_5443367884538282406);
void car_H_26(double *state, double *unused, double *out_2118258091734073683);
void car_h_27(double *state, double *unused, double *out_3419155337861062887);
void car_H_27(double *state, double *unused, double *out_680857031803682440);
void car_h_29(double *state, double *unused, double *out_5103173916313097488);
void car_H_29(double *state, double *unused, double *out_2394219758673233473);
void car_h_28(double *state, double *unused, double *out_3864825382797416024);
void car_H_28(double *state, double *unused, double *out_7476618775742764047);
void car_h_31(double *state, double *unused, double *out_6329140609472914175);
void car_H_31(double *state, double *unused, double *out_2744466193967425159);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}