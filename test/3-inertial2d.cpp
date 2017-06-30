#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif

#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>

#include <boost/utility.hpp>
#include <boost/algorithm/string.hpp>

#include "quat.h"

#define D_TO_R M_PI / 180.
#define R_TO_D 180. / M_PI

using namespace std;
using namespace boost::algorithm;

void stampa_q(Quaternione &q) {
  cout << endl;
  for (int i = 0; i < 4; i++) {
    cout << i << "   " << q[i] << endl;
  }
  cout << endl;
};

bool Belongs_to_string(char c, string s) {
  for (size_t i = 0; i < s.size(); i++) {
    if (c == s.at(i)) return true;
  }
  return false;
};

vector< vector<string> > Read_from_file(string file_name, char compress = 'c') {
  string line; vector<string> tokens; vector< vector<string> > file_tokens;
  fstream data_file; data_file.open(file_name.c_str());
  if (!data_file) {
    cout << "FAILED: file " << file_name << " could not be opened" << endl;
    cin.get();
    exit(777);
  }
  else { cout << "SUCCESS: file " << file_name << " opened!\n"; }
  while (!data_file.eof()) {
    line.clear();
    tokens.clear();
    getline(data_file, line);
    trim(line); // remove leading/trailing spaces
    if (line.size() > 0) {
      if (compress == 'c') split(tokens, line, is_any_of("\t ,;:"), token_compress_on);
      else split(tokens, line, is_any_of("\t ,;:"));
      if (Belongs_to_string(tokens[0].at(0), "#!")) continue;
      else file_tokens.push_back(tokens);
    }
  }
  return file_tokens;
};

vector<double> Integrate(vector<double> &x, vector<double> &f, double F0 = 0) {
  assert(x.size() == f.size());
  vector<double> F;
  F.push_back(F0);
  double dx, df, sum = F[0];
  for (size_t i = 1; i < x.size(); i++) {
    dx = x[i] - x[i - 1];
    df = .5*(f[i] + f[i - 1]);
    sum += dx*df;
    F.push_back(sum);
  }
  assert(F.size() == x.size());
  return F;
};

int main() {
  cout << "************* QUATERNIONI INERZIALI 2D *************\n";

  vector< vector<string> > file_tokens;
  vector<double> t;
  vector<double> omega_x, omega_y, omega_z, omega_norm;
  vector<double> aloc_x, aloc_y, aloc_z;

  file_tokens = Read_from_file("data2d/omega.dat");               // Importing LOCAL angular velocities
  for (size_t i = 0; i < file_tokens.size(); i++) {
    if (file_tokens[i].size() > 3) {
      t.push_back(atof(file_tokens[i][0].c_str()));
      omega_x.push_back(atof(file_tokens[i][1].c_str()));
      omega_y.push_back(atof(file_tokens[i][2].c_str()));
      omega_z.push_back(atof(file_tokens[i][3].c_str()));
    }
  }
  file_tokens.clear();

  transform(omega_x.begin(), omega_x.end(), omega_x.begin(), bind1st(multiplies<double>(), D_TO_R)); // omega's in radians
  transform(omega_y.begin(), omega_y.end(), omega_y.begin(), bind1st(multiplies<double>(), D_TO_R));
  transform(omega_z.begin(), omega_z.end(), omega_z.begin(), bind1st(multiplies<double>(), D_TO_R));

  file_tokens = Read_from_file("data2d/accni.dat");               // Importing LOCAL accelerations
  for (size_t i = 0; i < file_tokens.size(); i++) {
    if (file_tokens[i].size() > 3) {
      aloc_x.push_back(atof(file_tokens[i][1].c_str()));
      aloc_y.push_back(atof(file_tokens[i][2].c_str()));
      aloc_z.push_back(atof(file_tokens[i][3].c_str()));
    }
  }
  file_tokens.clear();

  vector<double> a_th_x, a_th_y, a_th_z;
  file_tokens = Read_from_file("data2d/acci.dat");                // Importing INERTIAL accelerations
  for (size_t i = 0; i < file_tokens.size(); i++) {
    if (file_tokens[i].size() > 3) {
      a_th_x.push_back(atof(file_tokens[i][1].c_str()));
      a_th_y.push_back(atof(file_tokens[i][2].c_str()));
      a_th_z.push_back(atof(file_tokens[i][3].c_str()));
    }
  }
  file_tokens.clear();

  vector<double> v_th_x, v_th_y, v_th_z;
  file_tokens = Read_from_file("data2d/veli.dat");                // Importing INERTIAL velocities
  for (size_t i = 0; i < file_tokens.size(); i++) {
    if (file_tokens[i].size() > 3) {
      v_th_x.push_back(atof(file_tokens[i][1].c_str()));
      v_th_y.push_back(atof(file_tokens[i][2].c_str()));
      v_th_z.push_back(atof(file_tokens[i][3].c_str()));
    }
  }
  file_tokens.clear();

  vector<double> r_th_x, r_th_y, r_th_z;
  file_tokens = Read_from_file("data2d/r.dat");                   // Importing INERTIAL positions
  for (size_t i = 0; i < file_tokens.size(); i++) {
    if (file_tokens[i].size() > 3) {
      r_th_x.push_back(atof(file_tokens[i][1].c_str()));
      r_th_y.push_back(atof(file_tokens[i][2].c_str()));
      r_th_z.push_back(atof(file_tokens[i][3].c_str()));
    }
  }
  file_tokens.clear();

  Quaternione dR;
  vector<Quaternione> R, omega_Q;
  for (size_t i = 0; i < t.size(); i++) {                        // Accumulating rotation quaternion loop
    omega_Q.push_back(Quaternione({ 0, omega_x[i], omega_y[i], omega_z[i] }));
    if (i == 0) {
      dR = Quaternione(omega_Q[i], (t[i + 1] - t[i]));
      R.push_back(dR);
    }
    else {
      dR = Quaternione(omega_Q[i], (t[i] - t[i - 1]));
      R.push_back(dR*R[i - 1]);
    }
  }

  cout << "1 - rotation quaternion " << R.size() << endl;

  vector<Quaternione> aloc_Q;
  for (size_t i = 0; i < t.size(); i++) {                        // Acceleration quaternion
    aloc_Q.push_back(Quaternione({ 0, aloc_x[i], aloc_y[i], aloc_z[i] }));
  }

  cout << "2 - local acceleration quaternion " << aloc_Q.size() << endl;

  vector<Quaternione> acc_Q;
  vector<double> a_x, a_y, a_z;
  for (size_t i = 0; i < t.size(); i++) {                        // Quaternionic rotation
    //    acc_Q.push_back( (~R[i]) * aloc_Q[i] * R[i] );
    acc_Q.push_back(R[i] * aloc_Q[i] * (~R[i]));
    a_x.push_back(acc_Q[i][1]);
    a_y.push_back(acc_Q[i][2]);
    a_z.push_back(acc_Q[i][3]);
  }

  vector<double> v_x, v_y, v_z, r_x, r_y, r_z;                  // Standard kinematic
  double v0_x = -2.0124999869791709, v0_y = 5.9999843750081379, v0_z = 0.0,
    r0_x = 10.999968750016276, r0_y = 2.0249999739583413, r0_z = 0.0;

  v_x = Integrate(t, a_x, v0_x);
  v_y = Integrate(t, a_y, v0_y);
  v_z = Integrate(t, a_z, v0_z);

  r_x = Integrate(t, v_x, r0_x);
  r_y = Integrate(t, v_y, r0_y);
  r_z = Integrate(t, v_z, r0_z);

  // ****************************************  CALCULATING ERRORS ***************************************
  vector<double> da_x, da_y, da_z, dv_x, dv_y, dv_z, dr_x, dr_y, dr_z;
  for (size_t i = 0; i < t.size(); i++) {
    da_x.push_back(fabs(a_x[i] - a_th_x[i]));
    da_y.push_back(fabs(a_y[i] - a_th_y[i]));
    da_z.push_back(fabs(a_z[i] - a_th_z[i]));
    dv_x.push_back(fabs(v_x[i] - v_th_x[i]));
    dv_y.push_back(fabs(v_y[i] - v_th_y[i]));
    dv_z.push_back(fabs(v_z[i] - v_th_z[i]));
    dr_x.push_back(fabs(r_x[i] - r_th_x[i]));
    dr_y.push_back(fabs(r_y[i] - r_th_y[i]));
    dr_z.push_back(fabs(r_z[i] - r_th_z[i]));
  }


  // ****************************************  SAVING TO FILE ******************************************* 
  char all_data_name[30]; sprintf(all_data_name, "data2d/all_data.txt");
  FILE * out_data = fopen(all_data_name, "w");
  fprintf(out_data, "%10s %10s %10s %10s %10s %10s %10s %10s %10s %10s %10s\n", "1 - Index", "2 -  t",
    "3 -  a_x", "4 -  a_y", "5 -  a_z", "6 -  v_x", "7 -  v_y", "8 -  v_z", "9 -  r_x", "10 -  r_y", "11 -  r_z");
  for (size_t i = 0; i < t.size(); i++) {
#ifdef _MSC_VER
    fprintf(out_data, "%10Iu %10.6f %10.6f %10.6f %10.6f %10.6f %10.6f %10.6f %10.6f %10.6f %10.6f\n", i, t[i], a_x[i], a_y[i], a_z[i], v_x[i], v_y[i], v_z[i], r_x[i], r_y[i], r_z[i]);
#else
    fprintf(out_data, "%10zu %10.6f %10.6f %10.6f %10.6f %10.6f %10.6f %10.6f %10.6f %10.6f %10.6f\n", i, t[i], a_x[i], a_y[i], a_z[i], v_x[i], v_y[i], v_z[i], r_x[i], r_y[i], r_z[i]);
#endif
  }
  fclose(out_data);

  char all_err_name[30]; sprintf(all_err_name, "data2d/all_errors.txt");
  out_data = fopen(all_err_name, "w");
  fprintf(out_data, "%10s %10s %10s %10s %10s %10s %10s %10s %10s %10s %10s\n", "1 - Index", "2 -  t",
    "3 -  da_x", "4 -  da_y", "5 -  da_z", "6 -  dv_x", "7 -  dv_y", "8 -  dv_z", "9 -  dr_x", "10 -  dr_y", "11 -  dr_z");
  for (size_t i = 0; i < t.size(); i++) {
#ifdef _MSC_VER
    fprintf(out_data, "%10Iu %10.6f %10.6f %10.6f %10.6f %10.6f %10.6f %10.6f %10.6f %10.6f %10.6f\n", i, t[i], da_x[i], da_y[i], da_z[i], dv_x[i], dv_y[i], dv_z[i], dr_x[i], dr_y[i], dr_z[i]);
#else
    fprintf(out_data, "%10zu %10.6f %10.6f %10.6f %10.6f %10.6f %10.6f %10.6f %10.6f %10.6f %10.6f\n", i, t[i], da_x[i], da_y[i], da_z[i], dv_x[i], dv_y[i], dv_z[i], dr_x[i], dr_y[i], dr_z[i]);
#endif
  }
  fclose(out_data);


  cout << endl << "Hit ENTER to close..." << endl;
  cin.get();
  return 0;
}
