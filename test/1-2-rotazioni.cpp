/*
libquat
https://github.com/physycom/libquat

Licensed under the MIT License <http://opensource.org/licenses/MIT>.
SPDX-License-Identifier: MIT

Copyright (c) 2016-2019 Alessandro Fabbri, Graziano Servizi, Stefano Sinigardi
Copyright (c) 2019-2022 Alessandro Fabbri, Stefano Sinigardi (stesinigardi@hotmail.com)

Permission is hereby  granted, free of charge, to any  person obtaining a copy
of this software and associated  documentation files (the "Software"), to deal
in the Software  without restriction, including without  limitation the rights
to  use, copy,  modify, merge,  publish, distribute,  sublicense, and/or  sell
copies  of  the Software,  and  to  permit persons  to  whom  the Software  is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE  IS PROVIDED "AS  IS", WITHOUT WARRANTY  OF ANY KIND,  EXPRESS OR
IMPLIED,  INCLUDING BUT  NOT  LIMITED TO  THE  WARRANTIES OF  MERCHANTABILITY,
FITNESS FOR  A PARTICULAR PURPOSE AND  NONINFRINGEMENT. IN NO EVENT  SHALL THE
AUTHORS  OR COPYRIGHT  HOLDERS  BE  LIABLE FOR  ANY  CLAIM,  DAMAGES OR  OTHER
LIABILITY, WHETHER IN AN ACTION OF  CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE  OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/ \

#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>

#include <boost/utility.hpp>
#include <boost/algorithm/string.hpp>

#include "quat.h"

using namespace std;

void stampa_q(Quaternione &q) {
  cout << endl;
  for (int i = 0; i < 4; i++) {
    cout << i << "   " << q[i] << endl;
  }
  cout << endl;
};

int main() {
  cout << "************* 1 - QUATERNIONI *************\n";
  double data1[] = { .1, .2, .3, 0 };
  double data2[] = { 0, 1.1, 1.2, 1.3 };

  Quaternione q1, q2;
  q1 = Quaternione(data1);
  q2 = Quaternione(data2);

  Quaternione q3 = q1*q2;

  for (int i = 0; i < 4; i++) {
    cout << i << "  " << q1[i] << "  " << q2[i] << "  " << q3[i] << endl;
  }

  cout << endl << endl << "************* 2 - QUATERNIONI ROTANTI *************\n";
  double vers_x[] = { 1, 0, 0 },
    vers_y[] = { 0, 1, 0 },
    vers_z[] = { 0, 0, 1 };

  double angolo = M_PI / 2;

  Quaternione q_x(vers_x),
    q_y(vers_y),
    q_z(vers_z),
    vettore({ 0, 10, 0, 0 });

  stampa_q(q_x); stampa_q(q_y); stampa_q(q_z); stampa_q(vettore);

  Quaternione ruotatore_x(q_x, angolo),
    ruotatore_y(q_y, angolo),
    ruotatore_z(q_z, angolo);

  stampa_q(ruotatore_x); stampa_q(ruotatore_y); stampa_q(ruotatore_z);

  Quaternione v_ruotato_y = ruotatore_y*vettore*(~ruotatore_y),
    v_ruotato_z = ruotatore_z*vettore*(~ruotatore_z);

  stampa_q(v_ruotato_y); stampa_q(v_ruotato_z);

  cout << endl << endl << "************* 3 - QUATERNIONI E CROSS PRODUCT *************\n";

  Quaternione q_xvy = .5*(q_x*q_y - q_y*q_x),
    q_yvz = .5*(q_y*q_z - q_z*q_y),
    q_zvx = .5*(q_z*q_x - q_x*q_z);

  cout << "x cross y" << endl; stampa_q(q_xvy);
  cout << "y cross z" << endl; stampa_q(q_yvz);
  cout << "z cross x" << endl; stampa_q(q_zvx);

  cout << "Hit ENTER to close..." << endl;
  cin.get();

  return 0;
}
