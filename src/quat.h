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
*/

# ifndef __QUATERNIONI_H__
# define __QUATERNIONI_H__
#include <fstream>
#include <initializer_list>

using namespace std;

class Quaternione {
  double comp[4];
  void copiatore(const Quaternione&);
  void spostatore(Quaternione &&);
  friend istream& operator >> (istream &, Quaternione &);
  friend ostream& operator << (ostream &, Quaternione);
  friend Quaternione operator * (double, Quaternione);
  friend Quaternione operator * (int, Quaternione);
public:
  double& operator[](int);
  Quaternione& operator = (const Quaternione&);
  Quaternione& operator = (Quaternione &&);
  Quaternione();
  ~Quaternione();
  Quaternione(const Quaternione&);
  Quaternione(Quaternione &&);
  Quaternione(const Quaternione&, double);
  Quaternione(initializer_list<double>);
  Quaternione(double(&)[3]);
  Quaternione(double(&)[4]);
  Quaternione operator * (Quaternione);
  Quaternione operator * (Quaternione *);
  Quaternione operator ~ ();
  double operator ! () const;
  bool operator==(Quaternione);
  bool operator!=(Quaternione);
  Quaternione operator + (Quaternione);
  Quaternione operator - (Quaternione);
  Quaternione operator += (Quaternione);
  Quaternione operator -= (Quaternione);
  Quaternione operator - ();
  Quaternione operator * (double);
  Quaternione operator * (int);
  Quaternione operator *= (double);
  Quaternione operator / (double);
  Quaternione operator /= (double);
  Quaternione operator / (Quaternione);
};

# endif
