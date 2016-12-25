#include <iostream>
#include <sstream>
#include <cmath>
#include <iomanip>
#include <cstdlib>
#include <cstdio>
#include <cstring>
#include <vector>
#include <utility>

#include "quaternioni.h"

// la prima classe a essere definita (completata) è la class Quaternione
void Quaternione::copiatore(const Quaternione& q) {
  for (int i = 0; i != 4; ++i) comp[i] = q.comp[i];
}

void Quaternione::spostatore(Quaternione && q) {
  for (int i = 0; i != 4; ++i) comp[i] = move(q[i]);
}

istream& operator >> (istream & i, Quaternione & q) {
  for (auto& inp : q.comp) i >> inp; return i;
}

ostream& operator << (ostream &o, Quaternione q) {
  for (auto out : q.comp) o << out << ' '; return o << '\n';
}

Quaternione operator * (double c, Quaternione q) { return q*c; }
Quaternione operator * (int c, Quaternione q) { return q*c; }

double& Quaternione::operator[](int i) { return comp[i]; }

Quaternione& Quaternione::operator = (const Quaternione& q) {
  copiatore(q); return *this;
}

Quaternione& Quaternione::operator = (Quaternione&& q) {
  spostatore(move(q)); return *this;
}

Quaternione::Quaternione() { for (auto& comp_ : comp) comp_ = 0.0; }

Quaternione::Quaternione(const Quaternione& q) { copiatore(q); }

Quaternione::Quaternione(Quaternione&& q) { spostatore(move(q)); }

Quaternione::Quaternione(initializer_list<double> il) {
  int index = 0; for (auto i : il) comp[index++] = i;
}

Quaternione::Quaternione(double(&x)[3]) {
  int index = 1; comp[0] = 0.0; for (auto i : x) comp[index++] = i;
}

Quaternione::Quaternione(double(&x)[4]) {
  int index = 0; for (auto i : x) comp[index++] = i;
}

Quaternione::~Quaternione() {}

Quaternione::Quaternione(const Quaternione& v, double delta) try {
  if (fabs(v.comp[0]) > 1.e-20) throw "pirla: non hai un quaternione immaginario puro\n";
  double norma = !v;
  double s = sin(0.5*norma*delta), c = cos(0.5*norma*delta);
  for (int i = 1; i != 4; ++i) comp[i] = v.comp[i] * s / norma;
  comp[0] = c;
}
catch (const char *s) {
  cerr << s, exit(251);
}

Quaternione Quaternione::operator * (Quaternione s) {
  Quaternione r;
  r[0] = comp[0] * s[0] - comp[1] * s[1] - comp[2] * s[2] - comp[3] * s[3];
  r[1] = comp[1] * s[0] + comp[0] * s[1] + comp[2] * s[3] - comp[3] * s[2];
  r[2] = comp[2] * s[0] + comp[0] * s[2] + comp[3] * s[1] - comp[1] * s[3];
  r[3] = comp[3] * s[0] + comp[0] * s[3] + comp[1] * s[2] - comp[2] * s[1];
  return r;
}

Quaternione Quaternione::operator * (Quaternione * s) try {
  if (fabs((*s)[0]) > 1.e-10) throw "pirla! Non stai rotando un vettore\n";
  if (fabs(!*this - 1.0) > 1.e-10) throw "pirla! Stai rotando con un quaternione non unitario\n";
  return *this * *s * ~*this;
}
catch (const char * st) {
  cerr << st; exit(253);
}

Quaternione Quaternione::operator ~ () {
  Quaternione r;
  r[0] = comp[0]; for (int i = 1; i != 4; ++i) r[i] = -comp[i]; return r;
}

double Quaternione::operator ! () const {
  double r = 0.0; for (auto i : comp) r += i*i; return sqrt(r);
}

Quaternione Quaternione::operator + (Quaternione q) {
  Quaternione r;
  for (int i = 0; i != 4; ++i) r[i] = comp[i] + q[i];
  return r;
}

Quaternione Quaternione::operator - (Quaternione q) {
  Quaternione r;
  for (int i = 0; i != 4; ++i) r[i] = comp[i] - q[i];
  return r;
}

Quaternione Quaternione::operator += (Quaternione q) {
  for (int i = 0; i != 4; ++i) comp[i] += q[i]; return *this;
}

Quaternione Quaternione::operator -= (Quaternione q) {
  for (int i = 0; i != 4; ++i) comp[i] -= q[i]; return *this;
}

Quaternione Quaternione::operator - () {
  Quaternione r;
  for (int i = 0; i != 4; ++i) r[i] = -comp[i]; return r;
}

Quaternione Quaternione::operator * (double c) {                     // from line 20
  Quaternione r;
  for (int i = 0; i != 4; ++i) r[i] = c * comp[i]; return r;
}

Quaternione Quaternione::operator * (int c) {                        // from line 21
  Quaternione r;
  for (int i = 0; i != 4; ++i) r[i] = c * comp[i]; return r;
}


Quaternione Quaternione::operator *= (double c) {
  for (int i = 0; i != 4; ++i) comp[i] *= c; return *this;
}

Quaternione Quaternione::operator / (double c) try {
  if (fabs(c) < 1.e-30) throw "pirla! Stai dividendo per zero\n";
  Quaternione r;
  for (int i = 0; i != 4; ++i) r[i] = comp[i] / c; return r;
}
catch (const char * s) {
  cerr << s; exit(255);
}

Quaternione Quaternione::operator /= (double c) try {
  if (fabs(c) < 1.e-30) throw "pirla! Stai dividendo per zero\n";
  for (int i = 0; i != 4; ++i) comp[i] /= c; return *this;
}
catch (const char * s) {
  cerr << s; exit(255);
}

Quaternione Quaternione::operator / (Quaternione q) try {
  Quaternione r;
  for (int i = 1; i != 4; ++i) {
    if (fabs(q[i]) < 1.e-30) throw "pirla! Stai dividendo per zero";
    r[i] = comp[i] / q[i];
  }
  r[0] = 0.0;
  return r;
}
catch (const char * s) {
  cerr << s; exit(254);
}

bool Quaternione::operator == (Quaternione s) {
  return !(*this - s) < 1.e-13;
}

bool Quaternione::operator != (Quaternione s) {
  return !(*this - s) >= 1.e-13;
}