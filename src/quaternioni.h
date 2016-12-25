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
