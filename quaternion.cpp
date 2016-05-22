#include "quaternion.hpp"

geom::Quaternion::Quaternion(): x(0.0), y(0.0), z(0.0), w(1.0) {}

geom::Quaternion::Quaternion(double x, double y, double z, double w): x(x), y(y), z(z), w(w) {}

double geom::Quaternion::operator [](unsigned int index) const {
  switch(index) {
    case 0: return x;
    case 1: return y;
    case 2: return z;
    default: return w;
  }
}

void geom::Quaternion::operator =(const Quaternion & q) {
  x = q.x;
  y = q.y;
  z = q.z;
  w = q.w;
}

void geom::Quaternion::operator *= (const Quaternion & q) {
	double x, y, z, w;
	x = this->x;
	y = this->y;
	z = this->z;
	w = this->w;

	this->x = w * q.x + x * q.w + y * q.z - z * q.y;
	this->y = w * q.y - x * q.z + y * q.w + z * q.x;
	this->z = w * q.z + x * q.y - y * q.x + z * q.w;
	this->w = w * q.w - x * q.x - y * q.y - z * q.z;
}

bool geom::Quaternion::operator ==(const Quaternion & q) const {
  return x == q.x &&
         y == q.y &&
         z == q.z &&
         w == q.w;
}

bool geom::Quaternion::operator !=(const Quaternion & q) const {
  return !operator==(q);
}

geom::Quaternion geom::Quaternion::operator * (const Quaternion & q) const {
	Quaternion qn;

	qn.x = w * q.x + x * q.w + y * q.z - z * q.y;
	qn.y = w * q.y - x * q.z + y * q.w + z * q.x;
	qn.z = w * q.z + x * q.y - y * q.x + z * q.w;
	qn.w = w * q.w - x * q.x - y * q.y - z * q.z;

	return qn;
}

void geom::Quaternion::clear() {
  x = 0.0f;
  y = 0.0f;
  z = 0.0f;
  w = 1.0f;
}

void geom::Quaternion::conjugate() {
  x = -x;
  y = -y;
  z = -z;
}

geom::Quaternion geom::Quaternion::get_conjugate() const {
  Quaternion q = (*this);
  q.conjugate();
  return q;
}
