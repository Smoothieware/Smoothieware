#include "Plane3D.h"

#include "math.h"

Plane3D::Plane3D(const Vector3 &v1, const Vector3 &v2, const Vector3 &v3)
{
    // get the normal of the plane
    Vector3 ab = v1.sub(v2);
    Vector3 ac = v1.sub(v3);

    Vector3 cp = ab.cross(ac);
    normal = cp.unit();

    // ax+by+cz+d=0
    // solve for d
    d = -normal.dot(v1);
}

typedef union { float f; uint32_t u; } conv_t;
// ctor used to restore a saved plane
Plane3D::Plane3D(uint32_t a, uint32_t b, uint32_t c, uint32_t d)
{
    conv_t ca, cb, cc, cd;
    ca.u= a; cb.u= b; cc.u= c; cd.u= d;
    this->normal = Vector3(ca.f, cb.f, cc.f);
    this->d= cd.f;
}

void Plane3D::encode(uint32_t& a, uint32_t& b, uint32_t& c, uint32_t& d)
{
    conv_t ca, cb, cc, cd;
    ca.f= this->normal[0];
    cb.f= this->normal[1];
    cc.f= this->normal[2];
    cd.f= this->d;
    a= ca.u; b= cb.u; c= cc.u; d= cd.u;
}

// solve for z given x and y
// z= (-ax - by - d)/c
float Plane3D::getz(float x, float y)
{
    return ((-normal[0] * x) - (normal[1] * y) - d) / normal[2];
}

Vector3 Plane3D::getNormal() const
{
    return normal;
}

Vector3 Plane3D::getUpwardsNormal() const
{
    if (normal.data()[2] >= 0) {
        return normal;
    }
    return normal.mul(-1);
}

float Plane3D::findRayIntersection(Vector3 rayOrigin, Vector3 rayDirection)
{
    // ray equation: P = O + tD (starting at origin O and travelling in direction D)
    // plane equation: (P - P0).N = 0  (passing through P0 and with normal N)
    // => (O + tD - P0).N = 0
    // => t = (N.(P0 - O)) / (N.D)

    float denomenator = this->normal.dot(rayDirection.unit());
    if (denomenator == 0) {
        return NAN; // No intersection.
    }

    Vector3 arbitraryPointOnPlane = Vector3(0, 0, this->getz(0, 0));
    float numerator = this->normal.dot(arbitraryPointOnPlane.sub(rayOrigin));
    float distance = numerator / denomenator;
    return distance;
}



