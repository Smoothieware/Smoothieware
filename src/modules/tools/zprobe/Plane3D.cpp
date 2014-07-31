#include "Plane3D.h"

Plane3D::Plane3D(const Vector3 &v1, const Vector3 &v2, const Vector3 &v3)
{
    // get the normal of the plane
    Vector3 ab = v1.sub(v2);
    Vector3 ac = v1.sub(v3);

    Vector3 cp = ab.cross(ac);
    normal = cp.unit();

    // ax+by+cz+d=0
    // solve for d
    Vector3 dv = normal.mul(v1);
    d = -dv[0] - dv[1] - dv[2];
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
