#ifndef __PLANE3D_H
#define __PLANE3D_H

#include "Vector3.h"

// define a plane given three points
class Plane3D
{
private:
    Vector3 normal;
    float d;

public:
    Plane3D(const Vector3 &v1, const Vector3 &v2, const Vector3 &v3);
    float getz(float x, float y);
    Vector3 getNormal() const;
    // TODO to save plane
    // string encode() const;
    // void decode(const char *);
};

#endif
