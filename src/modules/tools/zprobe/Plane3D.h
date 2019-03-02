#ifndef __PLANE3D_H
#define __PLANE3D_H

#include "Vector3.h"

#include <stdint.h>

// define a plane given three points
class Plane3D
{
private:
    Vector3 normal;
    float d;

public:
    Plane3D(const Vector3 &v1, const Vector3 &v2, const Vector3 &v3);
    Plane3D(uint32_t a, uint32_t b, uint32_t c, uint32_t d);
    float getz(float x, float y);
    Vector3 getNormal() const;
    void encode(uint32_t& a, uint32_t& b, uint32_t& c, uint32_t& d);
};

#endif
