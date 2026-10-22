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

    // Like getNormal(), but guaranteed to return a normal pointing in the direction of positive Z, i.e. upwards
    Vector3 getUpwardsNormal() const;

    /**
     * Checks for intersection of a ray with the plane.
     * @param rayOrigin Starting point of the ray
     * @param rayDirection Direction vector in which to cast the ray
     * @return The distance along the ray at which the intersection was found, or NAN if the ray does not intersect the plane.
     */
    float findRayIntersection(Vector3 rayOrigin, Vector3 rayDirection);
};

#endif
