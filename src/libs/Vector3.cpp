#include "Vector3.h"

#include <fastmath.h>
#include <cstddef>

float Vector3::nan = NAN;

Vector3::Vector3()
{
    elem[0] = elem[1] = elem[2] = 0.0F;
}

Vector3::Vector3(float a, float b, float c)
{
    elem[0] = a;
    elem[1] = b;
    elem[2] = c;
}

float& Vector3::operator[](int i)
{
    if (i >= 0 && i <= 2)
        return elem[i];
    return nan;
}

Vector3* Vector3::cross(Vector3& vec, Vector3* out)
{
    if (out == NULL)
        out = new Vector3();

    out->elem[0] = elem[1] * vec[2] - elem[2] * vec[1];
    out->elem[1] = elem[2] * vec[0] - elem[0] * vec[2];
    out->elem[2] = elem[0] * vec[1] - elem[1] * vec[0];

    return out;
}

float Vector3::dot(Vector3& vec)
{
    return  elem[0] * vec[0] +
            elem[1] * vec[1] +
            elem[2] * vec[2];
}

float Vector3::magsq()
{
    return  powf(elem[0], 2) +
            powf(elem[1], 2) +
            powf(elem[2], 2);
}

float Vector3::mag()
{
    return sqrtf(magsq());
}

Vector3* Vector3::add(Vector3& vec, Vector3* out)
{
    if (out == NULL)
        out = new Vector3();

    out->elem[0] = elem[0] + vec[0];
    out->elem[1] = elem[1] + vec[1];
    out->elem[2] = elem[2] + vec[2];

    return out;
}

Vector3* Vector3::sub(Vector3& vec, Vector3* out)
{
    if (out == NULL)
        out = new Vector3();

    out->elem[0] = elem[0] - vec[0];
    out->elem[1] = elem[1] - vec[1];
    out->elem[2] = elem[2] - vec[2];

    return out;
}

Vector3* Vector3::mul(float scalar, Vector3* out)
{
    if (out == NULL)
        out = new Vector3();

    out->elem[0] = elem[0] * scalar;
    out->elem[1] = elem[1] * scalar;
    out->elem[2] = elem[2] * scalar;

    return out;
}

Vector3* Vector3::unit(Vector3* out)
{
    if (out == NULL)
        out = new Vector3();

    float denom = mag();

    out->elem[0] = elem[0] / denom;
    out->elem[1] = elem[1] / denom;
    out->elem[2] = elem[2] / denom;

    return out;
}
