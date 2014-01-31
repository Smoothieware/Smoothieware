#ifndef _VECTOR3_H
#define _VECTOR3_H

class Vector3
{
public:
    Vector3();
    Vector3(float, float, float);

    float&   operator[](int);

    Vector3* cross(Vector3&, Vector3* out);

    float    dot(Vector3&);

    float    magsq();
    float    mag();

    Vector3* add(Vector3&, Vector3*);
    Vector3* sub(Vector3&, Vector3*);

    Vector3* mul(float, Vector3*);

    Vector3* unit(Vector3*);

private:
    float  elem[3];
    static float nan;
};

// typedef float Vector3[3];

// float* cross_product(Vector3 vec1, Vector3 vec2, Vector3 out);
// float dot_product(Vector3 vec1, Vector3 vec2);
// float magsq(Vector3 vec);
// float* vecsub(Vector3 vec1, Vector3 vec2, Vector3 out)
// float* scalar_mul(Vector3 vec, float scalar, Vector3 out)

#endif /* _VECTOR3_H */
