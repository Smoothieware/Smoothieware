#ifndef _VECTOR3_H
#define _VECTOR3_H

class Vector3 {
    public:
        Vector3();
        Vector3(float, float, float);

        float    operator[](int) const;

        Vector3  cross(const Vector3&) const;       // cross product
        float    dot(const Vector3&) const;         // dot product
        float    magsq() const;                     // squared vector length
        float    mag() const;                       // vector length
        Vector3  add(const Vector3&) const;         // addition
        Vector3  sub(const Vector3&) const;         // subtraction
        Vector3  mul(float) const;                  // scalar multiplication
        Vector3  mul(const Vector3& v) const;       // component wise multiplication
        Vector3  normalized(void) const;            // returns normalized vector
        void     normalize(void);                   // normalizes the vector
        
        Vector3& operator=(const Vector3& v);       // assignment
        void     operator*=(const float f);         // multiply assign
        void     operator/=(const float f);         // division assign
        void     operator+=(const Vector3& v);      // addition assign
        void     operator-=(const Vector3& v);      // subtraction assign
        
        Vector3  operator+(const Vector3& v) const; // addition
        Vector3  operator-(const Vector3& v) const; // subtraction
        Vector3  operator*(const Vector3& v) const; // component wise multiplication
        float    operator|(const Vector3& v) const; // dot product
        Vector3  operator%(const Vector3& v) const; // cross product
        Vector3  operator*(const float f) const;    // multiply with scalar
        Vector3  operator/(const float f) const;    // division by scalar
        Vector3  operator-(void) const;             // negate
        
        union {
            struct {
                float x;
                float y;
                float z;
            };

            float elem[3];
        };
        
    private:
        static float nan;
};

#endif
