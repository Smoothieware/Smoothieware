#ifndef _VECTOR2_H
#define _VECTOR2_H

class Vector2 {
    public:
        Vector2();
        Vector2(float, float);

        float&   operator[](int);

        float    dot(const Vector2&) const;         // dot product
        float    magsq() const;                     // squared vector length
        float    mag() const;                       // vector length
        Vector2  add(const Vector2&) const;         // addition
        Vector2  sub(const Vector2&) const;         // subtraction
        Vector2  mul(float) const;                  // scalar multiplication
        Vector2  mul(const Vector2& v) const;       // component wise multiplication
        Vector2  normalized(void) const;            // returns normalized vector
        void     normalize(void);                   // normalizes the vector
        
        Vector2& operator=(const Vector2& v);       // assignment
        void     operator*=(const float f);         // multiply assign
        void     operator/=(const float f);         // division assign
        void     operator+=(const Vector2& v);      // addition assign
        void     operator-=(const Vector2& v);      // subtraction assign
      
        Vector2  operator+(const Vector2& v) const; // addition
        Vector2  operator-(const Vector2& v) const; // subtraction
        Vector2  operator*(const Vector2& v) const; // component wise multiplication
        float    operator|(const Vector2& v) const; // dot product
        Vector2  operator*(const float f) const;    // multiply with scalar
        Vector2  operator/(const float f) const;    // division by scalar
        Vector2  operator-(void) const;             // negate
        
        union {
            struct {
                float x;
                float y;
            };

            float elem[2];
        };
        
    private:
        static float nan;
};

#endif
