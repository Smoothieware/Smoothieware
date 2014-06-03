#ifndef _MATRIX4x4_H
#define _MATRIX4x4_H

class Vector3;

class Matrix4x4 {
    public:
        Matrix4x4();
        
        void    buildUnit();                                
        void    buildFromBasisVectors(const Vector3& DirX, const Vector3& DirY, const Vector3& DirZ, const Vector3& Translation);      
        Vector3 transformPoint(const Vector3& p) const;  //< rotation and translation
        Vector3 rotateVector(const Vector3& v) const;    //< rotation only
        
        union {
            struct {
                float _11,_12,_13,_14;
                float _21,_22,_23,_24;
                float _31,_32,_33,_34;
                float _41,_42,_43,_44;
            };

            float m[4][4];
        };
};

#endif
