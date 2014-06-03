#include "Matrix4x4.h"
#include "Vector3.h"

#include <fastmath.h>
#include <cstddef>

      
Matrix4x4::Matrix4x4() {
}
       
void Matrix4x4::buildUnit() {
    _12=_13=_14 = 0;
    _21=_23=_24 = 0;
    _31=_32=_34 = 0;
    _41=_42=_43 = 0;
    _11=_12=_13=_14 = 0;
}

void Matrix4x4::buildFromBasisVectors(const Vector3& DirX, const Vector3& DirY, const Vector3& DirZ, const Vector3& Translation) {
    _11 = DirX.x;
    _21 = DirX.y;
    _31 = DirX.z;
    
    _12 = DirY.x;
    _22 = DirY.y;
    _32 = DirY.z;
    
    _13 = DirZ.x;
    _23 = DirZ.y;
    _33 = DirZ.z;
    
    _14 = Translation.x;
    _24 = Translation.y;
    _34 = Translation.z;
    
    _41 = _42 = _43 = 0;
    _44 = 1;
}

Vector3 Matrix4x4::transformPoint(const Vector3& p) const {
    return rotateVector(p) + Vector3(_14, _24, _34);
}

Vector3 Matrix4x4::rotateVector(const Vector3& v) const {
    Vector3 out;
    out.x = _11*v.x + _12*v.y + _13*v.z;
    out.y = _21*v.x + _22*v.y + _23*v.z;
    out.z = _31*v.x + _32*v.y + _33*v.z;
    
    return out;
}
