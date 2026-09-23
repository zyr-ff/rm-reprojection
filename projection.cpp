#include "projection.h"
#include <cmath>
#include <stdexcept>

// R * Pw (外参旋转)
Vec3 matVecMul(const Mat3& M, const Vec3& v) {
    Vec3 out;
    out.x = M.r[0]*v.x + M.r[1]*v.y + M.r[2]*v.z;
    out.y = M.r[3]*v.x + M.r[4]*v.y + M.r[5]*v.z;
    out.z = M.r[6]*v.x + M.r[7]*v.y + M.r[8]*v.z;
    return out;
}

// a + b  (外参平移)
Vec3 vecAdd(const Vec3& a, const Vec3& b) {
    Vec3 out;
    out.x = a.x + b.x;
    out.y = a.y + b.y;
    out.z = a.z + b.z;
    return out;
}

// 投影:K 内参、Pc 相机坐标(必须在 z>0 一侧)
Vec2 projectPoint(const Intrinsics& K, const Vec3& Pc) {
    if (Pc.z <= 0.0) {
        throw std::invalid_argument("projectPoint: Zc must be > 0 (point is behind camera)");
    }
    const double invZ = 1.0 / Pc.z;
    Vec2 out;
    out.u = K.fx * (Pc.x * invZ) + K.cx;
    out.v = K.fy * (Pc.y * invZ) + K.cy;
    return out;
}

// 像素欧氏距离
double pixelError(const Vec2& a, const Vec2& b) {
    const double dx = a.u - b.u;
    const double dy = a.v - b.v;
    return std::hypot(dx, dy);
}
