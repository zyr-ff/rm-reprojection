#pragma once

// 3D 向量 / 矩阵 / 相机内参
struct Vec3 {
    double x, y, z;
};

struct Vec2 {
    double u, v;
};

struct Mat3 {
    double r[9];
};

struct Intrinsics {
    double fx, fy, cx, cy;
};

// 矩阵 × 向量：Pc = R · Pw
Vec3 matVecMul(const Mat3& M, const Vec3& v);

// 向量加法：Pc = R · Pw + t
Vec3 vecAdd(const Vec3& a, const Vec3& b);

// 针孔投影：(u, v) = (fx·x/z + cx, fy·y/z + cy)
Vec2 projectPoint(const Intrinsics& K, const Vec3& Pc);

// 像素欧氏距离：sqrt(Δu² + Δv²)
double pixelError(const Vec2& a, const Vec2& b);