#include <iostream>
#include <iomanip>
#include <cmath>
#include <random>
#include <stdexcept>
#include <string>
#include "projection.h"

static void printResult(const std::string& label,
                        const Vec3& Pc,
                        const Vec2& projected,
                        const Vec2& observed) {
    std::cout << std::fixed << std::setprecision(6);
    std::cout << "--- " << label << " ---\n";
    std::cout << "  Pc        = (" << Pc.x << ", " << Pc.y << ", " << Pc.z << ")\n";
    std::cout << "  Projected = (" << projected.u << ", " << projected.v << ")\n";
    std::cout << "  Observed  = (" << observed.u << ", " << observed.v << ")\n";
    std::cout << "  Pixel err = " << pixelError(projected, observed) << "\n\n";
}

static void printException(const std::string& label, const std::exception& e) {
    std::cout << "--- " << label << " ---\n";
    std::cout << "  [caught exception] " << e.what() << "\n\n";
}

int main() {
    Vec3 Pw{ 0.1, 0.2, 1.5 };
    const double a  = 3.14159265358979323846 / 6.0;   // 30 度
    const double c0 = std::cos(a);
    const double s0 = std::sin(a);

    Mat3 R; // 绕 Y 轴 30 度
    R.r[0] =  c0; R.r[1] = 0.0; R.r[2] =  s0;
    R.r[3] = 0.0; R.r[4] = 1.0; R.r[5] = 0.0;
    R.r[6] = -s0; R.r[7] = 0.0; R.r[8] =  c0;
    Vec3 t{ 0.5, 0.0, 0.0 };

    Intrinsics K{ 800.0, 800.0, 320.0, 240.0 };

    // ===== Case A：正常点 + 加少量噪声的观测 =====
    try {
        Vec3  Pc        = vecAdd(matVecMul(R, Pw), t);
        Vec2  projected = projectPoint(K, Pc);
        Vec2  observed{ projected.u + 0.5, projected.v - 0.3 };
        printResult("Case A: 正常点 + 噪声观测", Pc, projected, observed);
    } catch (const std::exception& e) {
        printException("Case A", e);
    }

    // ===== Case B：相机背后（Zc<0）→ 应抛异常 =====
    try {
        Vec3  Pc{ 0.1, 0.2, -1.0 };
        Vec2  projected = projectPoint(K, Pc);
        Vec2  observed{ 0.0, 0.0 };
        printResult("Case B: 相机背后的点", Pc, projected, observed);
    } catch (const std::exception& e) {
        printException("Case B", e);
    }

    // ===== Case C：自造数据（观测 = 投影）→ 误差应 ≈ 0 =====
    try {
        Vec3  Pc        = vecAdd(matVecMul(R, Pw), t);
        Vec2  projected = projectPoint(K, Pc);
        Vec2  observed  = projected;
        printResult("Case C: 自造数据（observed = projected）", Pc, projected, observed);
    } catch (const std::exception& e) {
        printException("Case C", e);
    }

    // ===== Case D：100 次随机高斯噪声试验 =====
    {
        std::cout << "--- Case D: 100 trials, Gaussian sigma = 1.0 px ---\n";
        std::mt19937 rng(42);
        std::normal_distribution<double> N01(0.0, 1.0);
        Vec3  Pc = vecAdd(matVecMul(R, Pw), t);
        Vec2  truth = projectPoint(K, Pc);
        double total = 0.0;
        for (int i = 0; i < 100; ++i) {
            Vec2 observed{ truth.u + N01(rng), truth.v + N01(rng) };
            total += pixelError(truth, observed);
        }
        std::cout << std::fixed << std::setprecision(6);
        std::cout << "  Truth    = (" << truth.u << ", " << truth.v << ")\n";
        std::cout << "  Mean err = " << (total / 100.0)
                  << "  (期望 ≈ 1.2533 px = sigma * sqrt(pi/2))\n\n";
    }

    return 0;
}