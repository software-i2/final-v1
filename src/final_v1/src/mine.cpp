// Copyright by BeeX [2026]
#include <mine.h>

#include <algorithm>
#include <cmath>

namespace final_v1 {
namespace {

// Rings `spacing` apart from the middle out, each ring's points `spacing` apart.
void disc(double radius, double y, float up, double spacing, MineModel &m) {
    for (double r = 0.0; r <= radius; r += spacing) {
        const int n = std::max(1, static_cast<int>(std::ceil(2.0 * M_PI * r / spacing)));
        for (int k = 0; k < n; ++k) {
            const double a = 2.0 * M_PI * k / n;
            m.points.emplace_back(r * std::cos(a), y, r * std::sin(a));
            m.normals.emplace_back(0.0f, up, 0.0f);
        }
    }
}

}  // namespace

MineModel sampleMine(const MineShape &shape, double spacing) {
    MineModel    m;
    const double top = 0.5 * shape.face, bottom = 0.5 * shape.base, h = shape.height;
    m.name = shape.name;
    disc(top, 0.0, 1.0f, spacing, m);
    disc(bottom, -h, -1.0f, spacing, m);
    const double slant = std::hypot(h, bottom - top);
    const int    rings = std::max(1, static_cast<int>(std::ceil(slant / spacing)));
    for (int i = 0; i <= rings; ++i) {
        const double t = static_cast<double>(i) / rings, r = top + (bottom - top) * t;
        const int    n = std::max(1, static_cast<int>(std::ceil(2.0 * M_PI * r / spacing)));
        for (int k = 0; k < n; ++k) {
            const double a = 2.0 * M_PI * k / n;
            m.points.emplace_back(r * std::cos(a), -h * t, r * std::sin(a));
            m.normals.push_back(Eigen::Vector3f(h * std::cos(a), bottom - top, h * std::sin(a)).normalized());
        }
    }
    return m;
}

}  // namespace final_v1
