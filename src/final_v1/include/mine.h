// Copyright by BeeX [2026]
#pragma once

#include <Eigen/Geometry>

#include <string>
#include <vector>

namespace final_v1 {

// A mine is a cone cut flat at both ends: its top face and base diameters and its height, in metres.
struct MineShape {
    std::string name;
    double      face = 0.0, base = 0.0, height = 0.0;
};

// A mine as surface samples: metres, origin at the middle of its top face, axis up +y, normals pointing out.
struct MineModel {
    std::string                  name;
    std::vector<Eigen::Vector3f> points, normals;
};

// The top face, the side and the base, sampled `spacing` metres apart.
MineModel sampleMine(const MineShape &shape, double spacing = 0.006);

}  // namespace final_v1
