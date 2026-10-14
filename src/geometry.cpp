#include "collision.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>

namespace sweep {
namespace {

using Wide = long double;
using WideVec = glm::vec<3, Wide, glm::defaultp>;

bool finiteVector(Vec3 value) {
    return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
}

void validateTriangle(const Triangle& triangle) {
    if (!finiteVector(triangle.first) || !finiteVector(triangle.second) || !finiteVector(triangle.third)) {
        throw std::invalid_argument("Triangle coordinates must be finite");
    }
}

Wide squaredLength(WideVec value) {
    return glm::dot(value, value);
}

struct TriangleData {
    WideVec origin;
    std::array<WideVec, 3> corners;
    WideVec normal;
    Wide normalSquared;
    Wide longestEdge;

    explicit TriangleData(const Triangle& triangle)
        : origin(triangle.first),
          corners{WideVec(0), WideVec(triangle.second) - origin, WideVec(triangle.third) - origin},
          normal(glm::cross(corners[1], corners[2])),
          normalSquared(squaredLength(normal)),
          longestEdge(std::sqrt(std::max({squaredLength(corners[1]), squaredLength(corners[2]),
                                         squaredLength(corners[2] - corners[1])}))) {}
};

struct ClosestFeature {
    WideVec position{0};
    Wide distanceSquared = std::numeric_limits<Wide>::infinity();
    Feature feature = Feature::vertex;
};

bool insideFace(WideVec position, const TriangleData& triangle) {
    for (std::size_t edgeIndex = 0; edgeIndex < 3; ++edgeIndex) {
        const WideVec edgeStart = triangle.corners[edgeIndex];
        const WideVec edge = triangle.corners[(edgeIndex + 1) % 3] - edgeStart;
        if (glm::dot(glm::cross(edge, position - edgeStart), triangle.normal) < 0) {
            return false;
        }
    }
    return true;
}

ClosestFeature closestFeature(WideVec position, const TriangleData& triangle) {
    ClosestFeature closest;
    for (std::size_t edgeIndex = 0; edgeIndex < 3; ++edgeIndex) {
        const WideVec edgeStart = triangle.corners[edgeIndex];
        const WideVec edge = triangle.corners[(edgeIndex + 1) % 3] - edgeStart;
        const Wide edgeSquared = squaredLength(edge);
        const Wide parameter = edgeSquared > 0 ? std::clamp(glm::dot(position - edgeStart, edge) / edgeSquared, Wide(0), Wide(1)) : 0;
        const WideVec edgePosition = edgeStart + parameter * edge;
        const Wide distanceSquared = squaredLength(position - edgePosition);
        if (distanceSquared < closest.distanceSquared) {
            closest = {edgePosition, distanceSquared, parameter > 0 && parameter < 1 ? Feature::edge : Feature::vertex};
        }
    }
    if (triangle.normalSquared > 0) {
        const WideVec projected = position - triangle.normal * (glm::dot(position, triangle.normal) / triangle.normalSquared);
        const Wide distanceSquared = squaredLength(position - projected);
        if (distanceSquared < closest.distanceSquared && insideFace(projected, triangle)) {
            closest = {projected, distanceSquared, Feature::face};
        }
    }
    return closest;
}

WideVec contactNormal(WideVec center, WideVec position, WideVec displacement, const TriangleData& triangle) {
    const WideVec separation = center - position;
    const Wide separationSquared = squaredLength(separation);
    if (separationSquared > 0) {
        return separation / std::sqrt(separationSquared);
    }
    if (triangle.normalSquared > 0) {
        const WideVec normal = triangle.normal / std::sqrt(triangle.normalSquared);
        return glm::dot(normal, displacement) > 0 ? -normal : normal;
    }
    WideVec longestEdge = triangle.corners[1];
    if (squaredLength(triangle.corners[2]) > squaredLength(longestEdge)) {
        longestEdge = triangle.corners[2];
    }
    if (squaredLength(triangle.corners[2] - triangle.corners[1]) > squaredLength(longestEdge)) {
        longestEdge = triangle.corners[2] - triangle.corners[1];
    }
    const Wide edgeSquared = squaredLength(longestEdge);
    if (edgeSquared > 0) {
        const WideVec perpendicular = displacement - longestEdge * (glm::dot(displacement, longestEdge) / edgeSquared);
        if (squaredLength(perpendicular) > 0) {
            return -perpendicular / std::sqrt(squaredLength(perpendicular));
        }
        const WideVec absoluteEdge = glm::abs(longestEdge);
        const WideVec axis = absoluteEdge.x <= absoluteEdge.y && absoluteEdge.x <= absoluteEdge.z ? WideVec(1, 0, 0)
                           : absoluteEdge.y <= absoluteEdge.z ? WideVec(0, 1, 0) : WideVec(0, 0, 1);
        const WideVec normal = glm::cross(longestEdge, axis);
        return normal / std::sqrt(squaredLength(normal));
    }
    const Wide displacementSquared = squaredLength(displacement);
    return displacementSquared > 0 ? -displacement / std::sqrt(displacementSquared) : WideVec(0, 1, 0);
}

template<typename Candidate>
void radialRoots(WideVec relativeStart, WideVec displacement, Wide radius, Wide lengthTolerance, Candidate&& candidate) {
    const Wide motionSquared = squaredLength(displacement);
    if (motionSquared == 0) {
        return;
    }
    const Wide projection = glm::dot(relativeStart, displacement);
    const Wide closestTime = -projection / motionSquared;
    const WideVec closestOffset = relativeStart + closestTime * displacement;
    const Wide closestDistance = std::sqrt(squaredLength(closestOffset));
    if (closestDistance > radius + lengthTolerance) {
        return;
    }
    const Wide radialSquared = std::max(Wide(0), (radius - closestDistance) * (radius + closestDistance));
    if (radialSquared == 0) {
        candidate(closestTime);
        return;
    }
    const Wide discriminantRoot = std::sqrt(motionSquared) * std::sqrt(radialSquared);
    const Wide stableNumerator = -projection - std::copysign(discriminantRoot, projection);
    if (stableNumerator == 0) {
        candidate(closestTime);
        return;
    }
    const Wide startDistance = std::sqrt(squaredLength(relativeStart));
    const Wide constant = (startDistance - radius) * (startDistance + radius);
    Wide firstTime = stableNumerator / motionSquared;
    Wide secondTime = constant / stableNumerator;
    if (firstTime > secondTime) {
        std::swap(firstTime, secondTime);
    }
    candidate(firstTime);
    candidate(secondTime);
}

}

Vec3 closestPoint(Vec3 position, const Triangle& triangle) {
    if (!finiteVector(position)) {
        throw std::invalid_argument("Point coordinates must be finite");
    }
    validateTriangle(triangle);
    const TriangleData geometry(triangle);
    return Vec3(geometry.origin + closestFeature(WideVec(position) - geometry.origin, geometry).position);
}

Proximity closestSegment(Vec3 start, Vec3 end, const Triangle& triangle) {
    if (!finiteVector(start) || !finiteVector(end)) {
        throw std::invalid_argument("Segment coordinates must be finite");
    }
    validateTriangle(triangle);
    const TriangleData geometry(triangle);
    const WideVec localStart = WideVec(start) - geometry.origin;
    const WideVec displacement = WideVec(end) - WideVec(start);
    const Wide motionSquared = squaredLength(displacement);
    Wide bestSquared = std::numeric_limits<Wide>::infinity();
    Wide bestTime = 0;
    WideVec bestPosition(0);
    auto candidate = [&](Wide time, WideVec position) {
        const Wide distanceSquared = squaredLength(localStart + time * displacement - position);
        if (distanceSquared < bestSquared || (distanceSquared == bestSquared && time < bestTime)) {
            bestSquared = distanceSquared;
            bestTime = time;
            bestPosition = position;
        }
    };
    candidate(0, closestFeature(localStart, geometry).position);
    candidate(1, closestFeature(localStart + displacement, geometry).position);
    if (motionSquared > 0) {
        for (std::size_t edgeIndex = 0; edgeIndex < 3; ++edgeIndex) {
            const WideVec edgeStart = geometry.corners[edgeIndex];
            const WideVec edgeEnd = geometry.corners[(edgeIndex + 1) % 3];
            const WideVec edge = edgeEnd - edgeStart;
            const Wide edgeSquared = squaredLength(edge);
            candidate(std::clamp(glm::dot(edgeStart - localStart, displacement) / motionSquared, Wide(0), Wide(1)), edgeStart);
            candidate(std::clamp(glm::dot(edgeEnd - localStart, displacement) / motionSquared, Wide(0), Wide(1)), edgeEnd);
            if (edgeSquared == 0) {
                continue;
            }
            const WideVec crossDirection = glm::cross(displacement, edge);
            const Wide crossSquared = squaredLength(crossDirection);
            if (crossSquared > 0) {
                const WideVec separation = edgeStart - localStart;
                const Wide motionTime = glm::dot(glm::cross(separation, edge), crossDirection) / crossSquared;
                const Wide edgeTime = glm::dot(glm::cross(separation, displacement), crossDirection) / crossSquared;
                if (motionTime >= 0 && motionTime <= 1 && edgeTime >= 0 && edgeTime <= 1) {
                    candidate(motionTime, edgeStart + edgeTime * edge);
                }
            }
        }
        const Wide planeMotion = glm::dot(displacement, geometry.normal);
        if (geometry.normalSquared > 0 && planeMotion != 0) {
            const Wide planeTime = -glm::dot(localStart, geometry.normal) / planeMotion;
            if (planeTime >= 0 && planeTime <= 1) {
                const WideVec planePosition = localStart + planeTime * displacement;
                if (insideFace(planePosition, geometry)) {
                    candidate(planeTime, planePosition);
                }
            }
        }
    }
    Proximity result;
    result.distance = static_cast<double>(std::sqrt(bestSquared));
    result.time = static_cast<double>(bestTime);
    result.center = Vec3(geometry.origin + localStart + bestTime * displacement);
    result.position = Vec3(geometry.origin + bestPosition);
    return result;
}

Contact sweepTriangle(const Motion& motion, const Triangle& triangle, Tolerances tolerances) {
    if (!finiteVector(motion.start) || !finiteVector(motion.end) || !std::isfinite(motion.radius) || motion.radius < 0) {
        throw std::invalid_argument("Motion coordinates and radius must be finite with nonnegative radius");
    }
    if (!std::isfinite(tolerances.absolute) || !std::isfinite(tolerances.relative) || tolerances.absolute < 0 || tolerances.relative < 0) {
        throw std::invalid_argument("Tolerances must be finite and nonnegative");
    }
    validateTriangle(triangle);
    const TriangleData geometry(triangle);
    const WideVec localStart = WideVec(motion.start) - geometry.origin;
    const WideVec displacement = WideVec(motion.end) - WideVec(motion.start);
    const Wide motionLength = std::sqrt(squaredLength(displacement));
    const Wide radius = motion.radius;
    const Wide lengthTolerance = tolerances.absolute + tolerances.relative * std::max({radius, geometry.longestEdge, motionLength});
    const Wide contactRadius = radius + lengthTolerance;
    const Wide timeTolerance = motionLength > 0 ? lengthTolerance / motionLength : 0;
    Contact result;
    const auto initial = closestFeature(localStart, geometry);
    if (initial.distanceSquared <= contactRadius * contactRadius) {
        result.hit = true;
        result.startsOverlapping = true;
        result.time = 0;
        result.center = motion.start;
        result.position = Vec3(geometry.origin + initial.position);
        result.normal = Vec3(contactNormal(localStart, initial.position, displacement, geometry));
        result.feature = initial.feature;
        return result;
    }
    if (motionLength == 0) {
        return result;
    }
    Wide earliestTime = 1;
    auto candidate = [&](Wide time) {
        if (!std::isfinite(time) || time < -timeTolerance || time > 1 + timeTolerance) {
            return;
        }
        time = std::clamp(time, Wide(0), Wide(1));
        if (result.hit && time >= earliestTime) {
            return;
        }
        const WideVec center = localStart + time * displacement;
        const auto closest = closestFeature(center, geometry);
        const Wide numericalMargin = 32 * std::numeric_limits<Wide>::epsilon() * std::max({radius, geometry.longestEdge, motionLength});
        if (std::sqrt(closest.distanceSquared) > contactRadius + numericalMargin) {
            return;
        }
        earliestTime = time;
        result.hit = true;
        result.time = static_cast<double>(time);
        result.center = Vec3(geometry.origin + center);
        result.position = Vec3(geometry.origin + closest.position);
        result.normal = Vec3(contactNormal(center, closest.position, displacement, geometry));
        result.feature = closest.feature;
    };
    if (geometry.normalSquared > 0) {
        const WideVec normal = geometry.normal / std::sqrt(geometry.normalSquared);
        const Wide planeMotion = glm::dot(displacement, normal);
        if (planeMotion != 0) {
            const Wide planeStart = glm::dot(localStart, normal);
            candidate((radius - planeStart) / planeMotion);
            candidate((-radius - planeStart) / planeMotion);
        }
    }
    for (std::size_t edgeIndex = 0; edgeIndex < 3; ++edgeIndex) {
        const WideVec edgeStart = geometry.corners[edgeIndex];
        const WideVec relativeStart = localStart - edgeStart;
        radialRoots(relativeStart, displacement, radius, lengthTolerance, candidate);
        const WideVec edge = geometry.corners[(edgeIndex + 1) % 3] - edgeStart;
        const Wide edgeSquared = squaredLength(edge);
        if (edgeSquared > 0) {
            const WideVec perpendicularStart = relativeStart - edge * (glm::dot(relativeStart, edge) / edgeSquared);
            const WideVec perpendicularMotion = displacement - edge * (glm::dot(displacement, edge) / edgeSquared);
            radialRoots(perpendicularStart, perpendicularMotion, radius, lengthTolerance, candidate);
        }
    }
    return result;
}

double segmentBoxDistanceSquared(Vec3 start, Vec3 end, const Bounds& bounds) {
    if (!finiteVector(start) || !finiteVector(end) || !finiteVector(bounds.minimum) || !finiteVector(bounds.maximum)
        || bounds.minimum.x > bounds.maximum.x || bounds.minimum.y > bounds.maximum.y || bounds.minimum.z > bounds.maximum.z) {
        throw std::invalid_argument("Segment and ordered bounds must be finite");
    }
    const WideVec localStart = WideVec(start) - WideVec(bounds.minimum);
    const WideVec displacement = WideVec(end) - WideVec(start);
    const WideVec extent = WideVec(bounds.maximum) - WideVec(bounds.minimum);
    std::array<Wide, 8> intervals{0, 1};
    std::size_t intervalCount = 2;
    for (int axis = 0; axis < 3; ++axis) {
        if (displacement[axis] != 0) {
            for (const Wide boundary : {Wide(0), extent[axis]}) {
                const Wide time = (boundary - localStart[axis]) / displacement[axis];
                if (time > 0 && time < 1) {
                    intervals[intervalCount++] = time;
                }
            }
        }
    }
    std::sort(intervals.begin(), intervals.begin() + static_cast<std::ptrdiff_t>(intervalCount));
    Wide bestSquared = std::numeric_limits<Wide>::infinity();
    auto evaluate = [&](Wide time) {
        const WideVec point = localStart + time * displacement;
        const WideVec nearest = glm::clamp(point, WideVec(0), extent);
        bestSquared = std::min(bestSquared, squaredLength(point - nearest));
    };
    for (std::size_t intervalIndex = 1; intervalIndex < intervalCount; ++intervalIndex) {
        const Wide left = intervals[intervalIndex - 1];
        const Wide right = intervals[intervalIndex];
        const Wide midpoint = (left + right) / 2;
        Wide quadratic = 0;
        Wide linear = 0;
        for (int axis = 0; axis < 3; ++axis) {
            const Wide coordinate = localStart[axis] + midpoint * displacement[axis];
            if (coordinate < 0 || coordinate > extent[axis]) {
                const Wide offset = localStart[axis] - (coordinate < 0 ? 0 : extent[axis]);
                quadratic += displacement[axis] * displacement[axis];
                linear += offset * displacement[axis];
            }
        }
        evaluate(left);
        evaluate(right);
        if (quadratic > 0) {
            evaluate(std::clamp(-linear / quadratic, left, right));
        }
    }
    return static_cast<double>(bestSquared);
}

}
