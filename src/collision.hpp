#pragma once

#include <glm/glm.hpp>

#include <atomic>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <limits>
#include <mutex>
#include <span>
#include <thread>
#include <vector>

namespace sweep {

using Vec3 = glm::vec<3, double, glm::packed_highp>;
static_assert(sizeof(Vec3) == 3 * sizeof(double));
constexpr std::uint32_t noTriangle = std::numeric_limits<std::uint32_t>::max();

struct Triangle {
    Vec3 first{0.0};
    Vec3 second{0.0};
    Vec3 third{0.0};
};

struct Bounds {
    Vec3 minimum{std::numeric_limits<double>::infinity()};
    Vec3 maximum{-std::numeric_limits<double>::infinity()};
};

struct Motion {
    Vec3 start{0.0};
    Vec3 end{0.0};
    double radius = 0.4;
};

struct Tolerances {
    double absolute = 1e-10;
    double relative = 1e-10;
};

enum class Feature { face, edge, vertex };

struct Contact {
    bool hit = false;
    bool startsOverlapping = false;
    double time = 1.0;
    Vec3 center{0.0};
    Vec3 position{0.0};
    Vec3 normal{0.0};
    std::uint32_t triangleIndex = noTriangle;
    Feature feature = Feature::face;
};

struct Proximity {
    double distance = std::numeric_limits<double>::infinity();
    double time = 0.0;
    Vec3 center{0.0};
    Vec3 position{0.0};
    std::uint32_t triangleIndex = noTriangle;
};

struct QueryStats {
    std::uint64_t triangleTests = 0;
    std::uint64_t boxTests = 0;
};

struct MotionResult {
    Contact contact;
    Proximity clearance;
    QueryStats stats;
};

struct PathResult {
    Contact contact;
    Proximity clearance;
    std::size_t contactSegment = 0;
    std::size_t clearanceSegment = 0;
    double length = 0.0;
    QueryStats stats;
};

Vec3 closestPoint(Vec3 position, const Triangle& triangle);
Proximity closestSegment(Vec3 start, Vec3 end, const Triangle& triangle);
Contact sweepTriangle(const Motion& motion, const Triangle& triangle, Tolerances tolerances = {});
double segmentBoxDistanceSquared(Vec3 start, Vec3 end, const Bounds& bounds);
std::vector<Triangle> loadObj(const std::filesystem::path& filePath);

class CollisionScene {
public:
    explicit CollisionScene(std::vector<Triangle> triangles, Tolerances tolerances = {});
    MotionResult query(const Motion& motion, bool useBvh = true) const;
    PathResult queryPath(std::span<const Vec3> waypoints, double radius, bool useBvh = true) const;
    const std::vector<Triangle>& triangles() const;
    const Bounds& bounds() const;
    std::size_t nodeCount() const;
    std::size_t memoryBytes() const;

private:
    struct Node {
        Bounds bounds;
        std::uint32_t firstTriangle = 0;
        std::uint32_t triangleCount = 0;
        std::uint32_t leftChild = 0;
        std::uint32_t rightChild = 0;
    };

    std::uint32_t build(std::uint32_t firstTriangle, std::uint32_t triangleCount, std::uint32_t depth);
    std::vector<Triangle> sceneTriangles;
    std::vector<std::uint32_t> triangleIndices;
    std::vector<Node> nodes;
    Bounds sceneBounds;
    Tolerances tolerances;
};

class BatchExecutor {
public:
    explicit BatchExecutor(std::size_t threadCount = 0);
    ~BatchExecutor();
    BatchExecutor(const BatchExecutor&) = delete;
    BatchExecutor& operator=(const BatchExecutor&) = delete;
    void execute(const CollisionScene& scene, std::span<const Motion> motions, std::span<MotionResult> results);
    std::size_t threadCount() const;

private:
    void worker();
    void process();
    std::vector<std::thread> workers;
    std::mutex dispatchMutex;
    std::mutex stateMutex;
    std::condition_variable startCondition;
    std::condition_variable doneCondition;
    const CollisionScene* activeScene = nullptr;
    std::span<const Motion> activeMotions;
    std::span<MotionResult> activeResults;
    std::atomic<std::size_t> nextIndex{0};
    std::size_t generation = 0;
    std::size_t pendingWorkers = 0;
    bool stopping = false;
};

}
