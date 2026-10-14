#include "collision.hpp"

#define TINYOBJLOADER_USE_DOUBLE
#define TINYOBJLOADER_IMPLEMENTATION
#include <tiny_obj_loader.h>

#include <algorithm>
#include <array>
#include <charconv>
#include <cmath>
#include <fstream>
#include <iterator>
#include <numeric>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>

namespace sweep {
namespace {

constexpr std::uint32_t leafSize = 6;
constexpr std::uint32_t maxDepth = 48;
constexpr int binCount = 12;
constexpr double coordinateLimit = 1e60;

double magnitude(Vec3 value) {
    return std::hypot(value.x, value.y, value.z);
}

double largestCoordinate(Vec3 value) {
    return std::max({std::abs(value.x), std::abs(value.y), std::abs(value.z)});
}

bool validPosition(Vec3 value) {
    return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z) &&
           largestCoordinate(value) <= coordinateLimit;
}

void validateRadius(double radius) {
    if (!std::isfinite(radius) || radius < 0.0 || radius > coordinateLimit) {
        throw std::invalid_argument("Radius must be finite and between zero and 1e60");
    }
}

void validateMotion(const Motion& motion) {
    validateRadius(motion.radius);
    if (!validPosition(motion.start) || !validPosition(motion.end)) {
        throw std::invalid_argument("Motion coordinates must be finite and within 1e60");
    }
}

void extend(Bounds& bounds, Vec3 position) {
    bounds.minimum = glm::min(bounds.minimum, position);
    bounds.maximum = glm::max(bounds.maximum, position);
}

void extend(Bounds& bounds, const Triangle& triangle) {
    extend(bounds, triangle.first);
    extend(bounds, triangle.second);
    extend(bounds, triangle.third);
}

void extend(Bounds& bounds, const Bounds& additional) {
    extend(bounds, additional.minimum);
    extend(bounds, additional.maximum);
}

Vec3 centroid(const Triangle& triangle) {
    return triangle.first / 3.0 + triangle.second / 3.0 + triangle.third / 3.0;
}

double surfaceArea(const Bounds& bounds) {
    const Vec3 extent = bounds.maximum - bounds.minimum;
    return 2.0 * (extent.x * extent.y + extent.y * extent.z + extent.z * extent.x);
}

void padBounds(Bounds& bounds) {
    const double infinity = std::numeric_limits<double>::infinity();
    for (int axis = 0; axis < 3; ++axis) {
        const double padding = 64.0 * std::numeric_limits<double>::epsilon() *
                               std::max({std::abs(bounds.minimum[axis]),
                                         std::abs(bounds.maximum[axis]),
                                         bounds.maximum[axis] - bounds.minimum[axis]});
        bounds.minimum[axis] = std::nextafter(bounds.minimum[axis] - padding, -infinity);
        bounds.maximum[axis] = std::nextafter(bounds.maximum[axis] + padding, infinity);
    }
}

bool intersectBounds(const Motion& motion, const Bounds& bounds, double expansion,
                     double lastTime, double& firstTime) {
    firstTime = 0.0;
    double farTime = lastTime;
    const Vec3 displacement = motion.end - motion.start;
    for (int axis = 0; axis < 3; ++axis) {
        const double minimum = bounds.minimum[axis] - expansion;
        const double maximum = bounds.maximum[axis] + expansion;
        if (displacement[axis] == 0.0) {
            if (motion.start[axis] < minimum || motion.start[axis] > maximum) {
                return false;
            }
            continue;
        }
        double nearTime = (minimum - motion.start[axis]) / displacement[axis];
        double exitTime = (maximum - motion.start[axis]) / displacement[axis];
        if (nearTime > exitTime) {
            std::swap(nearTime, exitTime);
        }
        firstTime = std::max(firstTime, nearTime);
        farTime = std::min(farTime, exitTime);
        if (firstTime > farTime) {
            return false;
        }
    }
    return true;
}

bool preferContact(const Contact& candidate, const Contact& current) {
    return candidate.hit && (!current.hit || candidate.time < current.time ||
           (candidate.time == current.time && candidate.triangleIndex < current.triangleIndex));
}

bool preferProximity(const Proximity& candidate, const Proximity& current) {
    return candidate.distance < current.distance ||
           (candidate.distance == current.distance &&
            (candidate.triangleIndex < current.triangleIndex ||
             (candidate.triangleIndex == current.triangleIndex && candidate.time < current.time)));
}

void validateObjSource(const std::string& source) {
    std::istringstream input(source);
    std::string line;
    std::size_t lineNumber = 0;
    while (std::getline(input, line)) {
        ++lineNumber;
        std::istringstream fields(line);
        std::string keyword;
        fields >> keyword;
        if (keyword == "v" || keyword == "vn" || keyword == "vt") {
            std::string token;
            std::size_t coordinateCount = 0;
            while (fields >> token) {
                if (token.front() == '#') {
                    break;
                }
                const char* firstCharacter = token.data();
                const char* lastCharacter = firstCharacter + token.size();
                if (*firstCharacter == '+') {
                    ++firstCharacter;
                }
                double coordinate = 0.0;
                const auto result = std::from_chars(firstCharacter, lastCharacter, coordinate);
                if (result.ec != std::errc{} || result.ptr != lastCharacter ||
                    !std::isfinite(coordinate) || std::abs(coordinate) > coordinateLimit) {
                    throw std::runtime_error("Invalid OBJ coordinate at line " + std::to_string(lineNumber));
                }
                ++coordinateCount;
            }
            if ((keyword == "v" && coordinateCount < 3) ||
                (keyword == "vn" && coordinateCount != 3) ||
                (keyword == "vt" && (coordinateCount == 0 || coordinateCount > 3))) {
                throw std::runtime_error("Incomplete OBJ coordinate at line " + std::to_string(lineNumber));
            }
        } else if (keyword == "f") {
            std::size_t cornerCount = 0;
            std::string token;
            while (fields >> token) {
                if (token.front() == '#') {
                    break;
                }
                const std::string_view faceToken(token);
                std::size_t componentStart = 0;
                int componentIndex = 0;
                while (componentStart <= faceToken.size()) {
                    const std::size_t separator = faceToken.find('/', componentStart);
                    const std::size_t componentEnd = separator == std::string_view::npos ? faceToken.size() : separator;
                    const auto indexToken = faceToken.substr(componentStart, componentEnd - componentStart);
                    if (componentIndex > 2 ||
                        (indexToken.empty() && (componentIndex != 1 || separator == std::string_view::npos))) {
                        throw std::runtime_error("Invalid OBJ face index at line " + std::to_string(lineNumber));
                    }
                    if (!indexToken.empty()) {
                        int indexValue = 0;
                        const char* firstCharacter = indexToken.data();
                        const char* lastCharacter = firstCharacter + indexToken.size();
                        if (*firstCharacter == '+') {
                            ++firstCharacter;
                        }
                        const auto result = std::from_chars(firstCharacter, lastCharacter, indexValue);
                        if (result.ec != std::errc{} || result.ptr != lastCharacter || indexValue == 0) {
                            throw std::runtime_error("Invalid OBJ face index at line " + std::to_string(lineNumber));
                        }
                    }
                    if (separator == std::string_view::npos) {
                        break;
                    }
                    componentStart = separator + 1;
                    ++componentIndex;
                }
                ++cornerCount;
            }
            if (cornerCount != 3) {
                throw std::runtime_error("OBJ face must be triangular at line " + std::to_string(lineNumber));
            }
        }
    }
}

}

std::vector<Triangle> loadObj(const std::filesystem::path& filePath) {
    std::ifstream input(filePath, std::ios::binary);
    if (!input) {
        throw std::runtime_error("Cannot open OBJ: " + filePath.string());
    }
    const std::string source{std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>()};
    if (input.bad()) {
        throw std::runtime_error("Cannot read OBJ: " + filePath.string());
    }
    validateObjSource(source);

    tinyobj::ObjReaderConfig configuration;
    configuration.triangulate = false;
    configuration.vertex_color = false;
    tinyobj::ObjReader reader;
    if (!reader.ParseFromString(source, "", configuration)) {
        throw std::runtime_error("Invalid OBJ: " + reader.Error());
    }

    std::vector<Triangle> triangles;
    const auto& attributes = reader.GetAttrib();
    const auto& vertices = attributes.vertices;
    for (const auto& shape : reader.GetShapes()) {
        std::size_t indexOffset = 0;
        for (const auto faceSize : shape.mesh.num_face_vertices) {
            if (faceSize != 3 || indexOffset + 3 > shape.mesh.indices.size()) {
                throw std::runtime_error("OBJ must contain complete triangular faces");
            }
            std::array<Vec3, 3> corners;
            for (std::size_t cornerIndex = 0; cornerIndex < corners.size(); ++cornerIndex) {
                const auto& indices = shape.mesh.indices[indexOffset + cornerIndex];
                if (indices.vertex_index < 0 || static_cast<std::size_t>(indices.vertex_index) >= vertices.size() / 3 ||
                    indices.normal_index < -1 || indices.texcoord_index < -1 ||
                    (indices.normal_index >= 0 && static_cast<std::size_t>(indices.normal_index) >= attributes.normals.size() / 3) ||
                    (indices.texcoord_index >= 0 && static_cast<std::size_t>(indices.texcoord_index) >= attributes.texcoords.size() / 2)) {
                    throw std::runtime_error("OBJ face references a missing attribute");
                }
                const auto coordinateOffset = static_cast<std::size_t>(indices.vertex_index) * 3;
                corners[cornerIndex] = {vertices[coordinateOffset], vertices[coordinateOffset + 1], vertices[coordinateOffset + 2]};
                if (!validPosition(corners[cornerIndex])) {
                    throw std::runtime_error("OBJ coordinates must be finite and within 1e60");
                }
            }
            if (triangles.size() >= std::numeric_limits<std::uint32_t>::max() / 2) {
                throw std::runtime_error("OBJ contains too many triangles");
            }
            triangles.push_back({corners[0], corners[1], corners[2]});
            indexOffset += 3;
        }
    }
    if (triangles.empty()) {
        throw std::runtime_error("OBJ contains no triangular faces");
    }
    return triangles;
}

CollisionScene::CollisionScene(std::vector<Triangle> triangleData, Tolerances toleranceSettings)
    : sceneTriangles(std::move(triangleData)), tolerances(toleranceSettings) {
    if (!std::isfinite(toleranceSettings.absolute) || !std::isfinite(toleranceSettings.relative) ||
        toleranceSettings.absolute < 0.0 || toleranceSettings.relative < 0.0) {
        throw std::invalid_argument("Tolerances must be finite and nonnegative");
    }
    if (sceneTriangles.size() >= std::numeric_limits<std::uint32_t>::max() / 2) {
        throw std::invalid_argument("Scene contains too many triangles");
    }
    for (const auto& triangle : sceneTriangles) {
        if (!validPosition(triangle.first) || !validPosition(triangle.second) || !validPosition(triangle.third)) {
            throw std::invalid_argument("Triangle coordinates must be finite and within 1e60");
        }
        extend(sceneBounds, triangle);
    }
    triangleIndices.resize(sceneTriangles.size());
    std::iota(triangleIndices.begin(), triangleIndices.end(), std::uint32_t{0});
    nodes.reserve(sceneTriangles.size());
    if (!sceneTriangles.empty()) {
        build(0, static_cast<std::uint32_t>(sceneTriangles.size()), 0);
    }
}

std::uint32_t CollisionScene::build(std::uint32_t firstTriangle, std::uint32_t triangleCount, std::uint32_t depth) {
    const auto nodeIndex = static_cast<std::uint32_t>(nodes.size());
    Node node;
    node.firstTriangle = firstTriangle;
    node.triangleCount = triangleCount;
    Bounds centroidBounds;
    for (std::uint32_t offset = 0; offset < triangleCount; ++offset) {
        const auto& triangle = sceneTriangles[triangleIndices[firstTriangle + offset]];
        extend(node.bounds, triangle);
        extend(centroidBounds, centroid(triangle));
    }
    padBounds(node.bounds);
    nodes.push_back(node);
    if (triangleCount <= leafSize || depth >= maxDepth) {
        return nodeIndex;
    }

    struct Bin {
        Bounds bounds;
        std::uint32_t count = 0;
    };
    const Vec3 centroidExtent = centroidBounds.maximum - centroidBounds.minimum;
    double bestCost = std::numeric_limits<double>::infinity();
    int bestAxis = -1;
    int bestBoundary = 0;
    for (int axis = 0; axis < 3; ++axis) {
        if (centroidExtent[axis] <= 0.0) {
            continue;
        }
        std::array<Bin, binCount> bins;
        for (std::uint32_t offset = 0; offset < triangleCount; ++offset) {
            const auto& triangle = sceneTriangles[triangleIndices[firstTriangle + offset]];
            const double fraction = (centroid(triangle)[axis] - centroidBounds.minimum[axis]) / centroidExtent[axis];
            const int binIndex = std::clamp(static_cast<int>(fraction * binCount), 0, binCount - 1);
            extend(bins[binIndex].bounds, triangle);
            ++bins[binIndex].count;
        }
        std::array<Bin, binCount> suffixBins;
        Bin suffix;
        for (int binIndex = binCount - 1; binIndex >= 0; --binIndex) {
            if (bins[binIndex].count != 0) {
                extend(suffix.bounds, bins[binIndex].bounds);
                suffix.count += bins[binIndex].count;
            }
            suffixBins[binIndex] = suffix;
        }
        Bin prefix;
        for (int binIndex = 0; binIndex < binCount - 1; ++binIndex) {
            if (bins[binIndex].count != 0) {
                extend(prefix.bounds, bins[binIndex].bounds);
                prefix.count += bins[binIndex].count;
            }
            const Bin& remaining = suffixBins[binIndex + 1];
            if (prefix.count == 0 || remaining.count == 0) {
                continue;
            }
            const double cost = surfaceArea(prefix.bounds) * prefix.count + surfaceArea(remaining.bounds) * remaining.count;
            if (cost < bestCost) {
                bestCost = cost;
                bestAxis = axis;
                bestBoundary = binIndex;
            }
        }
    }

    const auto firstIndex = triangleIndices.begin() + firstTriangle;
    const auto endIndex = firstIndex + triangleCount;
    std::uint32_t leftCount = 0;
    if (bestAxis >= 0) {
        const auto middleIndex = std::partition(firstIndex, endIndex, [&](std::uint32_t triangleIndex) {
            const double fraction = (centroid(sceneTriangles[triangleIndex])[bestAxis] - centroidBounds.minimum[bestAxis]) /
                                    centroidExtent[bestAxis];
            const int binIndex = std::clamp(static_cast<int>(fraction * binCount), 0, binCount - 1);
            return binIndex <= bestBoundary;
        });
        leftCount = static_cast<std::uint32_t>(middleIndex - firstIndex);
    }
    if (leftCount == 0 || leftCount == triangleCount) {
        int splitAxis = centroidExtent.y > centroidExtent.x ? 1 : 0;
        if (centroidExtent.z > centroidExtent[splitAxis]) {
            splitAxis = 2;
        }
        leftCount = triangleCount / 2;
        std::nth_element(firstIndex, firstIndex + leftCount, endIndex,
                         [&](std::uint32_t firstTriangleIndex, std::uint32_t secondTriangleIndex) {
            const double firstCenter = centroid(sceneTriangles[firstTriangleIndex])[splitAxis];
            const double secondCenter = centroid(sceneTriangles[secondTriangleIndex])[splitAxis];
            return firstCenter < secondCenter || (firstCenter == secondCenter && firstTriangleIndex < secondTriangleIndex);
        });
    }
    const std::uint32_t leftChild = build(firstTriangle, leftCount, depth + 1);
    const std::uint32_t rightChild = build(firstTriangle + leftCount, triangleCount - leftCount, depth + 1);
    nodes[nodeIndex].triangleCount = 0;
    nodes[nodeIndex].leftChild = leftChild;
    nodes[nodeIndex].rightChild = rightChild;
    return nodeIndex;
}

MotionResult CollisionScene::query(const Motion& motion, bool useBvh) const {
    validateMotion(motion);
    MotionResult result;
    if (sceneTriangles.empty()) {
        return result;
    }
    const auto evaluateTriangle = [&](std::uint32_t triangleIndex, bool needsContact, bool needsClearance) {
        const auto& triangle = sceneTriangles[triangleIndex];
        if (needsContact) {
            ++result.stats.triangleTests;
            Contact candidate = sweepTriangle(motion, triangle, tolerances);
            candidate.triangleIndex = candidate.hit ? triangleIndex : noTriangle;
            if (preferContact(candidate, result.contact)) {
                result.contact = candidate;
            }
        }
        if (needsClearance) {
            ++result.stats.triangleTests;
            Proximity candidate = closestSegment(motion.start, motion.end, triangle);
            candidate.distance -= motion.radius;
            candidate.triangleIndex = triangleIndex;
            if (preferProximity(candidate, result.clearance)) {
                result.clearance = candidate;
            }
        }
    };
    if (!useBvh) {
        for (std::uint32_t triangleIndex = 0; triangleIndex < sceneTriangles.size(); ++triangleIndex) {
            evaluateTriangle(triangleIndex, true, true);
        }
        return result;
    }

    const double motionLength = magnitude(motion.end - motion.start);
    const double sceneExtent = magnitude(sceneBounds.maximum - sceneBounds.minimum);
    const double lengthTolerance = tolerances.absolute + tolerances.relative * std::max({motion.radius, motionLength, sceneExtent});
    const double numericalPadding = 128.0 * std::numeric_limits<double>::epsilon() *
                                    std::max({largestCoordinate(motion.start), largestCoordinate(motion.end),
                                              largestCoordinate(sceneBounds.minimum), largestCoordinate(sceneBounds.maximum),
                                              motion.radius, motionLength, sceneExtent});
    const double contactExpansion = motion.radius + lengthTolerance + numericalPadding;
    struct StackEntry {
        std::uint32_t nodeIndex;
        double firstTime;
        double centerDistance;
        bool contactPossible;
    };
    std::array<StackEntry, maxDepth + 2> pendingNodes;
    std::size_t pendingCount = 0;
    const auto evaluateNode = [&](std::uint32_t nodeIndex) {
        const auto& nodeBounds = nodes[nodeIndex].bounds;
        StackEntry entry{nodeIndex, 0.0, 0.0, false};
        ++result.stats.boxTests;
        entry.contactPossible = intersectBounds(motion, nodeBounds, contactExpansion, result.contact.time, entry.firstTime);
        ++result.stats.boxTests;
        entry.centerDistance = std::sqrt(std::max(0.0, segmentBoxDistanceSquared(motion.start, motion.end, nodeBounds)));
        return entry;
    };
    const auto needsClearance = [&](const StackEntry& entry) {
        return entry.centerDistance <= result.clearance.distance + motion.radius + numericalPadding;
    };
    const auto needsContact = [&](const StackEntry& entry) {
        return entry.contactPossible && entry.firstTime <= result.contact.time;
    };
    pendingNodes[pendingCount++] = evaluateNode(0);
    while (pendingCount != 0) {
        const StackEntry current = pendingNodes[--pendingCount];
        const bool contactRequired = needsContact(current);
        const bool clearanceRequired = needsClearance(current);
        if (!contactRequired && !clearanceRequired) {
            continue;
        }
        const Node& node = nodes[current.nodeIndex];
        if (node.triangleCount != 0) {
            for (std::uint32_t offset = 0; offset < node.triangleCount; ++offset) {
                evaluateTriangle(triangleIndices[node.firstTriangle + offset], contactRequired, clearanceRequired);
            }
            continue;
        }
        const StackEntry leftEntry = evaluateNode(node.leftChild);
        const StackEntry rightEntry = evaluateNode(node.rightChild);
        const bool visitLeft = needsContact(leftEntry) || needsClearance(leftEntry);
        const bool visitRight = needsContact(rightEntry) || needsClearance(rightEntry);
        if (visitLeft && visitRight) {
            const bool leftFirst = leftEntry.centerDistance < rightEntry.centerDistance ||
                                   (leftEntry.centerDistance == rightEntry.centerDistance && leftEntry.firstTime <= rightEntry.firstTime);
            pendingNodes[pendingCount++] = leftFirst ? rightEntry : leftEntry;
            pendingNodes[pendingCount++] = leftFirst ? leftEntry : rightEntry;
        } else if (visitLeft) {
            pendingNodes[pendingCount++] = leftEntry;
        } else if (visitRight) {
            pendingNodes[pendingCount++] = rightEntry;
        }
    }
    return result;
}

PathResult CollisionScene::queryPath(std::span<const Vec3> waypoints, double radius, bool useBvh) const {
    validateRadius(radius);
    if (waypoints.empty()) {
        throw std::invalid_argument("Path requires at least one waypoint");
    }
    PathResult result;
    for (std::size_t waypointIndex = 0; waypointIndex < waypoints.size(); ++waypointIndex) {
        if (!validPosition(waypoints[waypointIndex])) {
            throw std::invalid_argument("Path coordinates must be finite and within 1e60");
        }
        if (waypointIndex != 0) {
            result.length += magnitude(waypoints[waypointIndex] - waypoints[waypointIndex - 1]);
            if (!std::isfinite(result.length)) {
                throw std::invalid_argument("Path length exceeds supported range");
            }
        }
    }
    double traversedLength = 0.0;
    const std::size_t segmentCount = std::max<std::size_t>(1, waypoints.size() - 1);
    for (std::size_t segmentIndex = 0; segmentIndex < segmentCount; ++segmentIndex) {
        const Vec3 start = waypoints[segmentIndex];
        const Vec3 end = waypoints[std::min(segmentIndex + 1, waypoints.size() - 1)];
        const double segmentLength = magnitude(end - start);
        MotionResult segmentResult = query({start, end, radius}, useBvh);
        segmentResult.contact.time = result.length == 0.0 ? 0.0 :
                                     (traversedLength + segmentResult.contact.time * segmentLength) / result.length;
        segmentResult.contact.startsOverlapping = segmentResult.contact.startsOverlapping && traversedLength == 0.0;
        segmentResult.clearance.time = result.length == 0.0 ? 0.0 :
                                       (traversedLength + segmentResult.clearance.time * segmentLength) / result.length;
        if (preferContact(segmentResult.contact, result.contact)) {
            result.contact = segmentResult.contact;
            result.contactSegment = segmentIndex;
        }
        if (preferProximity(segmentResult.clearance, result.clearance)) {
            result.clearance = segmentResult.clearance;
            result.clearanceSegment = segmentIndex;
        }
        result.stats.triangleTests += segmentResult.stats.triangleTests;
        result.stats.boxTests += segmentResult.stats.boxTests;
        traversedLength += segmentLength;
    }
    return result;
}

const std::vector<Triangle>& CollisionScene::triangles() const {
    return sceneTriangles;
}

const Bounds& CollisionScene::bounds() const {
    return sceneBounds;
}

std::size_t CollisionScene::nodeCount() const {
    return nodes.size();
}

std::size_t CollisionScene::memoryBytes() const {
    return sizeof(*this) + sceneTriangles.capacity() * sizeof(Triangle) +
           triangleIndices.capacity() * sizeof(std::uint32_t) + nodes.capacity() * sizeof(Node);
}

BatchExecutor::BatchExecutor(std::size_t requestedThreadCount) {
    if (requestedThreadCount == 0) {
        requestedThreadCount = std::thread::hardware_concurrency();
    }
    const std::size_t boundedThreadCount = std::clamp<std::size_t>(requestedThreadCount, 1, 8);
    workers.reserve(boundedThreadCount - 1);
    try {
        for (std::size_t workerIndex = 1; workerIndex < boundedThreadCount; ++workerIndex) {
            workers.emplace_back([this] { worker(); });
        }
    } catch (...) {
        {
            std::lock_guard stateLock(stateMutex);
            stopping = true;
        }
        startCondition.notify_all();
        for (auto& thread : workers) {
            thread.join();
        }
        throw;
    }
}

BatchExecutor::~BatchExecutor() {
    std::lock_guard dispatchLock(dispatchMutex);
    {
        std::lock_guard stateLock(stateMutex);
        stopping = true;
    }
    startCondition.notify_all();
    for (auto& thread : workers) {
        thread.join();
    }
}

void BatchExecutor::execute(const CollisionScene& scene, std::span<const Motion> motions, std::span<MotionResult> results) {
    std::lock_guard dispatchLock(dispatchMutex);
    if (motions.size() != results.size()) {
        throw std::invalid_argument("Batch input and output sizes must match");
    }
    for (const auto& motion : motions) {
        validateMotion(motion);
    }
    if (motions.empty()) {
        return;
    }
    if (workers.empty()) {
        for (std::size_t motionIndex = 0; motionIndex < motions.size(); ++motionIndex) {
            results[motionIndex] = scene.query(motions[motionIndex]);
        }
        return;
    }
    {
        std::lock_guard stateLock(stateMutex);
        activeScene = &scene;
        activeMotions = motions;
        activeResults = results;
        nextIndex.store(0, std::memory_order_relaxed);
        pendingWorkers = workers.size();
        ++generation;
    }
    startCondition.notify_all();
    process();
    std::unique_lock stateLock(stateMutex);
    doneCondition.wait(stateLock, [this] { return pendingWorkers == 0; });
    activeScene = nullptr;
    activeMotions = {};
    activeResults = {};
}

std::size_t BatchExecutor::threadCount() const {
    return workers.size() + 1;
}

void BatchExecutor::worker() {
    std::size_t observedGeneration = 0;
    std::unique_lock stateLock(stateMutex);
    while (true) {
        startCondition.wait(stateLock, [this, observedGeneration] { return stopping || generation != observedGeneration; });
        if (stopping) {
            return;
        }
        observedGeneration = generation;
        stateLock.unlock();
        process();
        stateLock.lock();
        --pendingWorkers;
        if (pendingWorkers == 0) {
            doneCondition.notify_one();
        }
    }
}

void BatchExecutor::process() {
    constexpr std::size_t chunkSize = 8;
    while (true) {
        const std::size_t firstIndex = nextIndex.fetch_add(chunkSize, std::memory_order_relaxed);
        if (firstIndex >= activeMotions.size()) {
            return;
        }
        const std::size_t lastIndex = std::min(firstIndex + chunkSize, activeMotions.size());
        for (std::size_t motionIndex = firstIndex; motionIndex < lastIndex; ++motionIndex) {
            activeResults[motionIndex] = activeScene->query(activeMotions[motionIndex]);
        }
    }
}

}
