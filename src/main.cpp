#include "collision.hpp"

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#define NOGDI
#define NOUSER
#include <windows.h>
#endif

#include <raylib.h>
#include <raymath.h>
#include <rlgl.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>

namespace {

using sweep::Vec3;
constexpr double pi = 3.14159265358979323846;
constexpr double frameSeconds = 0.1;
constexpr std::size_t candidateCount = 49;
constexpr Color safeColor{63, 185, 138, 255};
constexpr Color closeColor{234, 171, 67, 255};
constexpr Color collisionColor{232, 91, 87, 255};

struct Options {
    std::filesystem::path modelPath;
    std::filesystem::path capturePath;
    std::filesystem::path exportPath;
    std::optional<Vec3> start;
    std::optional<Vec3> target;
    std::optional<double> radius;
    std::optional<double> speed;
    std::vector<Vec3> waypoints;
    std::size_t threadCount = 0;
    bool headless = false;
    bool resizeCheck = false;
    bool play = false;
    int captureFrames = 8;
};

double parseNumber(const char* text) {
    std::size_t consumed = 0;
    const std::string value(text);
    const double number = std::stod(value, &consumed);
    if (consumed != value.size() || !std::isfinite(number)) throw std::runtime_error("Expected a finite number: " + value);
    return number;
}

Options parseOptions(int argumentCount, char** arguments) {
    Options options;
    for (int argumentIndex = 1; argumentIndex < argumentCount; ++argumentIndex) {
        const std::string argument = arguments[argumentIndex];
        const auto nextValue = [&]() -> const char* {
            if (++argumentIndex >= argumentCount) throw std::runtime_error("Missing value for " + argument);
            return arguments[argumentIndex];
        };
        const auto nextVector = [&]() {
            const double positionX = parseNumber(nextValue());
            const double positionY = parseNumber(nextValue());
            const double positionZ = parseNumber(nextValue());
            return Vec3(positionX, positionY, positionZ);
        };
        if (argument == "--model") options.modelPath = nextValue();
        else if (argument == "--capture") options.capturePath = nextValue();
        else if (argument == "--export") options.exportPath = nextValue();
        else if (argument == "--start") options.start = nextVector();
        else if (argument == "--target") options.target = nextVector();
        else if (argument == "--waypoint") options.waypoints.push_back(nextVector());
        else if (argument == "--radius") options.radius = parseNumber(nextValue());
        else if (argument == "--speed") options.speed = parseNumber(nextValue());
        else if (argument == "--threads") {
            const double count = parseNumber(nextValue());
            if (count < 1 || count > 8 || count != std::floor(count)) throw std::runtime_error("Thread count must be 1 through 8.");
            options.threadCount = static_cast<std::size_t>(count);
        } else if (argument == "--frames") {
            const double count = parseNumber(nextValue());
            if (count < 8 || count > 3600 || count != std::floor(count)) throw std::runtime_error("Capture frames must be 8 through 3600.");
            options.captureFrames = static_cast<int>(count);
        } else if (argument == "--headless") options.headless = true;
        else if (argument == "--resize-check") options.resizeCheck = true;
        else if (argument == "--play") options.play = true;
        else if (argument == "--help") {
            std::cout << "sweep_demo [--model scene.obj] [--start x y z] [--target x y z]\n"
                         "           [--waypoint x y z] [--radius r] [--speed unitsPerSecond]\n"
                         "           [--threads 1..8] [--export queries.csv] [--headless]\n"
                         "           [--capture view.png] [--resize-check] [--play] [--frames count]\n\n"
                         "Left drag: destination. Right drag: robot. Middle drag: orbit.\n"
                         "Wheel: speed. Ctrl + wheel: radius. Shift + wheel: zoom.\n"
                         "Shift + left click: waypoint. Backspace: remove waypoint.\n"
                         "Double left click or Space: run/pause. R or double middle click: reset.\n"
                         "E: export current queries. Drop an OBJ: load environment.\n";
            std::exit(0);
        } else throw std::runtime_error("Unknown argument: " + argument);
    }
    if (options.radius && *options.radius <= 0) throw std::runtime_error("Viewer radius must be positive.");
    if (options.speed && *options.speed <= 0) throw std::runtime_error("Speed must be positive.");
    if (options.headless && options.exportPath.empty()) throw std::runtime_error("--headless requires --export.");
    if (options.headless && (!options.capturePath.empty() || options.play)) throw std::runtime_error("Capture and playback require the viewer.");
    return options;
}

struct World {
    sweep::CollisionScene scene;
    sweep::BatchExecutor executor;
    Vec3 center;
    double sceneScale;
    Vec3 robotPosition;
    Vec3 destination;
    Vec3 initialPosition;
    Vec3 initialDestination;
    std::vector<Vec3> waypoints;
    std::vector<Vec3> initialWaypoints;
    double radius;
    double speed;
    double initialRadius;
    double initialSpeed;
    double warningMargin;
    double queryMilliseconds = 0;
    std::array<sweep::Motion, candidateCount> motions;
    std::array<sweep::MotionResult, candidateCount> results;
    std::vector<Vec3> stepPath;
    sweep::PathResult stepResult;
    sweep::Contact lastContact;
    bool playing = false;
    bool stoppedForClearance = false;

    explicit World(const std::filesystem::path& modelPath, const Options& options)
        : scene(sweep::loadObj(modelPath)), executor(options.threadCount),
          center((scene.bounds().minimum + scene.bounds().maximum) * 0.5),
          sceneScale(std::max({scene.bounds().maximum.x - scene.bounds().minimum.x,
              scene.bounds().maximum.y - scene.bounds().minimum.y,
              scene.bounds().maximum.z - scene.bounds().minimum.z}) * 0.5) {
        if (!(sceneScale > 1e-12) || !std::isfinite(sceneScale)) throw std::runtime_error("Model extent is unsuitable for the viewer.");
        const double floorHeight = scene.bounds().minimum.y + sceneScale * 0.015;
        radius = options.radius.value_or(sceneScale / 30.0);
        robotPosition = options.start.value_or(Vec3(center.x, floorHeight + sceneScale * 0.075,
            center.z + sceneScale * 0.5));
        destination = options.target.value_or(Vec3(center.x, robotPosition.y, center.z - sceneScale * 0.5));
        speed = options.speed.value_or(sceneScale * (80.0 / 12.0));
        if (radius > sceneScale * 10 || speed > sceneScale * 1e6) throw std::runtime_error("Viewer radius or speed exceeds supported scene-relative limits.");
        initialPosition = robotPosition;
        initialDestination = destination;
        initialRadius = radius;
        initialSpeed = speed;
        warningMargin = sceneScale / 48.0;
        waypoints = options.waypoints;
        initialWaypoints = waypoints;
        for (const auto position : route()) {
            if (glm::length((position - center) / sceneScale) > 1e6) throw std::runtime_error("Viewer coordinates exceed the scene-relative display limit.");
        }
        playing = options.play;
        rebuild();
    }

    Vector3 displayPosition(Vec3 position) const {
        const Vec3 relative = (position - center) / sceneScale;
        return {static_cast<float>(relative.x), static_cast<float>(relative.y), static_cast<float>(relative.z)};
    }

    Vec3 worldPosition(Vector3 position) const {
        return center + Vec3(position.x, position.y, position.z) * sceneScale;
    }

    std::vector<Vec3> route() const {
        std::vector<Vec3> points{robotPosition};
        points.insert(points.end(), waypoints.begin(), waypoints.end());
        points.push_back(destination);
        return points;
    }

    double movementMargin(double motionLength = 0) const {
        const Vec3 coordinateSize = glm::max(glm::abs(robotPosition), glm::max(glm::abs(scene.bounds().minimum), glm::abs(scene.bounds().maximum)));
        const double coordinateScale = std::max({coordinateSize.x, coordinateSize.y, coordinateSize.z});
        const double maximumTravel = std::max(motionLength, std::max(speed, sceneScale * 500) * frameSeconds);
        return 1e-8 + std::max({sceneScale * 1e-6, maximumTravel * 1e-8,
            coordinateScale * 256 * std::numeric_limits<double>::epsilon()});
    }

    void moveRobot(Vec3 requestedPosition) {
        const double margin = movementMargin(glm::length(requestedPosition - robotPosition));
        const auto placement = scene.query({requestedPosition, requestedPosition, radius});
        const auto movement = scene.query({robotPosition, requestedPosition, radius});
        if (movement.contact.startsOverlapping) {
            if (placement.clearance.distance >= margin) robotPosition = requestedPosition;
        } else if (movement.contact.hit || placement.clearance.distance < margin * 0.5) {
            const auto stopped = scene.query({robotPosition, requestedPosition, radius + margin});
            if (stopped.contact.hit) robotPosition = stopped.contact.center;
        } else robotPosition = requestedPosition;
        playing = false;
    }

    void resizeRobot(double requestedRadius) {
        if (requestedRadius <= radius) { radius = requestedRadius; return; }
        const auto placement = scene.query({robotPosition, robotPosition, 0});
        const double availableRadius = placement.clearance.distance - movementMargin();
        radius = std::max(radius, std::min(requestedRadius, availableRadius));
    }

    void rebuild() {
        const auto queryStart = std::chrono::steady_clock::now();
        stoppedForClearance = false;
        const auto points = route();
        stepPath.clear();
        stepPath.push_back(robotPosition);
        double remainingDistance = speed * frameSeconds;
        for (std::size_t pointIndex = 1; pointIndex < points.size(); ++pointIndex) {
            const Vec3 offset = points[pointIndex] - points[pointIndex - 1];
            const double distance = glm::length(offset);
            if (distance <= remainingDistance) {
                stepPath.push_back(points[pointIndex]);
                remainingDistance -= distance;
            } else {
                stepPath.push_back(points[pointIndex - 1] + offset * (remainingDistance / distance));
                break;
            }
        }
        if (stepPath.size() == 1) stepPath.push_back(robotPosition);
        Vec3 direction(0);
        for (std::size_t pointIndex = 1; pointIndex < points.size(); ++pointIndex) {
            direction = points[pointIndex] - robotPosition;
            if (glm::length(direction) > sceneScale * 1e-12) break;
        }
        const double targetDistance = glm::length(direction);
        direction = targetDistance > 0 ? direction / targetDistance : Vec3(0, 0, -1);
        const double travelDistance = std::min(speed * frameSeconds, targetDistance);
        for (std::size_t candidateIndex = 0; candidateIndex < candidateCount; ++candidateIndex) {
            const double angle = (static_cast<double>(candidateIndex) / (candidateCount - 1) - 0.5) * pi * 0.52;
            const Vec3 candidateDirection(direction.x * std::cos(angle) - direction.z * std::sin(angle),
                direction.y, direction.x * std::sin(angle) + direction.z * std::cos(angle));
            motions[candidateIndex] = {robotPosition, robotPosition + candidateDirection * travelDistance, radius};
        }
        executor.execute(scene, motions, results);
        stepResult = scene.queryPath(stepPath, radius);
        queryMilliseconds = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - queryStart).count();
    }

    void reset() {
        robotPosition = initialPosition;
        destination = initialDestination;
        radius = initialRadius;
        speed = initialSpeed;
        waypoints = initialWaypoints;
        playing = false;
        lastContact = {};
        rebuild();
    }

    void advance() {
        const double margin = movementMargin(stepResult.length);
        const auto placement = scene.query({stepPath.back(), stepPath.back(), radius});
        if (stepResult.contact.hit || placement.clearance.distance < margin * 0.5) {
            const auto stopped = scene.queryPath(stepPath, radius + margin);
            if (!stopped.contact.hit) { playing = false; return; }
            robotPosition = stopped.contact.center;
            const std::size_t consumed = std::min(stopped.contactSegment, waypoints.size());
            waypoints.erase(waypoints.begin(), waypoints.begin() + static_cast<std::ptrdiff_t>(consumed));
            while (!waypoints.empty() && glm::length(waypoints.front() - robotPosition) < sceneScale * 1e-9) waypoints.erase(waypoints.begin());
            lastContact = stepResult.contact;
            stoppedForClearance = true;
            playing = false;
            return;
        }
        robotPosition = stepPath.back();
        const std::size_t completedCorners = stepPath.size() > 2 ? stepPath.size() - 2 : 0;
        const std::size_t consumed = std::min(completedCorners, waypoints.size());
        waypoints.erase(waypoints.begin(), waypoints.begin() + static_cast<std::ptrdiff_t>(consumed));
        while (!waypoints.empty() && glm::length(waypoints.front() - robotPosition) < sceneScale * 1e-9) waypoints.erase(waypoints.begin());
        if (glm::length(destination - robotPosition) < sceneScale * 1e-9 && waypoints.empty()) playing = false;
        rebuild();
    }

    void togglePlayback() {
        if (!playing && stoppedForClearance) { lastContact = {}; rebuild(); }
        playing = !playing;
    }
};

Color resultColor(const sweep::Contact& contact, double clearance, double margin) {
    return contact.hit ? collisionColor : clearance <= margin ? closeColor : safeColor;
}

Mesh makeEnvironmentMesh(const World& world, bool dark) {
    std::vector<float> vertices;
    std::vector<unsigned char> colors;
    vertices.reserve(world.scene.triangles().size() * 9);
    colors.reserve(world.scene.triangles().size() * 12);
    const Vec3 lightDirection = glm::normalize(Vec3(-0.6, 1, 0.5));
    for (const auto& triangle : world.scene.triangles()) {
        if (dark && std::min({triangle.first.y, triangle.second.y, triangle.third.y}) > world.robotPosition.y) continue;
        const Vec3 normalVector = glm::cross(triangle.second - triangle.first, triangle.third - triangle.first);
        const double normalLength = glm::length(normalVector);
        if (normalLength == 0) continue;
        const Vec3 normal = normalVector / normalLength;
        const double brightness = 0.62 + 0.31 * std::max(0.0, glm::dot(normal, lightDirection)) + 0.07 * std::max(0.0, normal.y);
        const bool isFloor = std::abs(normal.y) > 0.99 &&
            std::abs(triangle.first.y - world.scene.bounds().minimum.y) < world.sceneScale * 0.025;
        const Color baseColor = dark ? (isFloor ? Color{23, 34, 42, 255} : Color{62, 81, 91, 255})
            : (isFloor ? Color{204, 212, 211, 255} : Color{236, 231, 217, 255});
        const Color color{static_cast<unsigned char>(baseColor.r * brightness), static_cast<unsigned char>(baseColor.g * brightness),
            static_cast<unsigned char>(baseColor.b * brightness), 255};
        for (const auto position : {triangle.first, triangle.second, triangle.third}) {
            const Vector3 displayPoint = world.displayPosition(position);
            vertices.insert(vertices.end(), {displayPoint.x, displayPoint.y, displayPoint.z});
            colors.insert(colors.end(), {color.r, color.g, color.b, color.a});
        }
    }
    if (vertices.size() > static_cast<std::size_t>(std::numeric_limits<int>::max()) / sizeof(float)) throw std::runtime_error("Display mesh is too large.");
    Mesh mesh{};
    mesh.vertexCount = static_cast<int>(vertices.size() / 3);
    mesh.triangleCount = mesh.vertexCount / 3;
    if (mesh.vertexCount == 0) return mesh;
    mesh.vertices = static_cast<float*>(MemAlloc(static_cast<unsigned int>(vertices.size() * sizeof(float))));
    mesh.colors = static_cast<unsigned char*>(MemAlloc(static_cast<unsigned int>(colors.size())));
    if (!mesh.vertices || !mesh.colors) {
        MemFree(mesh.vertices);
        MemFree(mesh.colors);
        throw std::runtime_error("Display allocation failed.");
    }
    std::memcpy(mesh.vertices, vertices.data(), vertices.size() * sizeof(float));
    std::memcpy(mesh.colors, colors.data(), colors.size());
    UploadMesh(&mesh, false);
    return mesh;
}

struct Display {
    Mesh solidMesh{};
    Mesh planMesh{};
    Material material = LoadMaterialDefault();
    RenderTexture2D solidTexture{};
    RenderTexture2D planTexture{};
    int paneWidth = 0;
    int paneHeight = 0;
    double azimuth = 0.62;
    double elevation = 0.72;
    double distance = 3.8;

    ~Display() {
        if (solidMesh.vertexCount) UnloadMesh(solidMesh);
        if (planMesh.vertexCount) UnloadMesh(planMesh);
        if (solidTexture.id) UnloadRenderTexture(solidTexture);
        if (planTexture.id) UnloadRenderTexture(planTexture);
        UnloadMaterial(material);
    }

    void setScene(const World& world) {
        Mesh nextSolid = makeEnvironmentMesh(world, false);
        Mesh nextPlan{};
        try { nextPlan = makeEnvironmentMesh(world, true); }
        catch (...) { if (nextSolid.vertexCount) UnloadMesh(nextSolid); throw; }
        if (solidMesh.vertexCount) UnloadMesh(solidMesh);
        if (planMesh.vertexCount) UnloadMesh(planMesh);
        solidMesh = nextSolid;
        planMesh = nextPlan;
    }

    void resize(int width, int height) {
        const int nextWidth = std::max(1, (width - 2) / 2);
        const int nextHeight = std::max(1, height);
        if (nextWidth == paneWidth && nextHeight == paneHeight) return;
        const auto nextSolid = LoadRenderTexture(nextWidth, nextHeight);
        const auto nextPlan = LoadRenderTexture(nextWidth, nextHeight);
        if (!IsRenderTextureValid(nextSolid) || !IsRenderTextureValid(nextPlan)) {
            if (nextSolid.id) UnloadRenderTexture(nextSolid);
            if (nextPlan.id) UnloadRenderTexture(nextPlan);
            throw std::runtime_error("Render target allocation failed.");
        }
        if (solidTexture.id) UnloadRenderTexture(solidTexture);
        if (planTexture.id) UnloadRenderTexture(planTexture);
        solidTexture = nextSolid;
        planTexture = nextPlan;
        paneWidth = nextWidth;
        paneHeight = nextHeight;
    }

    Camera3D camera(bool plan) const {
        const double aspect = static_cast<double>(paneWidth) / paneHeight;
        if (plan) return {{0, 4, 0}, {0, 0, 0}, {0, 0, -1}, static_cast<float>(2.45 * std::max(1.0, 1.0 / aspect)), CAMERA_ORTHOGRAPHIC};
        const float fieldOfView = static_cast<float>(2 * std::atan(std::tan(pi / 8) * std::max(1.0, 0.9 / aspect)) * 180 / pi);
        return {{static_cast<float>(distance * std::sin(azimuth) * std::cos(elevation)),
                 static_cast<float>(distance * std::sin(elevation)),
                 static_cast<float>(distance * std::cos(azimuth) * std::cos(elevation))},
                {0, -0.05f, 0}, {0, 1, 0}, fieldOfView, CAMERA_PERSPECTIVE};
    }
};

std::optional<Vec3> pickPosition(const World& world, const Display& display, int pane) {
    Vector2 mousePosition = GetMousePosition();
    if (pane == 1) mousePosition.x -= static_cast<float>(display.paneWidth + 2);
    const Ray pointerRay = GetScreenToWorldRayEx(mousePosition, display.camera(pane == 1), display.paneWidth, display.paneHeight);
    const double displayHeight = (world.robotPosition.y - world.center.y) / world.sceneScale;
    if (std::abs(pointerRay.direction.y) < 1e-8) return std::nullopt;
    const double parameter = (displayHeight - pointerRay.position.y) / pointerRay.direction.y;
    if (parameter < 0) return std::nullopt;
    const Vector3 pickedPoint = Vector3Add(pointerRay.position, Vector3Scale(pointerRay.direction, static_cast<float>(parameter)));
    const Vec3 position = world.worldPosition(pickedPoint);
    if (glm::length(position - world.center) > world.sceneScale * 8) return std::nullopt;
    return position;
}

void updateTitle(const World& world) {
    std::ostringstream title;
    title << "Sweep | " << std::fixed << std::setprecision(1) << world.speed << " units/s | r " << std::setprecision(2)
          << world.radius << " | " << (world.stepResult.contact.hit ? "contact" : world.stepResult.clearance.distance <= world.warningMargin ? "close" : "clear")
          << " | " << std::setprecision(1) << world.queryMilliseconds << " ms";
    SetWindowTitle(title.str().c_str());
}

struct InputState {
    int dragPane = 0;
    double lastLeftClick = -1;
    double lastMiddleClick = -1;
    Vector2 lastLeftPosition{-1000, -1000};
};

void handleInput(World& world, Display& display, InputState& input) {
    bool changed = false;
    const bool shiftDown = IsKeyDown(KEY_LEFT_SHIFT) || IsKeyDown(KEY_RIGHT_SHIFT);
    const bool controlDown = IsKeyDown(KEY_LEFT_CONTROL) || IsKeyDown(KEY_RIGHT_CONTROL);
    if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT) || IsMouseButtonPressed(MOUSE_BUTTON_RIGHT)) {
        input.dragPane = GetMouseX() < display.paneWidth ? 0 : 1;
    }
    if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
        const double clickTime = GetTime();
        const Vector2 clickPosition = GetMousePosition();
        if (clickTime - input.lastLeftClick < 0.28 && Vector2Distance(clickPosition, input.lastLeftPosition) < 8 && !shiftDown) {
            world.togglePlayback();
            input.lastLeftClick = -1;
        } else {
            input.lastLeftClick = clickTime;
            input.lastLeftPosition = clickPosition;
            if (const auto position = pickPosition(world, display, input.dragPane)) {
                if (shiftDown) world.waypoints.push_back(*position);
                else world.destination = *position;
                changed = true;
            }
        }
    }
    const Vector2 delta = GetMouseDelta();
    if (IsMouseButtonDown(MOUSE_BUTTON_LEFT) && !shiftDown && (delta.x != 0 || delta.y != 0)) {
        if (const auto position = pickPosition(world, display, input.dragPane)) {
            world.destination = *position;
            world.playing = false;
            changed = true;
        }
    }
    if (IsMouseButtonDown(MOUSE_BUTTON_RIGHT)) {
        if (const auto position = pickPosition(world, display, input.dragPane)) {
            world.moveRobot(*position);
            changed = true;
        }
    }
    if (IsMouseButtonDown(MOUSE_BUTTON_MIDDLE)) {
        display.azimuth -= delta.x * 0.006;
        display.elevation = std::clamp(display.elevation + delta.y * 0.004, 0.2, 1.48);
    }
    if (IsMouseButtonPressed(MOUSE_BUTTON_MIDDLE)) {
        const double clickTime = GetTime();
        if (clickTime - input.lastMiddleClick < 0.28) { world.reset(); display.azimuth = 0.62; display.elevation = 0.72; display.distance = 3.8; updateTitle(world); }
        input.lastMiddleClick = clickTime;
    }
    const double wheel = GetMouseWheelMove();
    if (wheel != 0) {
        if (shiftDown) display.distance = std::clamp(display.distance * std::exp(-wheel * 0.1), 1.4, 8.0);
        else if (controlDown) { world.resizeRobot(std::clamp(world.radius * std::exp(wheel * 0.1), world.sceneScale * .002, world.sceneScale * .15)); changed = true; }
        else { world.speed = std::clamp(world.speed * std::exp(wheel * 0.15), world.sceneScale * .025, world.sceneScale * 500); changed = true; }
    }
    if (IsKeyPressed(KEY_SPACE)) world.togglePlayback();
    if (IsKeyPressed(KEY_BACKSPACE) && !world.waypoints.empty()) { world.waypoints.pop_back(); changed = true; }
    if (IsKeyPressed(KEY_R)) { world.reset(); updateTitle(world); }
    if (changed) { world.lastContact = {}; world.rebuild(); updateTitle(world); }
}

void drawPathLine(const World& world, Vec3 start, Vec3 end, float thickness, Color color) {
    if (glm::length(end - start) < world.sceneScale * 1e-10) return;
    DrawCylinderEx(world.displayPosition(start), world.displayPosition(end), thickness, thickness, 6, color);
}

void drawMarkers(const World& world, bool plan) {
    const float displayRadius = static_cast<float>(world.radius / world.sceneScale);
    const auto requestedRoute = world.route();
    const Color guideColor = plan ? Color{109, 131, 144, 160} : Color{99, 120, 125, 130};
    for (std::size_t segmentIndex = 1; segmentIndex < requestedRoute.size(); ++segmentIndex) {
        DrawLine3D(world.displayPosition(requestedRoute[segmentIndex - 1]), world.displayPosition(requestedRoute[segmentIndex]), guideColor);
    }
    for (const auto waypoint : world.waypoints) DrawSphereWires(world.displayPosition(waypoint), displayRadius * 0.35f, 6, 10, guideColor);
    for (std::size_t candidateIndex = 0; candidateIndex < candidateCount; ++candidateIndex) {
        const auto& motion = world.motions[candidateIndex];
        const auto& result = world.results[candidateIndex];
        const Color color = resultColor(result.contact, result.clearance.distance, world.warningMargin);
        const Vec3 stop = result.contact.hit ? result.contact.center : motion.end;
        DrawLine3D(world.displayPosition(motion.start), world.displayPosition(stop), Color{color.r, color.g, color.b, 215});
        DrawSphere(world.displayPosition(motion.end), .003f, Color{color.r, color.g, color.b, 145});
        if (result.contact.hit) {
            DrawLine3D(world.displayPosition(stop), world.displayPosition(motion.end), Color{color.r, color.g, color.b, 35});
            DrawSphere(world.displayPosition(result.contact.position), .004f, collisionColor);
        }
    }
    const Color selectedColor = resultColor(world.stepResult.contact, world.stepResult.clearance.distance, world.warningMargin);
    for (std::size_t segmentIndex = 1; segmentIndex < world.stepPath.size(); ++segmentIndex) {
        const bool contactSegment = world.stepResult.contact.hit && segmentIndex - 1 == world.stepResult.contactSegment;
        const Vec3 end = contactSegment ? world.stepResult.contact.center : world.stepPath[segmentIndex];
        drawPathLine(world, world.stepPath[segmentIndex - 1], end, .0035f, selectedColor);
        if (contactSegment) break;
    }
    const Vector3 frameEnd = world.displayPosition(world.stepPath.back());
    DrawSphereWires(frameEnd, displayRadius, 10, 16, Color{130, 159, 171, 140});
    const Vector3 target = world.displayPosition(world.destination);
    DrawCircle3D(target, displayRadius * .65f, {1, 0, 0}, 90, plan ? Color{220, 235, 238, 245} : Color{50, 91, 101, 255});
    DrawLine3D(Vector3Add(target, {-0.016f, 0, 0}), Vector3Add(target, {0.016f, 0, 0}), guideColor);
    DrawLine3D(Vector3Add(target, {0, 0, -0.016f}), Vector3Add(target, {0, 0, 0.016f}), guideColor);
    const auto& contact = world.lastContact.hit ? world.lastContact : world.stepResult.contact;
    if (contact.hit) {
        DrawSphereWires(world.displayPosition(contact.center), displayRadius, 12, 20, collisionColor);
        DrawSphere(world.displayPosition(contact.position), .012f, {255, 207, 168, 255});
        drawPathLine(world, contact.position, contact.position + contact.normal * (world.sceneScale * .065), .002f, collisionColor);
    }
    const Vector3 robot = world.displayPosition(world.robotPosition);
    DrawSphereEx(robot, displayRadius, 18, 24, plan ? Color{213, 226, 225, 255} : Color{49, 80, 91, 255});
    DrawSphereWires(robot, displayRadius * 1.01f, 8, 16, {103, 178, 181, 255});
    DrawCircle3D(robot, displayRadius * 1.35f, {1, 0, 0}, 90, {94, 158, 166, 160});
}

void render(const World& world, const Display& display) {
    for (const bool plan : {false, true}) {
        BeginTextureMode(plan ? display.planTexture : display.solidTexture);
        ClearBackground(plan ? Color{10, 20, 27, 255} : Color{239, 241, 237, 255});
        rlSetClipPlanes(0.001, 100);
        BeginMode3D(display.camera(plan));
        rlDisableBackfaceCulling();
        const Mesh mesh = plan ? display.planMesh : display.solidMesh;
        if (mesh.vertexCount) DrawMesh(mesh, display.material, MatrixIdentity());
        if (plan) { rlDrawRenderBatchActive(); rlDisableDepthTest(); }
        drawMarkers(world, plan);
        rlDrawRenderBatchActive();
        rlEnableDepthTest();
        EndMode3D();
        EndTextureMode();
    }
    BeginDrawing();
    ClearBackground({43, 55, 61, 255});
    const Rectangle source{0, 0, static_cast<float>(display.paneWidth), -static_cast<float>(display.paneHeight)};
    DrawTextureRec(display.solidTexture.texture, source, {0, 0}, WHITE);
    DrawTexturePro(display.planTexture.texture, source,
        {static_cast<float>(display.paneWidth + 2), 0, static_cast<float>(GetScreenWidth() - display.paneWidth - 2), static_cast<float>(display.paneHeight)}, {0, 0}, 0, WHITE);
    EndDrawing();
}

void writeQueries(const std::filesystem::path& filePath, const World& world) {
    std::ofstream output(filePath);
    if (!output) throw std::runtime_error("Could not create query report: " + filePath.string());
    output.imbue(std::locale::classic());
    output << std::setprecision(17) << "kind,index,startX,startY,startZ,endX,endY,endZ,radius,speed,frameSeconds,hit,startsOverlapping,time,contactX,contactY,contactZ,normalX,normalY,normalZ,triangleIndex,clearance,clearanceTime,triangleTests,boxTests\n";
    const auto writeResult = [&](const char* kind, std::size_t index, Vec3 start, Vec3 end, const sweep::Contact& contact,
                                 const sweep::Proximity& clearance, const sweep::QueryStats& stats) {
        output << kind << ',' << index << ',' << start.x << ',' << start.y << ',' << start.z << ','
            << end.x << ',' << end.y << ',' << end.z << ',' << world.radius << ',' << world.speed << ',' << frameSeconds << ','
            << contact.hit << ',' << contact.startsOverlapping << ',' << contact.time << ',' << contact.position.x << ',' << contact.position.y << ','
            << contact.position.z << ',' << contact.normal.x << ',' << contact.normal.y << ',' << contact.normal.z << ',' << contact.triangleIndex << ','
            << clearance.distance << ',' << clearance.time << ',' << stats.triangleTests << ',' << stats.boxTests << '\n';
    };
    for (std::size_t candidateIndex = 0; candidateIndex < candidateCount; ++candidateIndex) {
        const auto& result = world.results[candidateIndex];
        writeResult("fan", candidateIndex, world.motions[candidateIndex].start, world.motions[candidateIndex].end,
                    result.contact, result.clearance, result.stats);
    }
    writeResult(world.stoppedForClearance ? "last_attempt" : "next_frame", 0, world.stepPath.front(), world.stepPath.back(), world.stepResult.contact, world.stepResult.clearance, world.stepResult.stats);
    const auto points = world.route();
    const auto routeResult = world.scene.queryPath(points, world.radius);
    writeResult("remaining_route", 0, points.front(), points.back(), routeResult.contact, routeResult.clearance, routeResult.stats);
    for (std::size_t segmentIndex = 1; segmentIndex < points.size(); ++segmentIndex) {
        const auto result = world.scene.query({points[segmentIndex - 1], points[segmentIndex], world.radius});
        writeResult("route_segment", segmentIndex - 1, points[segmentIndex - 1], points[segmentIndex], result.contact, result.clearance, result.stats);
    }
    output.close();
    if (!output) throw std::runtime_error("Could not finish query report.");
    std::cout << "Exported queries to " << filePath << '\n';
}

int runViewer(std::unique_ptr<World> world, const Options& options, const std::filesystem::path& applicationPath) {
    SetConfigFlags(FLAG_WINDOW_RESIZABLE | FLAG_MSAA_4X_HINT | FLAG_VSYNC_HINT);
    InitWindow(1600, 900, "Sweep");
    if (!IsWindowReady()) throw std::runtime_error("Could not open an OpenGL 3.3 window.");
    struct WindowLifetime { ~WindowLifetime() { CloseWindow(); } } windowLifetime;
    SetWindowMinSize(760, 460);
    SetTargetFPS(60);
    Display display;
    display.resize(GetScreenWidth(), GetScreenHeight());
    display.setScene(*world);
    InputState input;
    double accumulator = 0;
    int frameCount = 0;
    updateTitle(*world);
    while (!WindowShouldClose()) {
        display.resize(GetScreenWidth(), GetScreenHeight());
        handleInput(*world, display, input);
        if (IsFileDropped()) {
            const auto droppedFiles = LoadDroppedFiles();
            try {
                if (droppedFiles.count != 1) throw std::runtime_error("Drop one OBJ.");
                Options replacementOptions;
                replacementOptions.threadCount = options.threadCount;
                auto replacement = std::make_unique<World>(droppedFiles.paths[0], replacementOptions);
                display.setScene(*replacement);
                world = std::move(replacement);
                updateTitle(*world);
            } catch (const std::exception& error) { std::cerr << error.what() << '\n'; SetWindowTitle("Sweep | OBJ could not be loaded"); }
            UnloadDroppedFiles(droppedFiles);
        }
        if (IsKeyPressed(KEY_E)) {
            try {
                const auto exportDirectory = applicationPath / "queries";
                std::filesystem::create_directories(exportDirectory);
                const auto timestamp = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
                writeQueries(exportDirectory / ("queries-" + std::to_string(timestamp) + ".csv"), *world);
                updateTitle(*world);
            } catch (const std::exception& error) { std::cerr << error.what() << '\n'; SetWindowTitle("Sweep | export failed"); }
        }
        if (world->playing) {
            accumulator += std::min(static_cast<double>(GetFrameTime()), 0.25);
            int substepCount = 0;
            while (accumulator >= frameSeconds && world->playing && substepCount < 4) {
                accumulator -= frameSeconds;
                world->advance();
                ++substepCount;
                updateTitle(*world);
            }
        } else accumulator = 0;
        if (options.resizeCheck && frameCount == 3) SetWindowSize(1200, 760);
        render(*world, display);
        if (++frameCount == options.captureFrames && !options.capturePath.empty()) {
            Image screenshot{rlReadScreenPixels(GetRenderWidth(), GetRenderHeight()), GetRenderWidth(), GetRenderHeight(), 1, PIXELFORMAT_UNCOMPRESSED_R8G8B8A8};
            const bool saved = ExportImage(screenshot, options.capturePath.string().c_str());
            UnloadImage(screenshot);
            if (!saved) throw std::runtime_error("Capture failed.");
            if (!options.exportPath.empty()) writeQueries(options.exportPath, *world);
            break;
        }
    }
    return 0;
}

}

int main(int argumentCount, char** arguments) {
#ifdef _WIN32
    const HANDLE outputHandle = GetStdHandle(STD_OUTPUT_HANDLE);
    if (argumentCount > 1 && (outputHandle == nullptr || outputHandle == INVALID_HANDLE_VALUE) && AttachConsole(ATTACH_PARENT_PROCESS)) {
        FILE* consoleStream = nullptr;
        freopen_s(&consoleStream, "CONOUT$", "w", stdout);
        freopen_s(&consoleStream, "CONOUT$", "w", stderr);
    }
#endif
    try {
        const Options options = parseOptions(argumentCount, arguments);
        SetTraceLogLevel(LOG_WARNING);
        const std::filesystem::path applicationPath = GetApplicationDirectory();
        const auto modelPath = options.modelPath.empty() ? applicationPath / "assets" / "factory.obj" : options.modelPath;
        auto world = std::make_unique<World>(modelPath, options);
        std::cout << world->scene.triangles().size() << " triangles, " << world->scene.nodeCount() << " nodes, " << world->executor.threadCount()
                  << " threads, " << world->queryMilliseconds << " ms for " << candidateCount << " candidates and selected path\n"
                  << "Selected: hit=" << world->stepResult.contact.hit << " time=" << world->stepResult.contact.time
                  << " clearance=" << world->stepResult.clearance.distance << '\n';
        if (!options.exportPath.empty()) writeQueries(options.exportPath, *world);
        if (options.headless) return 0;
        return runViewer(std::move(world), options, applicationPath);
    } catch (const std::exception& error) { std::cerr << "sweep: " << error.what() << '\n'; return 1; }
}
