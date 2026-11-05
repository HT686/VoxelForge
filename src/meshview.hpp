#pragma once
// The 3D view of a mesh: orbit camera, marked triangles, rim lines, and picking.
#define GL_GLEXT_PROTOTYPES
#include <GL/gl.h>
#include <GL/glext.h>
#include <cstdint>
#include <vector>
#include "mesh.hpp"

struct LineVertex { V3 p; float r, g, b; };
class MeshView {
public:
    enum Flag : uint8_t { Plain = 0, Marked = 1, Filling = 2 };
    void init();
    void setMesh(const Mesh& m, bool resetCamera);
    // Draws only the vertices, as a point cloud. spacingMm: how far apart they lie, so that a surface stays closed when zoomed in.
    void setPointCloud(const Mesh& m, bool resetCamera, float spacingMm);
    // Which direction of the mesh counts as up on screen. Meshes for export already have Z up; a raw
    // scan is still in camera coordinates and would otherwise be seen from the side.
    void setUpDirection(V3 up);
    void setFlags(const std::vector<uint8_t>& oneBytePerTriangle);   // empty: nothing marked
    void setLines(const std::vector<LineVertex>& pairsOfPoints);
    bool hasMesh() const { return count_ > 0; }
    bool hasColor() const { return hasColor_; }
    bool showColor = true;
    bool showFloor = true;
    void rotate(float dx, float dy) { yaw_ += dx*0.008f; pitch_ = std::clamp(pitch_+dy*0.008f, -1.5f, 1.5f); depthValid_ = false; pickValid_ = false; }
    void zoom(float wheel) { zoom_ = std::clamp(zoom_*std::pow(0.9f, wheel), 0.01f, 10.f); depthValid_ = false; pickValid_ = false; }
    void pan(float dx, float dy);
    void focus(V3 center, float radius);
    // Follows a model that moved away, for example after turning it. A chosen spot on it stays.
    void keepInSight(const Mesh& m);
    GLuint render(int w, int h);
    // Everything below refers to the last render, with pixels counted from the top left of the image.
    bool project(V3 p, float& x, float& y, float& depth) const;
    long pickTriangle(int x, int y);                  // -1: nothing there
    void ray(float x, float y, V3& origin, V3& direction) const;
    void prepareDepth();                              // call once before isVisible
    bool isVisible(float x, float y, float depth, float toleranceMm) const;
    const float* rotation() const { return R_; }
    float radius() const { return radius_; }
    V3 center() const { return center_; }
private:
    GLuint prog_ = 0, lineProg_ = 0, pickProg_ = 0, backProg_ = 0, floorProg_ = 0, vao_ = 0, emptyVao_ = 0, vbo_ = 0, ibo_ = 0, fboMs_ = 0, msColor_ = 0, msDepth_ = 0, fbo_ = 0, color_ = 0, pickFbo_ = 0, ids_ = 0, depth_ = 0, flagBuf_ = 0, flagTex_ = 0, lineVao_ = 0, lineVbo_ = 0;
    void renderPick();
    bool pickValid_ = false;
    float floorZ_ = 0, floorR_ = 100;   // in Z-up coordinates, around the model
    V3 floorCenter_;
    bool hasColor_ = false, hasFlags_ = false, pointMode_ = false; float pointSpacing_ = 0.25f;
    GLsizei count_ = 0, lineCount_ = 0; int w_ = 0, h_ = 0; V3 center_; float radius_ = 100, yaw_ = 0, pitch_ = 0, zoom_ = 1, panX_ = 0, panY_ = 0;
    float model_[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};   // mesh -> Z up
    float R_[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1}, mvp_[16] = {0}, t_[3] = {0, 0, 0}, near_ = 1, far_ = 1000, ft_ = 1;
    std::vector<float> depthPixels_; bool depthValid_ = false;
    float linear(float d) const { return 2*far_*near_/(far_+near_-(2*d-1)*(far_-near_)); }
};
