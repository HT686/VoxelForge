#include <algorithm>
#include <cmath>
#include <cstdio>
#include "meshview.hpp"

namespace {
GLuint compile(GLenum type, const char* src) {
    GLuint s = glCreateShader(type); glShaderSource(s, 1, &src, nullptr); glCompileShader(s);
    GLint ok = 0; glGetShaderiv(s, GL_COMPILE_STATUS, &ok);
    if (!ok) { char log[2048]; glGetShaderInfoLog(s, sizeof log, nullptr, log); fprintf(stderr, "Shader: %s\n", log); }
    return s;
}
GLuint link(std::initializer_list<GLuint> shaders) {
    GLuint p = glCreateProgram(); for (GLuint s : shaders) glAttachShader(p, s);
    glLinkProgram(p); GLint ok = 0; glGetProgramiv(p, GL_LINK_STATUS, &ok);
    if (!ok) { char log[2048]; glGetProgramInfoLog(p, sizeof log, nullptr, log); fprintf(stderr, "Shader: %s\n", log); }
    for (GLuint s : shaders) glDeleteShader(s);
    return p;
}
}
void MeshView::init() {
    const char* vs = R"(#version 330 core
layout(location=0) in vec3 p; layout(location=1) in vec3 n; layout(location=2) in vec3 col;
uniform mat4 mvp; uniform mat3 nm; uniform mat4 view; out vec3 N; out vec3 C; out vec3 VP;
void main(){ N=nm*n; C=col; VP=(view*vec4(p,1.0)).xyz; gl_Position=mvp*vec4(p,1.0); })";
    const char* fs = R"(#version 330 core
in vec3 N; in vec3 C; in vec3 VP; uniform int useColor; uniform int hasFlags; uniform usamplerBuffer flags;
layout(location=0) out vec4 c;
void main(){
  vec3 n=normalize(N); if(!gl_FrontFacing) n=-n; vec3 v=normalize(-VP);
  // A soft studio light: wide key from above left, cool fill, hemisphere ambient and a thin rim.
  vec3 keyL=normalize(vec3(-0.35,0.65,0.68)), fillL=normalize(vec3(0.7,-0.1,0.45));
  float key=max(dot(n,keyL),0.0), fill=max(dot(n,fillL),0.0), hemi=0.5+0.5*n.y;
  vec3 ambient=mix(vec3(0.20,0.21,0.25),vec3(0.46,0.50,0.58),hemi);
  float fres=pow(1.0-max(dot(n,v),0.0),3.0);
  vec3 h=normalize(keyL+v); float spec=pow(max(dot(n,h),0.0),40.0);
  vec3 base=gl_FrontFacing?vec3(0.60,0.76,0.95):vec3(0.32,0.38,0.46);
  bool painted=useColor==1;
  if(painted) base=gl_FrontFacing?C:C*0.45;
  bool flagged=false;
  if(hasFlags==1){ uint m=texelFetch(flags,gl_PrimitiveID).r; if(m==1u){base=vec3(0.97,0.27,0.27);flagged=true;} else if(m==2u){base=vec3(0.97,0.60,0.20);flagged=true;} }
  vec3 lit=painted&&!flagged ? base*(ambient*1.15+vec3(0.62)*key+vec3(0.14,0.15,0.18)*fill)+vec3(0.10)*spec
                             : base*(ambient*0.95+vec3(0.78)*key+vec3(0.20,0.24,0.30)*fill)+vec3(0.20)*spec;
  lit+=vec3(0.25,0.30,0.38)*fres*0.6;
  c=vec4(lit,1.0); })";
    const char* pickFs = R"(#version 330 core
layout(location=0) out uint id; void main(){ id=uint(gl_PrimitiveID)+1u; })";
    const char* pickVs = R"(#version 330 core
layout(location=0) in vec3 p; uniform mat4 mvp; void main(){ gl_Position=mvp*vec4(p,1.0); })";
    pickProg_ = link({compile(GL_VERTEX_SHADER, pickVs), compile(GL_FRAGMENT_SHADER, pickFs)});
    // Background: a soft spotlight falling off to the corners.
    const char* backVs = R"(#version 330 core
out vec2 uv; void main(){ vec2 q=vec2((gl_VertexID<<1)&2,gl_VertexID&2); uv=q; gl_Position=vec4(q*2.0-1.0,0.9999,1.0); })";
    const char* backFs = R"(#version 330 core
in vec2 uv; out vec4 c; void main(){ float d=distance(uv,vec2(0.5,0.60)); vec3 a=vec3(0.098,0.122,0.161), b=vec3(0.036,0.046,0.064); c=vec4(mix(a,b,smoothstep(0.0,0.85,d)),1.0); })";
    backProg_ = link({compile(GL_VERTEX_SHADER, backVs), compile(GL_FRAGMENT_SHADER, backFs)});
    // Floor under the model: fading grid and a soft contact shadow.
    const char* floorVs = R"(#version 330 core
uniform mat4 mvp; uniform mat3 up2mesh; uniform vec3 center; uniform float z; uniform float r; out vec2 g;
void main(){ vec2 q=vec2((gl_VertexID<<1)&2,gl_VertexID&2)*2.0-1.0; q*=3.0; g=q*r; vec3 u=vec3(center.xy+g,z); gl_Position=mvp*vec4(up2mesh*u,1.0); })";
    const char* floorFs = R"(#version 330 core
in vec2 g; uniform float r; uniform float step1; out vec4 c;
float grid(vec2 p,float s){ vec2 q=abs(fract(p/s-0.5)-0.5)/fwidth(p/s); return 1.0-min(min(q.x,q.y),1.0); }
void main(){ float d=length(g)/r; float fade=1.0-smoothstep(0.7,2.4,d);
  float a=(grid(g,step1)*0.10+grid(g,step1*5.0)*0.20)*fade;
  float sh=exp(-d*d*2.4)*0.42;
  vec3 col=mix(vec3(0.0),vec3(0.62,0.72,0.86),a/(a+sh+0.0001));
  c=vec4(col,clamp(a+sh,0.0,1.0)); })";
    floorProg_ = link({compile(GL_VERTEX_SHADER, floorVs), compile(GL_FRAGMENT_SHADER, floorFs)});
    prog_ = link({compile(GL_VERTEX_SHADER, vs), compile(GL_FRAGMENT_SHADER, fs)});
    // Lines of a fixed width on screen. They sit slightly in front, so that a rim on the surface shows.
    const char* lvs = R"(#version 330 core
layout(location=0) in vec3 p; layout(location=1) in vec3 col; uniform mat4 mvp; out vec3 vc;
void main(){ vc=col; gl_Position=mvp*vec4(p,1.0); })";
    const char* lgs = R"(#version 330 core
layout(lines) in; layout(triangle_strip, max_vertices=4) out; in vec3 vc[]; out vec3 fc; uniform vec2 viewport; uniform float width;
void main(){
  vec4 a=gl_in[0].gl_Position, b=gl_in[1].gl_Position; if(a.w<=0.0||b.w<=0.0) return;
  vec2 d=(b.xy/b.w-a.xy/a.w)*viewport; float l=length(d); if(l<1e-4) d=vec2(1,0); else d/=l;
  vec2 o=vec2(-d.y,d.x)*width/viewport;
  const float bias=0.0006;
  fc=vc[0]; gl_Position=vec4(a.xy+o*a.w,a.z-bias*a.w,a.w); EmitVertex(); gl_Position=vec4(a.xy-o*a.w,a.z-bias*a.w,a.w); EmitVertex();
  fc=vc[1]; gl_Position=vec4(b.xy+o*b.w,b.z-bias*b.w,b.w); EmitVertex(); gl_Position=vec4(b.xy-o*b.w,b.z-bias*b.w,b.w); EmitVertex();
  EndPrimitive(); })";
    const char* lfs = R"(#version 330 core
in vec3 fc; layout(location=0) out vec4 c; void main(){ c=vec4(fc,1.0); })";
    lineProg_ = link({compile(GL_VERTEX_SHADER, lvs), compile(GL_GEOMETRY_SHADER, lgs), compile(GL_FRAGMENT_SHADER, lfs)});
    glGenVertexArrays(1, &vao_); glGenVertexArrays(1, &emptyVao_); glGenBuffers(1, &vbo_); glGenBuffers(1, &ibo_); glGenFramebuffers(1, &fbo_); glGenFramebuffers(1, &fboMs_); glGenFramebuffers(1, &pickFbo_);
    glGenTextures(1, &color_); glGenTextures(1, &ids_); glGenRenderbuffers(1, &depth_); glGenRenderbuffers(1, &msColor_); glGenRenderbuffers(1, &msDepth_);
    glGenBuffers(1, &flagBuf_); glGenTextures(1, &flagTex_); glGenVertexArrays(1, &lineVao_); glGenBuffers(1, &lineVbo_);
}
void MeshView::setUpDirection(V3 up) {
    const V3 z = normalized(up), x = normalized(cross(std::fabs(z.y) < 0.9f ? V3{0, 1, 0} : V3{1, 0, 0}, z)), y = cross(z, x);
    const float r[9] = {x.x, x.y, x.z, y.x, y.y, y.z, z.x, z.y, z.z};
    std::copy(r, r+9, model_); depthValid_ = false; pickValid_ = false;
}
void MeshView::setPointCloud(const Mesh& m, bool resetCamera, float spacingMm) {
    setMesh(m, resetCamera); pointMode_ = true; pointSpacing_ = spacingMm; count_ = (GLsizei)m.vertices.size();
}
void MeshView::setMesh(const Mesh& m, bool resetCamera) {
    pointMode_ = false;
    std::vector<float> v(m.vertices.size()*9);
    hasColor_ = m.colors.size() == m.vertices.size()*3;
    const bool hasNormals = m.normals.size() == m.vertices.size();
    #pragma omp parallel for
    for (size_t i = 0; i < m.vertices.size(); i++) {
        const V3& p = m.vertices[i]; const V3 n = hasNormals ? m.normals[i] : V3{0, 0, 1}; float* o = &v[i*9];
        o[0] = p.x; o[1] = p.y; o[2] = p.z; o[3] = n.x; o[4] = n.y; o[5] = n.z;
        o[6] = hasColor_ ? m.colors[i*3]/255.f : 0; o[7] = hasColor_ ? m.colors[i*3+1]/255.f : 0; o[8] = hasColor_ ? m.colors[i*3+2]/255.f : 0;
    }
    glBindVertexArray(vao_);
    glBindBuffer(GL_ARRAY_BUFFER, vbo_); glBufferData(GL_ARRAY_BUFFER, v.size()*4, v.data(), GL_STATIC_DRAW);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ibo_); glBufferData(GL_ELEMENT_ARRAY_BUFFER, m.indices.size()*4, m.indices.data(), GL_STATIC_DRAW);
    glEnableVertexAttribArray(0); glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 36, (void*)0);
    glEnableVertexAttribArray(1); glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 36, (void*)12);
    glEnableVertexAttribArray(2); glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, 36, (void*)24);
    glBindVertexArray(0);
    count_ = (GLsizei)m.indices.size(); depthValid_ = false; pickValid_ = false;
    // The floor lies under the model, measured along the up direction.
    { float lo = 1e30f, x0 = 1e30f, x1 = -1e30f, y0 = 1e30f, y1 = -1e30f; std::vector<float> zs; zs.reserve(m.vertices.size()/8+1);
      for (size_t i = 0; i < m.vertices.size(); i += 8) { const V3& p = m.vertices[i]; const float ux = model_[0]*p.x+model_[1]*p.y+model_[2]*p.z, uy = model_[3]*p.x+model_[4]*p.y+model_[5]*p.z, uz = model_[6]*p.x+model_[7]*p.y+model_[8]*p.z; zs.push_back(uz); x0 = std::min(x0, ux); x1 = std::max(x1, ux); y0 = std::min(y0, uy); y1 = std::max(y1, uy); }
      if (!zs.empty()) { std::nth_element(zs.begin(), zs.begin()+zs.size()/500, zs.end()); lo = zs[zs.size()/500]; floorZ_ = lo; floorCenter_ = V3{(x0+x1)/2, (y0+y1)/2, 0}; floorR_ = std::max(1.f, 0.5f*std::max(x1-x0, y1-y0)); } }
    if (resetCamera) {
        V3 lo, hi; meshBounds(m, lo, hi);
        center_ = (lo+hi)*0.5f; radius_ = std::max(1.f, norm(hi-lo)*0.5f); zoom_ = 1; panX_ = panY_ = 0;
        // Start on the scanned side: where most of the surface faces, seen slightly from above.
        V3 facing{}; if (hasNormals) for (const V3& n : m.normals) facing = facing + n;
        const V3 f{model_[0]*facing.x+model_[1]*facing.y+model_[2]*facing.z, model_[3]*facing.x+model_[4]*facing.y+model_[5]*facing.z, 0};
        yaw_ = std::atan2(-f.x, -f.y); pitch_ = 0.35f;
    }
}
void MeshView::setFlags(const std::vector<uint8_t>& flags) {
    hasFlags_ = !flags.empty(); if (!hasFlags_) return;
    glBindBuffer(GL_TEXTURE_BUFFER, flagBuf_); glBufferData(GL_TEXTURE_BUFFER, flags.size(), flags.data(), GL_DYNAMIC_DRAW);
    glBindTexture(GL_TEXTURE_BUFFER, flagTex_); glTexBuffer(GL_TEXTURE_BUFFER, GL_R8UI, flagBuf_);
    glBindBuffer(GL_TEXTURE_BUFFER, 0); glBindTexture(GL_TEXTURE_BUFFER, 0);
}
void MeshView::setLines(const std::vector<LineVertex>& lines) {
    glBindVertexArray(lineVao_); glBindBuffer(GL_ARRAY_BUFFER, lineVbo_);
    glBufferData(GL_ARRAY_BUFFER, lines.size()*sizeof(LineVertex), lines.data(), GL_DYNAMIC_DRAW);
    glEnableVertexAttribArray(0); glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(LineVertex), (void*)0);
    glEnableVertexAttribArray(1); glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(LineVertex), (void*)12);
    glBindVertexArray(0); lineCount_ = (GLsizei)lines.size();
}
void MeshView::pan(float dx, float dy) {
    const float dist = radius_*2.6f*zoom_, perPixel = 2*dist/(ft_*std::max(1, h_));
    panX_ += dx*perPixel; panY_ -= dy*perPixel; depthValid_ = false; pickValid_ = false;
}
void MeshView::focus(V3 c, float r) { center_ = c; panX_ = panY_ = 0; zoom_ = std::clamp(std::max(r, 1.f)*5/(radius_*2.6f), 0.01f, 10.f); depthValid_ = false; pickValid_ = false; }
void MeshView::keepInSight(const Mesh& m) {
    if (m.vertices.empty()) return;
    V3 lo, hi; meshBounds(m, lo, hi); const float margin = norm(hi-lo)*0.2f; const V3& c = center_;
    if (c.x >= lo.x-margin && c.x <= hi.x+margin && c.y >= lo.y-margin && c.y <= hi.y+margin && c.z >= lo.z-margin && c.z <= hi.z+margin) return;
    center_ = (lo+hi)*0.5f; radius_ = std::max(1.f, norm(hi-lo)*0.5f); panX_ = panY_ = 0; depthValid_ = false; pickValid_ = false;
}
GLuint MeshView::render(int w, int h) {
    if (w != w_ || h != h_) {
        w_ = w; h_ = h; depthValid_ = false; pickValid_ = false;
        glBindTexture(GL_TEXTURE_2D, color_);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, w, h, 0, GL_RGBA, GL_UNSIGNED_BYTE, nullptr);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR); glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glBindFramebuffer(GL_FRAMEBUFFER, fbo_); glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, color_, 0);
        // What is shown is antialiased. Picking needs exact ids, so it has its own single-sampled target.
        GLint maxSamples = 4; glGetIntegerv(GL_MAX_SAMPLES, &maxSamples); const GLsizei samples = std::min(maxSamples, 4);
        glBindRenderbuffer(GL_RENDERBUFFER, msColor_); glRenderbufferStorageMultisample(GL_RENDERBUFFER, samples, GL_RGBA8, w, h);
        glBindRenderbuffer(GL_RENDERBUFFER, msDepth_); glRenderbufferStorageMultisample(GL_RENDERBUFFER, samples, GL_DEPTH_COMPONENT24, w, h);
        glBindFramebuffer(GL_FRAMEBUFFER, fboMs_);
        glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_RENDERBUFFER, msColor_); glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, msDepth_);
        glBindTexture(GL_TEXTURE_2D, ids_);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_R32UI, w, h, 0, GL_RED_INTEGER, GL_UNSIGNED_INT, nullptr);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST); glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
        glBindRenderbuffer(GL_RENDERBUFFER, depth_); glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT24, w, h);
        glBindFramebuffer(GL_FRAMEBUFFER, pickFbo_);
        glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, ids_, 0);
        glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, depth_);
    }
    glBindFramebuffer(GL_FRAMEBUFFER, fboMs_);
    glViewport(0, 0, w, h); glDisable(GL_SCISSOR_TEST); glDisable(GL_BLEND); glDisable(GL_CULL_FACE); glEnable(GL_MULTISAMPLE);
    const GLfloat far1 = 1; glClearBufferfv(GL_DEPTH, 0, &far1);
    // The mesh has Z up. Turn about Z, then tilt: a pure rotation, so that nothing appears mirrored.
    { float V[9]; viewRotationZUp(yaw_, pitch_, V); for (int r = 0; r < 3; r++) for (int c = 0; c < 3; c++) R_[r*3+c] = V[r*3]*model_[c]+V[r*3+1]*model_[3+c]+V[r*3+2]*model_[6+c]; }
    const float dist = radius_*2.6f*zoom_; near_ = std::max(0.05f, dist-radius_*2); far_ = dist+radius_*3+floorR_*6;
    const float aspect = (float)w/h; ft_ = 1/std::tan(0.4f);
    for (int r = 0; r < 3; r++) t_[r] = -(R_[r*3]*center_.x + R_[r*3+1]*center_.y + R_[r*3+2]*center_.z);
    t_[0] += panX_; t_[1] += panY_; t_[2] -= dist;
    std::fill(mvp_, mvp_+16, 0.f);
    const float P0 = ft_/aspect, P1 = ft_, P2 = (far_+near_)/(near_-far_), P3 = 2*far_*near_/(near_-far_);
    for (int c = 0; c < 3; c++) { mvp_[c*4+0] = P0*R_[c]; mvp_[c*4+1] = P1*R_[3+c]; mvp_[c*4+2] = P2*R_[6+c]; mvp_[c*4+3] = -R_[6+c]; }
    mvp_[12] = P0*t_[0]; mvp_[13] = P1*t_[1]; mvp_[14] = P2*t_[2]+P3; mvp_[15] = -t_[2];
    const float nm[9] = {R_[0], R_[3], R_[6], R_[1], R_[4], R_[7], R_[2], R_[5], R_[8]};
    const float viewM[16] = {R_[0], R_[3], R_[6], 0, R_[1], R_[4], R_[7], 0, R_[2], R_[5], R_[8], 0, t_[0], t_[1], t_[2], 1};
    // Background first, without touching depth.
    glDisable(GL_DEPTH_TEST); glUseProgram(backProg_); glBindVertexArray(emptyVao_); glDrawArrays(GL_TRIANGLES, 0, 3);
    glEnable(GL_DEPTH_TEST);
    glUseProgram(prog_);
    glUniformMatrix4fv(glGetUniformLocation(prog_, "mvp"), 1, GL_FALSE, mvp_);
    glUniformMatrix4fv(glGetUniformLocation(prog_, "view"), 1, GL_FALSE, viewM);
    glUniformMatrix3fv(glGetUniformLocation(prog_, "nm"), 1, GL_FALSE, nm);
    glUniform1i(glGetUniformLocation(prog_, "useColor"), hasColor_ && showColor ? 1 : 0);
    glUniform1i(glGetUniformLocation(prog_, "hasFlags"), hasFlags_ ? 1 : 0);
    glActiveTexture(GL_TEXTURE1); glBindTexture(GL_TEXTURE_BUFFER, flagTex_); glUniform1i(glGetUniformLocation(prog_, "flags"), 1); glActiveTexture(GL_TEXTURE0);
    glBindVertexArray(vao_);
    if (pointMode_) {
        // As large as the gaps between points on screen. The view shows about 2.6 radii at zoom 1.
        glPointSize(std::clamp(1.6f*pointSpacing_*h/(radius_*2.6f*zoom_), 1.5f, 8.f));
        glDrawArrays(GL_POINTS, 0, count_);
    }
    else glDrawElements(GL_TRIANGLES, count_, GL_UNSIGNED_INT, nullptr);
    // The floor goes behind the model, blended over the background. Not from below.
    if (showFloor && pitch_ > 0.02f) {
        glEnable(GL_BLEND); glBlendFuncSeparate(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA, GL_ONE, GL_ONE_MINUS_SRC_ALPHA); glDepthMask(GL_FALSE);
        glUseProgram(floorProg_);
        glUniformMatrix4fv(glGetUniformLocation(floorProg_, "mvp"), 1, GL_FALSE, mvp_);
        const float up2mesh[9] = {model_[0], model_[1], model_[2], model_[3], model_[4], model_[5], model_[6], model_[7], model_[8]};   // columns: the inverse of model_
        glUniformMatrix3fv(glGetUniformLocation(floorProg_, "up2mesh"), 1, GL_FALSE, up2mesh);
        glUniform3f(glGetUniformLocation(floorProg_, "center"), floorCenter_.x, floorCenter_.y, 0);
        glUniform1f(glGetUniformLocation(floorProg_, "z"), floorZ_); glUniform1f(glGetUniformLocation(floorProg_, "r"), floorR_);
        const float raw = floorR_/4; const float mag = std::pow(10.f, std::floor(std::log10(raw))); const float n = raw/mag; const float step = (n < 2 ? 1 : n < 5 ? 2 : 5)*mag;
        glUniform1f(glGetUniformLocation(floorProg_, "step1"), step);
        glBindVertexArray(emptyVao_); glDrawArrays(GL_TRIANGLES, 0, 3);
        glDisable(GL_BLEND); glDepthMask(GL_TRUE);
    }
    if (lineCount_ > 0) {
        glUseProgram(lineProg_);
        glUniformMatrix4fv(glGetUniformLocation(lineProg_, "mvp"), 1, GL_FALSE, mvp_);
        glUniform2f(glGetUniformLocation(lineProg_, "viewport"), (float)w, (float)h);
        glUniform1f(glGetUniformLocation(lineProg_, "width"), 2.2f);
        glBindVertexArray(lineVao_); glDrawArrays(GL_LINES, 0, lineCount_);
    }
    glBindVertexArray(0); glUseProgram(0); glDisable(GL_DEPTH_TEST);
    glActiveTexture(GL_TEXTURE1); glBindTexture(GL_TEXTURE_BUFFER, 0); glActiveTexture(GL_TEXTURE0);
    glBindFramebuffer(GL_READ_FRAMEBUFFER, fboMs_); glBindFramebuffer(GL_DRAW_FRAMEBUFFER, fbo_);
    glBlitFramebuffer(0, 0, w, h, 0, 0, w, h, GL_COLOR_BUFFER_BIT, GL_NEAREST);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    return color_;
}
void MeshView::renderPick() {
    if (pickValid_ || !w_) return;
    glBindFramebuffer(GL_FRAMEBUFFER, pickFbo_); glViewport(0, 0, w_, h_); glDisable(GL_SCISSOR_TEST); glDisable(GL_BLEND); glEnable(GL_DEPTH_TEST);
    const GLuint none[4] = {0, 0, 0, 0}; const GLfloat far1 = 1; glClearBufferuiv(GL_COLOR, 0, none); glClearBufferfv(GL_DEPTH, 0, &far1);
    glUseProgram(pickProg_); glUniformMatrix4fv(glGetUniformLocation(pickProg_, "mvp"), 1, GL_FALSE, mvp_);
    glBindVertexArray(vao_); if (!pointMode_) glDrawElements(GL_TRIANGLES, count_, GL_UNSIGNED_INT, nullptr);
    glBindVertexArray(0); glUseProgram(0); glDisable(GL_DEPTH_TEST); glBindFramebuffer(GL_FRAMEBUFFER, 0);
    pickValid_ = true; depthValid_ = false;
}
bool MeshView::project(V3 p, float& x, float& y, float& depth) const {
    const float cw = mvp_[3]*p.x+mvp_[7]*p.y+mvp_[11]*p.z+mvp_[15]; if (cw <= 1e-6f) return false;
    const float cx = mvp_[0]*p.x+mvp_[4]*p.y+mvp_[8]*p.z+mvp_[12], cy = mvp_[1]*p.x+mvp_[5]*p.y+mvp_[9]*p.z+mvp_[13], cz = mvp_[2]*p.x+mvp_[6]*p.y+mvp_[10]*p.z+mvp_[14];
    x = (cx/cw+1)*0.5f*w_; y = (1-cy/cw)*0.5f*h_; depth = (cz/cw+1)*0.5f;
    return true;
}
long MeshView::pickTriangle(int x, int y) {
    if (x < 0 || y < 0 || x >= w_ || y >= h_) return -1;
    renderPick();
    GLuint id = 0; glBindFramebuffer(GL_READ_FRAMEBUFFER, pickFbo_); glReadBuffer(GL_COLOR_ATTACHMENT0);
    glPixelStorei(GL_PACK_ALIGNMENT, 1); glReadPixels(x, h_-1-y, 1, 1, GL_RED_INTEGER, GL_UNSIGNED_INT, &id);
    glBindFramebuffer(GL_READ_FRAMEBUFFER, 0);
    return (long)id-1;
}
void MeshView::ray(float x, float y, V3& o, V3& d) const {
    const V3 tv{t_[0], t_[1], t_[2]};
    auto toWorld = [&](V3 v) { return V3{R_[0]*v.x+R_[3]*v.y+R_[6]*v.z, R_[1]*v.x+R_[4]*v.y+R_[7]*v.z, R_[2]*v.x+R_[5]*v.y+R_[8]*v.z}; };
    o = toWorld(tv*-1.f);
    const float aspect = (float)w_/h_;
    d = normalized(toWorld(V3{(2*x/w_-1)*aspect/ft_, (1-2*y/h_)/ft_, -1}));
}
void MeshView::prepareDepth() {
    if (!pickValid_) renderPick();
    if (depthValid_ || !w_) return;
    depthPixels_.resize((size_t)w_*h_);
    glBindFramebuffer(GL_READ_FRAMEBUFFER, pickFbo_); glPixelStorei(GL_PACK_ALIGNMENT, 1);
    glReadPixels(0, 0, w_, h_, GL_DEPTH_COMPONENT, GL_FLOAT, depthPixels_.data());
    glBindFramebuffer(GL_READ_FRAMEBUFFER, 0); depthValid_ = true;
}
bool MeshView::isVisible(float x, float y, float depth, float tol) const {
    const int ix = (int)x, iy = (int)y;
    if (ix < 0 || iy < 0 || ix >= w_ || iy >= h_ || depthPixels_.empty()) return false;
    return linear(depth) <= linear(depthPixels_[(size_t)(h_-1-iy)*w_+ix])+tol;
}
