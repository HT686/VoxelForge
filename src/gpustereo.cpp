#include "gpustereo.hpp"
#include "i18n.hpp"
#define GL_GLEXT_PROTOTYPES
#define EGL_EGLEXT_PROTOTYPES
#include <EGL/egl.h>
#include <EGL/eglext.h>
#include <GL/gl.h>
#include <GL/glext.h>
#include <algorithm>
#include <cstdio>
#include <cstring>

namespace {
constexpr GLuint64 kWaitNs = 3000000000ull;   // a frame that takes this long means the card is gone

const char* kHalf = R"(#version 430
layout(local_size_x=32, local_size_y=32) in;
layout(r8ui, binding=0) uniform readonly uimage2D src; layout(r8ui, binding=1) uniform writeonly uimage2D dst;
void main(){ ivec2 p=ivec2(gl_GlobalInvocationID.xy); if(any(greaterThanEqual(p,imageSize(dst)))) return;
  ivec2 q=p*2; uint s=imageLoad(src,q).r+imageLoad(src,q+ivec2(1,0)).r+imageLoad(src,q+ivec2(0,1)).r+imageLoad(src,q+ivec2(1,1)).r;
  imageStore(dst,p,uvec4((s+2u)>>2)); })";

const char* kCensus = R"(#version 430
layout(local_size_x=32, local_size_y=32) in;
layout(r8ui, binding=0) uniform readonly uimage2D src; layout(rg32ui, binding=1) uniform writeonly uimage2D dst;
void main(){ ivec2 p=ivec2(gl_GlobalInvocationID.xy), s=imageSize(src); if(any(greaterThanEqual(p,s))) return;
  uvec2 code=uvec2(0u);
  if(p.x>=3&&p.y>=3&&p.x<s.x-3&&p.y<s.y-3){ uint m=imageLoad(src,p).r; int bit=0;
    for(int dy=-3;dy<=3;dy++) for(int dx=-3;dx<=3;dx++){ if(dx==0&&dy==0) continue;
      if(imageLoad(src,p+ivec2(dx,dy)).r>m){ if(bit<32) code.x|=1u<<bit; else code.y|=1u<<(bit-32); } bit++; } }
  imageStore(dst,p,uvec4(code,0u,0u)); })";

// For each cell and shift: the census distance summed over the 7 rows of the window.
const char* kColumns = R"(#version 430
layout(local_size_x=16, local_size_y=16, local_size_z=1) in;
layout(rg32ui, binding=0) uniform readonly uimage2D cRef; layout(rg32ui, binding=1) uniform readonly uimage2D cOther; layout(r16ui, binding=2) uniform writeonly uimage3D columns;
uniform int qh, qw, hw, sMin, nS; const int R=3;
void main(){ ivec3 p=ivec3(gl_GlobalInvocationID); if(p.x>=qw||p.y>=qh||p.z>=nS||p.y<R+2||p.y>=qh-R-2) return;
  uint sum=0u; int xo=2*p.x-sMin-p.z;
  for(int yy=p.y-R;yy<=p.y+R;yy++){
    if(xo>=3&&xo<hw-3){ uvec2 a=imageLoad(cRef,ivec2(2*p.x,2*yy)).rg, b=imageLoad(cOther,ivec2(xo,2*yy)).rg; sum+=uint(bitCount(a.x^b.x)+bitCount(a.y^b.y)); }
    else sum+=48u; }
  imageStore(columns,p,uvec4(sum)); })";

const char* kSelect = R"(#version 430
layout(local_size_x=16, local_size_y=16) in;
layout(r16ui, binding=0) uniform readonly uimage3D columns; layout(r16i, binding=1) uniform writeonly iimage2D coarse;
uniform int qh, qw, hw, sMin, nS; const int R=3, kNoShift=-32768;
uint box[1024];
void main(){ ivec2 p=ivec2(gl_GlobalInvocationID.xy); if(p.x>=qw||p.y>=qh) return;
  int result=kNoShift;
  if(p.y>=R+2&&p.y<qh-R-2&&p.x>=R+2&&p.x<qw-R){
    uint best=0xffffffffu, second=0xffffffffu, total=0u; int bi=-1;
    for(int k=0;k<nS;k++){ uint s=0u; for(int xx=p.x-R;xx<=p.x+R;xx++) s+=imageLoad(columns,ivec3(xx,p.y,k)).r; box[k]=s; total+=s; if(s<best){best=s;bi=k;} }
    for(int k=0;k<nS;k++) if(abs(k-bi)>2&&box[k]<second) second=box[k];
    int xb=2*p.x-(sMin+bi);
    if(bi>=0&&xb>=3&&xb<hw-3&&best*20u<second*18u&&best*uint(nS)*100u<total*55u) result=2*(sMin+bi);
  }
  imageStore(coarse,p,ivec4(result)); })";

const char* kFine = R"(#version 430
layout(local_size_x=16, local_size_y=16) in;
layout(r8ui, binding=0) uniform readonly uimage2D imRef; layout(r8ui, binding=1) uniform readonly uimage2D imOther; layout(r16i, binding=2) uniform readonly iimage2D coarse;
layout(std430, binding=3) writeonly buffer Out { float shift[]; };
uniform int W, H, gw, gh, step, B, qw, qh, minVariance; uniform float minNcc; const int kNoShift=-32768, Search=4;
void main(){ ivec2 g=ivec2(gl_GlobalInvocationID.xy); if(g.x>=gw||g.y>=gh) return; int cell=g.y*gw+g.x; shift[cell]=0.0;
  int margin=B/step+1; if(g.x<margin||g.y<margin||g.x>=gw-margin||g.y>=gh-margin) return;
  int x=step*g.x, y=step*g.y, qx=x/4, qy=y/4; int cand[3]; int nc=0;
  for(int r=0;r<=2&&nc==0;r++) for(int dy=-r;dy<=r;dy++) for(int dx=-r;dx<=r;dx++){
    int yy=qy+dy, xx=qx+dx; if(yy<0||yy>=qh||xx<0||xx>=qw) continue;
    int c=imageLoad(coarse,ivec2(xx,yy)).r; if(c==kNoShift) continue;
    bool dup=false; for(int q=0;q<nc;q++) dup=dup||abs(cand[q]-c)<=Search;
    if(!dup&&nc<3){cand[nc]=c;nc++;} }
  if(nc==0) return;
  const int N=(2*B+1)*(2*B+1); int sa=0, saa=0; int A[225]; int a=0;
  for(int dy=-B;dy<=B;dy++) for(int dx=-B;dx<=B;dx++){ int v=int(imageLoad(imRef,ivec2(x+dx,y+dy)).r); A[a]=v; a++; sa+=v; saa+=v*v; }
  int varA=N*saa-sa*sa; if(varA<minVariance*N*N) return;
  float bestScore=-2.0, bestShift=0.0; float scores[11];
  for(int q=0;q<nc;q++){
    int s0=cand[q]-Search-1, first=-1, last=-1;
    for(int k=0;k<11;k++){
      int s=s0+k, xo=x-s; scores[k]=-2.0;
      if(xo-B<0||xo+B+1>W) continue;
      int sb=0, sbb=0, sab=0; a=0;
      for(int dy=-B;dy<=B;dy++) for(int dx=-B;dx<=B;dx++){ int v=int(imageLoad(imOther,ivec2(xo+dx,y+dy)).r); sb+=v; sbb+=v*v; sab+=A[a]*v; a++; }
      int varB=N*sbb-sb*sb; if(varB<minVariance*N*N) continue;
      scores[k]=float(N*sab-sa*sb)/sqrt(float(varA)*float(varB));
      if(first<0) first=k;
      last=k;
    }
    if(first<0) continue;
    for(int k=max(1,first);k<=min(last,9);k++)
      if(scores[k]>bestScore&&scores[k]>=scores[k-1]&&scores[k]>=scores[k+1]&&scores[k-1]>-1.5&&scores[k+1]>-1.5){
        float dn=scores[k-1]-2.0*scores[k]+scores[k+1]; bestScore=scores[k];
        bestShift=float(s0+k)+(dn<-1e-6?0.5*(scores[k-1]-scores[k+1])/dn:0.0); }
  }
  if(bestScore>=minNcc) shift[cell]=bestShift; })";

// See StereoMatcher::refineSlanted.
const char* kRefine = R"(#version 430
layout(local_size_x=16, local_size_y=16) in;
layout(r8ui, binding=0) uniform readonly uimage2D imRef; layout(r8ui, binding=1) uniform readonly uimage2D imOther;
layout(std430, binding=2) readonly buffer In { float shiftIn[]; }; layout(std430, binding=3) writeonly buffer Out { float shiftOut[]; };
uniform int W, H, gw, gh, step, B, detail, limit; uniform float minNcc, focal, cxRef, cy, dispOffset, baselineProduct, cX, cY, cZ, cR;
const int kN=13; const float kStep=0.25;
float depthOf(float s){ float d=dispOffset-s; return d>1.0?baselineProduct/d:0.0; }
void main(){ ivec2 g=ivec2(gl_GlobalInvocationID.xy); if(g.x>=gw||g.y>=gh) return; int cell=g.y*gw+g.x;
  float s0=shiftIn[cell]; float keep=detail==1?0.0:s0; shiftOut[cell]=keep;
  int fitR=max(3,12/step);
  if(g.x<fitR||g.y<fitR||g.x>=gw-fitR||g.y>=gh-fitR||s0==0.0) return;
  if(limit==1&&detail==0){ float z=depthOf(s0), X=-(float(step*g.x)-cxRef)/focal*z-cX, Y=(float(step*g.y)-cy)/focal*z-cY, Z=z-cZ; if(X*X+Y*Y+Z*Z>cR*cR) return; }
  float sxx=0.0,sxy=0.0,syy=0.0,sxs=0.0,sys=0.0; int n=0;
  for(int dy=-fitR;dy<=fitR;dy++) for(int dx=-fitR;dx<=fitR;dx++){
    float q=shiftIn[(g.y+dy)*gw+g.x+dx];
    if(q==0.0||abs(q-s0)>0.25*float(max(abs(dx),abs(dy))*step)+1.0) continue;
    float X=float(dx*step), Y=float(dy*step), Z=q-s0; sxx+=X*X; sxy+=X*Y; syy+=Y*Y; sxs+=X*Z; sys+=Y*Z; n++; }
  float det=sxx*syy-sxy*sxy; if(n<30||det<1e-6) return;
  float gx=(sxs*syy-sys*sxy)/det, gy=(sys*sxx-sxs*sxy)/det;
  int x=step*g.x, y=step*g.y; if(y-B<0||y+B>=H||x-B<0||x+B>=W) return;
  const int N=(2*B+1)*(2*B+1); float A[225]; float sa=0.0, saa=0.0; int a=0;
  // Measured from the middle of the range: the sums stay small, and single precision is enough for them.
  for(int dy=-B;dy<=B;dy++) for(int dx=-B;dx<=B;dx++){ float v=float(imageLoad(imRef,ivec2(x+dx,y+dy)).r)-128.0; A[a]=v; a++; sa+=v; saa+=v*v; }
  float fN=float(N), varA=fN*saa-sa*sa; if(varA<=(detail==1?64.0*fN*fN:0.0)) return;
  float scores[kN]; 
  for(int c=0;c<kN;c++){
    float s=s0+float(c-kN/2)*kStep; float sb=0.0,sbb=0.0,sab=0.0; a=0;
    for(int dy=-B;dy<=B;dy++) for(int dx=-B;dx<=B;dx++){
      float xo=float(x+dx)-(s+gx*float(dx)+gy*float(dy)); int xi=int(floor(xo)); if(xi<0||xi+1>=W) return;
      float t=xo-float(xi); float v=float(imageLoad(imOther,ivec2(xi,y+dy)).r)*(1.0-t)+float(imageLoad(imOther,ivec2(xi+1,y+dy)).r)*t-128.0;
      sb+=v; sbb+=v*v; sab+=A[a]*v; a++; }
    float varB=fN*sbb-sb*sb; scores[c]=varB>0.0?(fN*sab-sa*sb)/sqrt(varA*varB):-2.0;
  }
  int k=0; for(int c=1;c<kN;c++) if(scores[c]>scores[k]) k=c;
  if(k==0||k==kN-1||scores[k]<minNcc) return;
  float dn=scores[k-1]-2.0*scores[k]+scores[k+1];
  if(dn<-1e-6) shiftOut[cell]=s0+(float(k-kN/2)+0.5*(scores[k-1]-scores[k+1])/dn)*kStep; })";
}

struct GpuStereo::Impl {
    EGLDisplay display = EGL_NO_DISPLAY; EGLContext context = EGL_NO_CONTEXT; EGLSurface surface = EGL_NO_SURFACE; std::string info = tr("noch nicht gestartet"); bool ready = false;
    GLuint halfP = 0, censusP = 0, columnsP = 0, selectP = 0, fineP = 0, refineP = 0;
    GLuint texRef = 0, texOther = 0, halfRef = 0, halfOther = 0, cenRef = 0, cenOther = 0, coarse = 0, columns = 0, bufA = 0, bufB = 0;
    int W = 0, H = 0, volumeS = 0, cells = 0;
    GLuint program(const char* src, const char* name) {
        GLuint s = glCreateShader(GL_COMPUTE_SHADER); glShaderSource(s, 1, &src, nullptr); glCompileShader(s); GLint ok = 0; glGetShaderiv(s, GL_COMPILE_STATUS, &ok);
        if (!ok) { char log[800] = ""; glGetShaderInfoLog(s, sizeof log, nullptr, log); info = std::string("Shader ")+name+": "+log; glDeleteShader(s); return 0; }
        GLuint p = glCreateProgram(); glAttachShader(p, s); glLinkProgram(p); glDeleteShader(s); glGetProgramiv(p, GL_LINK_STATUS, &ok);
        if (!ok) { char log[800] = ""; glGetProgramInfoLog(p, sizeof log, nullptr, log); info = std::string("Shader ")+name+": "+log; glDeleteProgram(p); return 0; }
        return p;
    }
    static GLuint tex2D(GLenum fmt, int w, int h) { GLuint t = 0; glGenTextures(1, &t); glBindTexture(GL_TEXTURE_2D, t); glTexStorage2D(GL_TEXTURE_2D, 1, fmt, w, h); return t; }
    void set(GLuint p, const char* n, int v) { glUniform1i(glGetUniformLocation(p, n), v); }
    void setF(GLuint p, const char* n, float v) { glUniform1f(glGetUniformLocation(p, n), v); }
    bool current() { return eglGetCurrentContext() == context || eglMakeCurrent(display, surface, surface, context); }
    bool wait() {
        GLsync f = glFenceSync(GL_SYNC_GPU_COMMANDS_COMPLETE, 0); const GLenum r = glClientWaitSync(f, GL_SYNC_FLUSH_COMMANDS_BIT, kWaitNs); glDeleteSync(f);
        if (r != GL_ALREADY_SIGNALED && r != GL_CONDITION_SATISFIED) { info = tr("Die Grafikkarte antwortet nicht mehr."); ready = false; return false; }
        if (glGetError() != GL_NO_ERROR) { info = tr("Fehler der Grafikkarte."); ready = false; return false; }
        return true;
    }
    void resize(int w, int h) {
        if (w == W && h == H) return;
        W = w; H = h;
        texRef = tex2D(GL_R8UI, W, H); texOther = tex2D(GL_R8UI, W, H); halfRef = tex2D(GL_R8UI, W/2, H/2); halfOther = tex2D(GL_R8UI, W/2, H/2);
        cenRef = tex2D(GL_RG32UI, W/2, H/2); cenOther = tex2D(GL_RG32UI, W/2, H/2); coarse = tex2D(GL_R16I, W/4, H/4);
    }
    void volume(int nS) {
        if (nS == volumeS) return;
        if (columns) glDeleteTextures(1, &columns);
        glGenTextures(1, &columns); glBindTexture(GL_TEXTURE_3D, columns); glTexStorage3D(GL_TEXTURE_3D, 1, GL_R16UI, W/4, H/4, nS); volumeS = nS;
    }
    void buffers(int n) {
        if (n == cells) return;
        if (!bufA) { glGenBuffers(1, &bufA); glGenBuffers(1, &bufB); }
        for (GLuint b : {bufA, bufB}) { glBindBuffer(GL_SHADER_STORAGE_BUFFER, b); glBufferData(GL_SHADER_STORAGE_BUFFER, (GLsizeiptr)n*4, nullptr, GL_DYNAMIC_COPY); }
        cells = n;
    }
};
GpuStereo::GpuStereo() : impl_(std::make_unique<Impl>()) {}
GpuStereo::~GpuStereo() {
    if (impl_->display != EGL_NO_DISPLAY) { eglMakeCurrent(impl_->display, EGL_NO_SURFACE, EGL_NO_SURFACE, EGL_NO_CONTEXT); if (impl_->context != EGL_NO_CONTEXT) eglDestroyContext(impl_->display, impl_->context); if (impl_->surface != EGL_NO_SURFACE) eglDestroySurface(impl_->display, impl_->surface); }
}
const std::string& GpuStereo::info() const { return impl_->info; }
bool GpuStereo::init() {
    Impl& g = *impl_; if (g.ready) return g.current();
    // No window is needed: first without any display system, then through the default display.
    auto getPlatform = (PFNEGLGETPLATFORMDISPLAYEXTPROC)eglGetProcAddress("eglGetPlatformDisplayEXT");
    EGLint major = 0, minor = 0;
    if (getPlatform) { g.display = getPlatform(EGL_PLATFORM_SURFACELESS_MESA, EGL_DEFAULT_DISPLAY, nullptr); if (g.display != EGL_NO_DISPLAY && !eglInitialize(g.display, &major, &minor)) g.display = EGL_NO_DISPLAY; }
    if (g.display == EGL_NO_DISPLAY) { g.display = eglGetDisplay(EGL_DEFAULT_DISPLAY); if (g.display == EGL_NO_DISPLAY || !eglInitialize(g.display, &major, &minor)) { g.display = EGL_NO_DISPLAY; g.info = tr("Keine Grafikschnittstelle (EGL) gefunden."); return false; } }
    if (!eglBindAPI(EGL_OPENGL_API)) { g.info = tr("OpenGL über EGL ist nicht verfügbar."); return false; }
    const EGLint want[] = {EGL_SURFACE_TYPE, EGL_PBUFFER_BIT, EGL_RENDERABLE_TYPE, EGL_OPENGL_BIT, EGL_NONE}; EGLConfig config = nullptr; EGLint found = 0;
    eglChooseConfig(g.display, want, &config, 1, &found);
    const EGLint version[] = {EGL_CONTEXT_MAJOR_VERSION, 4, EGL_CONTEXT_MINOR_VERSION, 3, EGL_CONTEXT_OPENGL_PROFILE_MASK, EGL_CONTEXT_OPENGL_CORE_PROFILE_BIT, EGL_NONE};
    g.context = eglCreateContext(g.display, found ? config : EGL_NO_CONFIG_KHR, EGL_NO_CONTEXT, version);
    if (g.context == EGL_NO_CONTEXT) { g.info = tr("Die Grafikkarte bietet kein OpenGL 4.3."); return false; }
    if (!eglMakeCurrent(g.display, EGL_NO_SURFACE, EGL_NO_SURFACE, g.context)) {
        const EGLint size[] = {EGL_WIDTH, 1, EGL_HEIGHT, 1, EGL_NONE}; if (found) g.surface = eglCreatePbufferSurface(g.display, config, size);
        if (g.surface == EGL_NO_SURFACE || !eglMakeCurrent(g.display, g.surface, g.surface, g.context)) { g.info = tr("Der Grafikkontext ließ sich nicht aktivieren."); return false; }
    }
    const char* renderer = (const char*)glGetString(GL_RENDERER); g.info = renderer ? renderer : "unbekannt";
    // A software renderer would only be slower than the CPU code.
    if (g.info.find("llvmpipe") != std::string::npos || g.info.find("softpipe") != std::string::npos || g.info.find("Software") != std::string::npos) { g.info = trf("Nur Software-Grafik (%s).", g.info); return false; }
    if (!(g.halfP = g.program(kHalf, "half")) || !(g.censusP = g.program(kCensus, "census")) || !(g.columnsP = g.program(kColumns, "columns")) || !(g.selectP = g.program(kSelect, "select")) || !(g.fineP = g.program(kFine, "fine")) || !(g.refineP = g.program(kRefine, "refine"))) return false;
    g.ready = true; return true;
}
bool GpuStereo::match(const uint8_t* ref, const uint8_t* other, int W, int H, const StereoParams& params, std::vector<float>& shift) {
    Impl& g = *impl_; if (!g.ready || !g.current()) return false;
    while (glGetError() != GL_NO_ERROR) {}
    g.resize(W, H);
    const int hw = W/2, hh = H/2, qw = W/4, qh = H/4, sMin = params.shiftMin/2, nS = std::clamp((params.shiftMax-params.shiftMin)/2, 1, 1024), step = params.gridStep, gw = W/step, gh = H/step;
    g.volume(nS); g.buffers(gw*gh);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    glBindTexture(GL_TEXTURE_2D, g.texRef); glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, W, H, GL_RED_INTEGER, GL_UNSIGNED_BYTE, ref);
    glBindTexture(GL_TEXTURE_2D, g.texOther); glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, W, H, GL_RED_INTEGER, GL_UNSIGNED_BYTE, other);
    const auto groups = [](int n, int local) { return (GLuint)((n+local-1)/local); };
    glUseProgram(g.halfP);
    glBindImageTexture(0, g.texRef, 0, GL_FALSE, 0, GL_READ_ONLY, GL_R8UI); glBindImageTexture(1, g.halfRef, 0, GL_FALSE, 0, GL_WRITE_ONLY, GL_R8UI); glDispatchCompute(groups(hw, 32), groups(hh, 32), 1);
    glBindImageTexture(0, g.texOther, 0, GL_FALSE, 0, GL_READ_ONLY, GL_R8UI); glBindImageTexture(1, g.halfOther, 0, GL_FALSE, 0, GL_WRITE_ONLY, GL_R8UI); glDispatchCompute(groups(hw, 32), groups(hh, 32), 1);
    glMemoryBarrier(GL_SHADER_IMAGE_ACCESS_BARRIER_BIT);
    glUseProgram(g.censusP);
    glBindImageTexture(0, g.halfRef, 0, GL_FALSE, 0, GL_READ_ONLY, GL_R8UI); glBindImageTexture(1, g.cenRef, 0, GL_FALSE, 0, GL_WRITE_ONLY, GL_RG32UI); glDispatchCompute(groups(hw, 32), groups(hh, 32), 1);
    glBindImageTexture(0, g.halfOther, 0, GL_FALSE, 0, GL_READ_ONLY, GL_R8UI); glBindImageTexture(1, g.cenOther, 0, GL_FALSE, 0, GL_WRITE_ONLY, GL_RG32UI); glDispatchCompute(groups(hw, 32), groups(hh, 32), 1);
    glMemoryBarrier(GL_SHADER_IMAGE_ACCESS_BARRIER_BIT);
    glUseProgram(g.columnsP); g.set(g.columnsP, "qh", qh); g.set(g.columnsP, "qw", qw); g.set(g.columnsP, "hw", hw); g.set(g.columnsP, "sMin", sMin); g.set(g.columnsP, "nS", nS);
    glBindImageTexture(0, g.cenRef, 0, GL_FALSE, 0, GL_READ_ONLY, GL_RG32UI); glBindImageTexture(1, g.cenOther, 0, GL_FALSE, 0, GL_READ_ONLY, GL_RG32UI); glBindImageTexture(2, g.columns, 0, GL_TRUE, 0, GL_WRITE_ONLY, GL_R16UI);
    glDispatchCompute(groups(qw, 16), groups(qh, 16), (GLuint)nS);
    glMemoryBarrier(GL_SHADER_IMAGE_ACCESS_BARRIER_BIT);
    glUseProgram(g.selectP); g.set(g.selectP, "qh", qh); g.set(g.selectP, "qw", qw); g.set(g.selectP, "hw", hw); g.set(g.selectP, "sMin", sMin); g.set(g.selectP, "nS", nS);
    glBindImageTexture(0, g.columns, 0, GL_TRUE, 0, GL_READ_ONLY, GL_R16UI); glBindImageTexture(1, g.coarse, 0, GL_FALSE, 0, GL_WRITE_ONLY, GL_R16I);
    glDispatchCompute(groups(qw, 16), groups(qh, 16), 1);
    glMemoryBarrier(GL_SHADER_IMAGE_ACCESS_BARRIER_BIT);
    glUseProgram(g.fineP); g.set(g.fineP, "W", W); g.set(g.fineP, "H", H); g.set(g.fineP, "gw", gw); g.set(g.fineP, "gh", gh); g.set(g.fineP, "step", step); g.set(g.fineP, "B", params.windowRadius); g.set(g.fineP, "qw", qw); g.set(g.fineP, "qh", qh); g.set(g.fineP, "minVariance", params.minVariance); g.setF(g.fineP, "minNcc", params.minNcc);
    glBindImageTexture(0, g.texRef, 0, GL_FALSE, 0, GL_READ_ONLY, GL_R8UI); glBindImageTexture(1, g.texOther, 0, GL_FALSE, 0, GL_READ_ONLY, GL_R8UI); glBindImageTexture(2, g.coarse, 0, GL_FALSE, 0, GL_READ_ONLY, GL_R16I);
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 3, g.bufA);
    glDispatchCompute(groups(gw, 16), groups(gh, 16), 1);
    glMemoryBarrier(GL_BUFFER_UPDATE_BARRIER_BIT|GL_SHADER_STORAGE_BARRIER_BIT);
    if (!g.wait()) return false;
    shift.resize((size_t)gw*gh); glBindBuffer(GL_SHADER_STORAGE_BUFFER, g.bufA); glGetBufferSubData(GL_SHADER_STORAGE_BUFFER, 0, (GLsizeiptr)shift.size()*4, shift.data());
    return glGetError() == GL_NO_ERROR;
}
bool GpuStereo::refine(int W, int H, int gw, int gh, int step, const StereoCalib& calib, const StereoParams& params, int radius, float minNcc, bool detail, std::vector<float>& shift) {
    Impl& g = *impl_; if (!g.ready || !g.current() || W != g.W || H != g.H) return false;
    g.buffers(gw*gh); glBindBuffer(GL_SHADER_STORAGE_BUFFER, g.bufA); glBufferSubData(GL_SHADER_STORAGE_BUFFER, 0, (GLsizeiptr)gw*gh*4, shift.data());
    glUseProgram(g.refineP); const GLuint p = g.refineP;
    g.set(p, "W", W); g.set(p, "H", H); g.set(p, "gw", gw); g.set(p, "gh", gh); g.set(p, "step", step); g.set(p, "B", radius); g.set(p, "detail", detail ? 1 : 0); g.set(p, "limit", params.slantLimit ? 1 : 0);
    g.setF(p, "minNcc", minNcc); g.setF(p, "focal", calib.focalPx); g.setF(p, "cxRef", calib.cxRef()); g.setF(p, "cy", calib.cy()); g.setF(p, "dispOffset", calib.dispOffset); g.setF(p, "baselineProduct", calib.focalPx*calib.baselineMm); g.setF(p, "cX", params.slantCx); g.setF(p, "cY", params.slantCy); g.setF(p, "cZ", params.slantCz); g.setF(p, "cR", params.slantR);
    glBindImageTexture(0, g.texRef, 0, GL_FALSE, 0, GL_READ_ONLY, GL_R8UI); glBindImageTexture(1, g.texOther, 0, GL_FALSE, 0, GL_READ_ONLY, GL_R8UI);
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 2, g.bufA); glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 3, g.bufB);
    glDispatchCompute((GLuint)((gw+15)/16), (GLuint)((gh+15)/16), 1);
    glMemoryBarrier(GL_BUFFER_UPDATE_BARRIER_BIT|GL_SHADER_STORAGE_BARRIER_BIT);
    if (!g.wait()) return false;
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, g.bufB); glGetBufferSubData(GL_SHADER_STORAGE_BUFFER, 0, (GLsizeiptr)gw*gh*4, shift.data());
    return glGetError() == GL_NO_ERROR;
}
