// Command line tool: device info, live depth check, recording, offline reconstruction.
#include <algorithm>
#include <chrono>
#include <vector>
#include <cstdio>
#include <string>
#include "device.hpp"
#include "markers.hpp"
#include "measure.hpp"
#include "meshedit.hpp"
#include "pipeline.hpp"
#include "project.hpp"
#include "register.hpp"
#include "simscan.hpp"
#include "boardcal.hpp"
#include "source.hpp"
#include <cmath>
#include <random>

static int usage() {
    puts("voxelforge-cli info\n"
         "voxelforge-cli depth [frames] [exposure] [fern]    Tiefe live messen, Statistik ausgeben\n"
         "voxelforge-cli record <ordner> <frames> [exposure] [fern] [farbe] [laseraus]\n"
         "voxelforge-cli tafel <ordner> [ordner ...]    Kalibrierplatte (30 mm Punktabstand) vermessen, Korrektur bestimmen\n"
         "voxelforge-cli tafelvergleich <ordner>        Aufnahme mit 'beides': Ebene der dichten Tiefe gegen die Ebene der Punkte\n"
         "voxelforge-cli reconstruct <ordner> <ausgabe.stl|obj|ply> [voxel_mm] [drehteller]\n"
         "voxelforge-cli optimieren <ein.stl|obj|ply> <aus.stl|obj|ply> [lochumfang_mm, -1 = alle]   Ein-Klick-Prozess\n"
         "voxelforge-cli abstand [sekunden]             Abstand wie in der App mitschreiben (Profil Klein)\n"
         "voxelforge-cli vergleich <ordner> [bilder] [voxel_mm]  Stereo auf CPU und Grafikkarte Bild fuer Bild vergleichen\n"
         "voxelforge-cli messen <scan.stl>              Flaechen, Kantenlaengen und Winkel eines Quaders\n"
         "voxelforge-cli projekt <datei.vfproj>         Scans eines Projekts auflisten\n"
         "voxelforge-cli ausrichten <projekt.vfproj> [ziel.ply] [a b]   Wie die App: jeden Scan an den ersten ausgerichteten (oder Scan b an a)\n"
         "voxelforge-cli verfolgtest <projekt.vfproj> [scan]   Form-Tracking ohne Marker gegen die Marker-Lagen eines Scans pruefen\n"
         "voxelforge-cli ausrichttest <modell.stl|projekt.vfproj> [durchgaenge]   Automatische Ausrichtung pruefen");
    return 2;
}

// ---- Test bench for aligning scans ------------------------------------------------------------
// How far the points of m land from where they belong, root mean square in mm.
static float poseError(const Mesh& m, const Pose& got, const Pose& truth) {
    double s = 0; size_t n = 0; const size_t step = std::max<size_t>(1, m.vertices.size()/2000);
    for (size_t i = 0; i < m.vertices.size(); i += step) { const V3 e = got.apply(m.vertices[i])-truth.apply(m.vertices[i]); s += dot(e, e); n++; }
    return n ? (float)std::sqrt(s/n) : 0.f;
}
struct BenchCase { std::string name; Mesh src, dst; Pose truth; V3 srcUp, dstUp; Table srcTable, dstTable; };
static void runBench(std::vector<BenchCase>& cases) {
    int first = 0, found = 0, confident = 0, wrongConfident = 0; double seconds = 0;
    for (BenchCase& c : cases) {
        const auto t0 = std::chrono::steady_clock::now();
        std::vector<RegisterResult> r;
        if (const char* pairs = getenv("VF_PAARE")) {
            // A person picks n spots on the shared surface, each about 1.5 mm off.
            const int n = atoi(pairs); std::mt19937 pg(7+(int)cases.size()); std::normal_distribution<float> jitter(0, 0.9f);
            std::vector<size_t> shared; std::vector<V3> dn; for (size_t i = 0; i < c.src.vertices.size(); i += 7) { const V3 q = c.truth.apply(c.src.vertices[i]); float best = 1e9f; for (size_t j = 0; j < c.dst.vertices.size(); j += 3) best = std::min(best, norm(c.dst.vertices[j]-q)); if (best < 0.6f) shared.push_back(i); }
            std::vector<V3> sp, sn, dp; std::uniform_int_distribution<size_t> pick(0, shared.empty() ? 0 : shared.size()-1);
            for (int k = 0; k < n && !shared.empty(); k++) { const size_t i = shared[pick(pg)]; sp.push_back(c.src.vertices[i]+V3{jitter(pg), jitter(pg), jitter(pg)}); sn.push_back(c.src.normals[i]); dp.push_back(c.truth.apply(c.src.vertices[i])); dn.push_back(c.truth.rot(c.src.normals[i])); }
            r = registerFromPairs(c.src, c.dst, sp, sn, dp, dn, c.srcTable, c.dstTable);
        } else r = registerAuto(c.src, c.dst, {}, c.srcTable, c.dstTable);
        const double dt = std::chrono::duration<double>(std::chrono::steady_clock::now()-t0).count(); seconds += dt;
        int rank = -1; float err0 = r.empty() ? -1 : poseError(c.src, r[0].srcToDst, c.truth);
        for (size_t i = 0; i < r.size() && rank < 0; i++) if (poseError(c.src, r[i].srcToDst, c.truth) < 1.f) rank = (int)i;
        const bool conf = !r.empty() && r[0].confident;
        first += rank == 0; found += rank >= 0; confident += rank == 0 && conf; wrongConfident += rank != 0 && conf;
        printf("%-28s %2zu Vorschl.  richtig an %2d  Fehler erster %7.2f mm  %s  Ueberl. %3.0f%%  Rest %.3f  Widerspr. %4.1f%%  fest %.3f  %.1f s\n", c.name.c_str(), r.size(), rank < 0 ? -1 : rank+1, err0,
               conf ? "SICHER" : "      ", r.empty() ? 0.f : 100*r[0].overlap, r.empty() ? 0.f : r[0].rmsMm, r.empty() ? 0.f : 100*r[0].conflict, r.empty() ? 0.f : r[0].pinned, dt);
        if (getenv("VF_WAHR")) { const RegisterResult t = judgePlacement(c.src, c.dst, c.truth, c.srcTable, c.dstTable), f = refineRegistration(c.src, c.dst, c.truth, 1.f, c.srcTable, c.dstTable);
            printf("    wahr: Ueberl. %3.0f%% Rest %.3f Widerspr. %4.1f%% fest %.3f | nach ICP %.2f mm daneben, fest %.3f\n", 100*t.overlap, t.rmsMm, 100*t.conflict, t.pinned, poseError(c.src, f.srcToDst, c.truth), f.pinned); }
        fflush(stdout);
    }
    printf("\n%zu Faelle: richtig an erster Stelle %d, unter den Vorschlaegen %d, sicher und richtig %d, sicher aber falsch %d, %.1f s\n", cases.size(), first, found, confident, wrongConfident, seconds);
}
static bool openDev(OtterDevice& d) { std::string e; if (!d.open(e)) { fprintf(stderr, "Fehler: %s\n", e.c_str()); return false; } return true; }

int main(int argc, char** argv) {
    if (argc < 2) return usage();
    std::string cmd = argv[1];
    if (cmd == "info") {
        OtterDevice d; if (!openDev(d)) return 1;
        printf("%s\nf=%.3f px  Basis=%.3f mm  Versatz=%.1f px  Belichtung=%d\n", d.description().c_str(), d.calib().focalPx, d.calib().baselineMm, d.calib().dispOffset, d.irExposure());
        return 0;
    }
    if (cmd == "depth") {
        int frames = argc > 2 ? atoi(argv[2]) : 60, expo = argc > 3 ? atoi(argv[3]) : 3000;
        OtterDevice d; if (!openDev(d)) return 1;
        if (argc > 4 && std::string(argv[4]) == "fern") d.setCameraPair(CameraPair::Far);
        std::string e; if (!d.startStreams(e)) { fprintf(stderr, "Fehler: %s\n", e.c_str()); return 1; }
        d.setIrExposure(expo);
        Pipeline p; StereoFrame f; int n = 0, drop = 0; float ms = 0; float zc = 0, zv = 0;
        auto t0 = std::chrono::steady_clock::now();
        while (n < frames) {
            if (!d.nextPair(f, 2000)) { fprintf(stderr, "Timeout\n"); drop++; if (drop > 3) return 1; continue; }
            p.process(f, d.calib()); ms += p.stats().stereoMs; n++;
            zv += p.stats().validDepth; zc += p.stats().medianDepth;
        }
        double s = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
        printf("Bilder=%d  %.1f fps  Stereo %.1f ms  gueltig %.1f %%  Median-Tiefe %.1f mm\n", n, n/s, ms/n, 100*zv/n, zc/n);
        return 0;
    }
    if (cmd == "vergleich" && argc >= 3) {
        StereoCalib c; if (!readCalib(argv[2], c)) { fprintf(stderr, "calib.txt fehlt\n"); return 1; }
        const int frames = argc > 3 ? atoi(argv[3]) : 200; const float voxel = argc > 4 ? (float)atof(argv[4]) : .15f; const ScanProfile prof = profileFor(ScanSize::Small);
        StereoMatcher cpu, gpu; gpu.compute_mode = ComputeMode::Gpu;
        const float fb = c.focalPx*c.baselineMm; const int sMin = (int)std::floor((c.dispOffset-fb/prof.zMin)/2)*2, sMax = (int)std::ceil((c.dispOffset-fb/prof.zMax)/2)*2;
        for (StereoMatcher* m : {&cpu, &gpu}) { m->params.shiftMin = sMin; m->params.shiftMax = std::max(sMin+32, sMax); m->params.slantRefine = voxel <= .25f; }
        StereoFrame f; DepthMap dc, dg; int n = 0, step = 1; const int total = [&] { int k = 0; while (k < 100000 && readPair(argv[2], k+50, f)) k += 50; return k+50; }(); step = std::max(1, total/std::max(1, frames));
        std::vector<float> diffs; std::vector<float> diffsMm; size_t both = 0, onlyC = 0, onlyG = 0; double msC = 0, msG = 0; StereoTimes tg{};
        auto measure = [&](const DepthMap& a, const DepthMap& b) { for (size_t i = 0; i < a.z.size(); i++) { const bool x = a.z[i] > 0, y = b.z[i] > 0; both += x && y; onlyC += x && !y; onlyG += y && !x; if (x && y) { const float sa = c.dispOffset-fb/a.z[i], sb = c.dispOffset-fb/b.z[i]; diffs.push_back(std::fabs(sa-sb)); diffsMm.push_back(std::fabs(a.z[i]-b.z[i])); } } };
        for (int i = 0; i < frames && readPair(argv[2], i*step, f); i++) {
            auto t0 = std::chrono::steady_clock::now(); cpu.compute(f.ref.data(), f.other.data(), f.width, f.height, c, dc); DepthMap dcD = dc; cpu.refineDetail(f.ref.data(), f.other.data(), f.width, f.height, c, dcD);
            auto t1 = std::chrono::steady_clock::now(); gpu.compute(f.ref.data(), f.other.data(), f.width, f.height, c, dg); StereoTimes t = gpu.times; DepthMap dgD = dg; gpu.refineDetail(f.ref.data(), f.other.data(), f.width, f.height, c, dgD);
            auto t2 = std::chrono::steady_clock::now();
            if (i) { msC += std::chrono::duration<double, std::milli>(t1-t0).count(); msG += std::chrono::duration<double, std::milli>(t2-t1).count(); tg.fine += t.fine; tg.region += t.region; tg.slant += t.slant; tg.detail += gpu.times.detail; }
            measure(dc, dg); measure(dcD, dgD); n++;
        }
        if (!gpu.usingGpu()) { printf("Grafikkarte nicht verwendet: %s\n", gpu.gpuInfo().c_str()); return 1; }
        auto pct = [](std::vector<float>& v, double p) { if (v.empty()) return 0.f; size_t k = std::min(v.size()-1, (size_t)(v.size()*p)); std::nth_element(v.begin(), v.begin()+k, v.end()); return v[k]; };
        printf("Karte: %s\nBilder %d  Punkte beide %zu, nur CPU %zu (%.3f %%), nur GPU %zu (%.3f %%)\n", gpu.gpuInfo().c_str(), n, both, onlyC, 100.0*onlyC/std::max<size_t>(1, both), onlyG, 100.0*onlyG/std::max<size_t>(1, both));
        const float s50 = pct(diffs, .5), s99 = pct(diffs, .99), s999 = pct(diffs, .999), sMaxD = pct(diffs, 1), z50 = pct(diffsMm, .5), z99 = pct(diffsMm, .99), z999 = pct(diffsMm, .999);
        printf("Versatz-Abweichung px: Median %.5f, p99 %.5f, p99.9 %.5f, max %.3f\nTiefe mm: Median %.5f, p99 %.5f, p99.9 %.5f\n", s50, s99, s999, sMaxD, z50, z99, z999);
        const int k = std::max(1, n-1); printf("Zeit ms/Bild: CPU %.1f, GPU %.1f (Grob+Fein %.1f, Region %.1f, Schraeg %.1f, Detail %.1f)\n", msC/k, msG/k, tg.fine/k, tg.region/k, tg.slant/k, tg.detail/k);
        return 0;
    }
    if (cmd == "abstand") {
        const int seconds = argc > 2 ? atoi(argv[2]) : 40; int expo = 3000;
        OtterDevice d; if (!openDev(d)) return 1;
        std::string e; if (!d.startStreams(e)) { fprintf(stderr, "Fehler: %s\n", e.c_str()); return 1; }
        d.setIrExposure(expo);
        Pipeline p; p.setTurntable(true); p.setProfile(profileFor(ScanSize::Small));
        StereoFrame f; int n = 0, misses = 0; const auto t0 = std::chrono::steady_clock::now(); double nextPrint = 0;
        puts("Sekunde  Anzeige  Szene  Objektbereich  gueltig  Marker  Unterlage  Belichtung  p99  Obj-Median Obj-p95 Obj-ueberstrahlt  Punkte");
        for (;;) {
            const double t = std::chrono::duration<double>(std::chrono::steady_clock::now()-t0).count(); if (t > seconds) break;
            if (!d.nextPair(f, 2000)) { if (++misses > 3) { fprintf(stderr, "Timeout\n"); return 1; } continue; }
            p.process(f, d.calib());
            if (++n % 10 == 0) { const int next = p.nextExposure(f, expo); if (next != expo) { expo = next; d.setIrExposure(expo); } }
            if (t < nextPrint) continue;
            nextPrint = t+.5;
            auto median = [](const DepthMap& m) { std::vector<float> s; for (size_t i = 0; i < m.z.size(); i += 4) if (m.z[i] > 0) s.push_back(m.z[i]); if (s.empty()) return 0.f; std::nth_element(s.begin(), s.begin()+s.size()/2, s.end()); return s[s.size()/2]; };
            std::vector<uint8_t> sorted(f.ref.begin(), f.ref.end()); std::nth_element(sorted.begin(), sorted.begin()+sorted.size()*99/100, sorted.end());
            const auto& s = p.stats();
            // Brightness on the object: the brightest pixel near each depth cell, which is the dot there.
            const DepthMap& ob = p.objectDepth(); std::vector<int> peaks; int cells = 0; const int st = 1280/std::max(1, ob.width);
            for (int y = 0; y < ob.height; y++) for (int x = 0; x < ob.width; x++) if (ob.z[(size_t)y*ob.width+x] > 0) { cells++; int m = 0; for (int dy = -4; dy <= 4; dy++) for (int dx = -4; dx <= 4; dx++) { const int X = st*x+dx, Y = st*y+dy; if (X >= 0 && Y >= 0 && X < f.width && Y < f.height) m = std::max<int>(m, f.ref[(size_t)Y*f.width+X]); } peaks.push_back(m); }
            int pm = 0, p95 = 0; float sat = 0; if (!peaks.empty()) { std::sort(peaks.begin(), peaks.end()); pm = peaks[peaks.size()/2]; p95 = peaks[peaks.size()*95/100]; int ns = 0; for (int v : peaks) ns += v >= 250; sat = 100.f*ns/peaks.size(); }
            printf("%6.1f  %6.0f  %5.0f  %12.0f  %6.1f%%  %6d  %9s  %10d  %3d  %9d %7d %14.0f%%  %6d\n", t, s.medianDepth, median(p.depth()), median(p.objectDepth()), 100*s.validDepth, s.markersDetected, s.baseFound ? "ja" : "nein", expo, sorted[sorted.size()*99/100], pm, p95, sat, cells);
            fflush(stdout);
        }
        return 0;
    }
    if (cmd == "record" && argc >= 4) {
        int frames = atoi(argv[3]), expo = argc > 4 ? atoi(argv[4]) : 3000;
        OtterDevice d; if (!openDev(d)) return 1;
        for (int a = 5; a < argc; a++) {
            if (std::string(argv[a]) == "fern") d.setCameraPair(CameraPair::Far);
            if (std::string(argv[a]) == "farbe") d.setColorEnabled(true);
        }
        bool laserOff = false, both = false; for (int a = 5; a < argc; a++) { laserOff |= std::string(argv[a]) == "laseraus"; both |= std::string(argv[a]) == "beides"; }
        if (both) {
            // Same view twice, the scanner must not move: first with the projector, then without, into <ordner>/an and <ordner>/aus.
            const int dark = argc > 4 ? std::max(expo, 12000) : 12000;
            std::string e; if (!d.startStreams(e)) { fprintf(stderr, "Fehler: %s\n", e.c_str()); return 1; }
            const std::string on = std::string(argv[2])+"/an", off = std::string(argv[2])+"/aus"; StereoFrame f;
            d.setIrExposure(expo); d.setLaser(true); writeCalib(on, d.calib()); for (int i = 0; i < 15; i++) d.nextPair(f, 1000);
            for (int i = 0; i < frames; i++) { if (!d.nextPair(f, 2000) || !writePair(on, i, f, e)) { fprintf(stderr, "Aufnahme fehlgeschlagen\n"); return 1; } }
            d.setLaser(false); d.setIrExposure(dark); writeCalib(off, d.calib()); for (int i = 0; i < 20; i++) d.nextPair(f, 1000);
            for (int i = 0; i < frames; i++) { if (!d.nextPair(f, 2000) || !writePair(off, i, f, e)) { fprintf(stderr, "Aufnahme fehlgeschlagen\n"); d.setLaser(true); return 1; } }
            d.setLaser(true); d.setIrExposure(expo);
            printf("%d Bildpaare mit und %d ohne Projektor in %s\n", frames, frames, argv[2]);
            return 0;
        }
        std::string e; if (!d.startStreams(e)) { fprintf(stderr, "Fehler: %s\n", e.c_str()); return 1; }
        d.setIrExposure(expo);
        if (laserOff) d.setLaser(false);   // only the board's own dots, for calibration
        writeCalib(argv[2], d.calib());
        StereoFrame f;
        for (int i = 0; i < 10; i++) d.nextPair(f, 1000);
        for (int i = 0; i < frames; i++) {
            if (!d.nextPair(f, 2000)) { fprintf(stderr, "Timeout\n"); return 1; }
            if (!writePair(argv[2], i, f, e)) { fprintf(stderr, "%s\n", e.c_str()); return 1; }
        }
        if (laserOff) d.setLaser(true);
        printf("%d Bildpaare in %s\n", frames, argv[2]);
        return 0;
    }
    if (cmd == "reconstruct" && argc >= 4) {
        StereoCalib c; if (!readCalib(argv[2], c)) { fprintf(stderr, "calib.txt fehlt\n"); return 1; }
        PipelineOptions po; if (argc > 4) po.voxelMm = (float)atof(argv[4]);
        if (getenv("VF_OHNE_MARKER")) po.useMarkers = false;
        if (getenv("VF_TV")) po.maxTrackVoxelMm = (float)atof(getenv("VF_TV"));
        Pipeline p(po); p.setVoxelQuality(po.voxelMm); p.setScanning(true);
        if (getenv("VF_DETAIL")) { p.stereo.params.slantRefine = true; p.options.detailDepth = true; }   // the sharp depth of the finest setting at any voxel size
        if (getenv("VF_GROB")) { p.stereo.params.slantRefine = false; p.options.detailDepth = false; }
        if (argc > 5 && std::string(argv[5]) == "drehteller") p.setTurntable(true);
        StereoFrame f; int n = 0; float ms = 0, st = 0;
        while (readPair(argv[2], n, f)) { p.process(f, c); ms += p.stats().totalMs; st += p.stats().stereoMs;
            if (getenv("VF_DEBUG") && n < 8) { const auto& q = p.stats(); size_t v = 0; for (float z : p.objectDepth().z) v += z > 0; printf("  Bild %d: Zustand %d Objektpunkte %zu Inlier %.2f Deckung %.2f Rest %.2f ICP %d Bereich %.0f mm\n", n, (int)q.state, v, q.inlierRatio, q.coverage, q.rmsMm, q.icpPoints, p.region().radius); }
            n++; }
        if (!n) { fprintf(stderr, "Keine Bildpaare\n"); return 1; }
        const auto& s = p.stats();
        printf("Bilder=%d integriert=%d verloren=%d  Gesamt %.1f ms/Bild (Stereo %.1f)  Bloecke=%zu  Speicher=%.0f MB\n", n, s.integratedFrames, s.lostFrames, ms/n, st/n, s.blocks, s.memoryBytes/1e6);
        if (const char* pf = getenv("VF_POSEN")) {   // camera centers in the model, one line per integrated frame
            ScanData sd; Mesh dummy; (void)dummy;
            if (p.takeScan(sd)) { if (FILE* o = fopen(pf, "w")) { for (const ScanFrame& fr : sd.frames) fprintf(o, "%f %f %f\n", fr.pose.t.x, fr.pose.t.y, fr.pose.t.z); fclose(o); } }
            return 0;
        }
        if (const char* nv = getenv("VF_NEU")) { std::string rep; p.rebuildModel((float)atof(nv), rep); printf("Neu berechnet mit %s mm\n", nv); }
        Mesh m; MeshOptions mo; mo.upright = true; if (getenv("VF_OHNE_FEIN")) mo.cleanUp = false; p.buildMesh(mo, m);
        V3 lo, hi; meshBounds(m, lo, hi);
        printf("Netz: %zu Dreiecke, Groesse %.1f x %.1f x %.1f mm\n", m.triangleCount(), hi.x-lo.x, hi.y-lo.y, hi.z-lo.z);
        std::string e; if (!saveMesh(m, argv[3], e)) { fprintf(stderr, "%s\n", e.c_str()); return 1; }
        printf("Gespeichert: %s\n", argv[3]);
        return 0;
    }
    if (cmd == "optimieren" && argc >= 4) {
        Mesh m; std::string e; OneClickConfig c;
        if (!loadMeshFile(argv[2], m, e)) { fprintf(stderr, "%s\n", e.c_str()); return 1; }
        if (argc > 4) c.maxHolePerimeterMm = (float)atof(argv[4]);   // < 0: alle Loecher
        const OneClickReport r = runOneClick(m, c, [](const char* step, float) { printf("  %s\n", step); fflush(stdout); });
        printf("%d Teile entfernt, %d Loecher gefuellt, %d offen, %zu -> %zu Dreiecke\n", r.partsRemoved, r.holesClosed, r.holesLeft, r.trianglesBefore, r.trianglesAfter);
        if (!saveMesh(m, argv[3], e)) { fprintf(stderr, "%s\n", e.c_str()); return 1; }
        printf("Gespeichert: %s\n", argv[3]);
        return 0;
    }
    if (cmd == "projekt" && argc >= 3) {
        Project p; std::string e; if (!loadProject(p, argv[2], e)) { fprintf(stderr, "%s\n", e.c_str()); return 1; }
        for (ScanData& s : p.scans) {
            if (removeSupport(s)) printf("%s: Unterlage gefunden und entfernt\n", s.name.c_str());
            buildOverview(s); const Table t = scanTable(s);
            const BasePlane b = fitBasePlane(s.markers, s.frames.empty() ? V3{} : s.frames[s.frames.size()/2].pose.t);
            int on = 0; for (const V3& m : s.markers) on += b.valid && std::fabs(b.height(m)) < 1.f;
            size_t below = 0; if (b.valid && s.overview) for (const V3& v : s.overview->vertices) below += b.height(v) < -2.f;
            float box[3] = {0, 0, 0}; if (s.overview) { Mesh m = *s.overview; orientUpright(m, s.up); tightBoxSize(m, box); }
            printf("%-12s %.0f x %.0f x %.0f mm  ", s.name.c_str(), box[0], box[1], box[2]);
            printf("%-12s %5zu Bilder  %3zu Marker  %s%s  up %.2f %.2f %.2f  Tisch %s (Ebene %s, %d auf der Ebene, %zu von %zu Punkten darunter)\n", s.name.c_str(), s.frames.size(), s.markers.size(), s.placed ? "ausgerichtet" : "offen", s.included ? "" : " (ausgelassen)", s.up.x, s.up.y, s.up.z,
                   t.known ? "ja" : "nein", b.valid ? "ja" : "nein", on, below, s.overview ? s.overview->vertices.size() : 0);
        }
        return 0;
    }
    if (cmd == "uebersicht" && argc >= 5) {   // overview of one scan, upright, as a file
        Project p; std::string e; if (!loadProject(p, argv[2], e)) { fprintf(stderr, "%s\n", e.c_str()); return 1; }
        ScanData& s = p.scans[(size_t)atoi(argv[3])-1]; if (!getenv("VF_ROH")) removeSupport(s); buildOverview(s); Mesh m = *s.overview; m.colors.clear();
        const Table t = scanTable(s); orientUpright(m, t.known ? t.up : s.up); computeNormals(m);
        if (!saveMesh(m, argv[4], e)) { fprintf(stderr, "%s\n", e.c_str()); return 1; }
        return 0;
    }
    if (cmd == "verfolgtest" && argc >= 3) {
        // The frames of a marker scan, played back without markers: tracking by shape alone, measured against the marker poses.
        Project p; std::string e; if (!loadProject(p, argv[2], e)) { fprintf(stderr, "%s\n", e.c_str()); return 1; }
        const float voxel = getenv("VF_VOXEL") ? (float)atof(getenv("VF_VOXEL")) : 0.5f;
        for (size_t si = 0; si < p.scans.size(); si++) {
            if (argc > 3 && (size_t)atoi(argv[3]) != si+1) continue;
            const ScanData& s = p.scans[si]; if (s.markers.size() < 5 || s.frames.size() < 10) { printf("%s: ohne Marker, kein Sollwert\n", s.name.c_str()); continue; }
            PipelineOptions o; o.voxelMm = voxel; o.useMarkers = false; if (getenv("VF_TV")) o.maxTrackVoxelMm = (float)atof(getenv("VF_TV")); Pipeline pipe(o); pipe.setScanning(true);
            DepthMap d = s.grid; d.z.assign((size_t)d.width*d.height, 0.f);
            // Object center, in the coordinates of the first frame.
            V3 c{}; size_t nc = 0; { const ScanFrame& f0 = s.frames[0]; for (size_t i = 0; i < f0.cells.size(); i += 7) { const int x = (int)(f0.cells[i]%d.width), y = (int)(f0.cells[i]/d.width); const float z = f0.z[i]; c = c+V3{(x-d.cx)/d.fx*z, (y-d.cy)/d.fy*z, z}; nc++; } c = c*(1.f/std::max<size_t>(1, nc)); }
            const Pose t0inv = inversePose(s.frames[0].pose);
            double sumDeg = 0, sumMm = 0; float maxDeg = 0, maxMm = 0, lastDeg = 0, lastMm = 0; int lost = 0, n = 0, warned = 0; const auto t0 = std::chrono::steady_clock::now();
            for (size_t i = 0; i < s.frames.size(); i++) {
                std::fill(d.z.begin(), d.z.end(), 0.f); const ScanFrame& f = s.frames[i]; for (size_t k = 0; k < f.cells.size(); k++) d.z[f.cells[k]] = f.z[k];
                pipe.trackAndIntegrate(d);
                if (getenv("VF_VERLAUF")) { static int lastState = -1; const int st = (int)pipe.stats().state; const Pose tr = mul(t0inv, f.pose), es = pipe.pose(); const Pose df = mul(inversePose(tr), es); const float a = std::acos(std::clamp((df.R[0]+df.R[4]+df.R[8]-1)/2, -1.f, 1.f))*57.2958f;
                    if (st != lastState || a > 4 || i % 100 == 0) printf("  %4zu Zustand %d  Fehler %6.2f Grad  Inlier %.2f Deckung %.2f Rest %.2f  fest %.3f Wiederaufn. %d\n", i, st, a, pipe.stats().inlierRatio, pipe.stats().coverage, pipe.stats().rmsMm, pipe.stats().weakest, pipe.stats().relocations); lastState = st; }
                if (getenv("VF_DEBUG") && i < 6) { size_t v = 0; for (float z : d.z) v += z > 0; printf("  Bild %zu: %zu Punkte, Zustand %d, Inlier %.2f, Rest %.2f, Punkte ICP %d\n", i, v, (int)pipe.stats().state, pipe.stats().inlierRatio, pipe.stats().rmsMm, pipe.stats().icpPoints); }
                if (getenv("VF_FEST") && pipe.stats().state == TrackState::Good) { static std::vector<float> w; w.push_back(pipe.stats().weakest); if (i+1 == s.frames.size()) { std::sort(w.begin(), w.end()); printf("  fest: 1%% %.3f  5%% %.3f  50%% %.3f\n", w[w.size()/100], w[w.size()/20], w[w.size()/2]); w.clear(); } }
                warned += pipe.stats().shapeWeak;
                if (const char* vp = getenv("VF_VORSCHAU")) if (i == s.frames.size()/4 || i == s.frames.size()/2 || i+1 == s.frames.size()) {
                    std::vector<uint8_t> rgba; pipe.renderPreview(640, 400, rgba, true); const std::string path = std::string(vp)+"_"+std::to_string(i)+".ppm";
                    if (FILE* o = fopen(path.c_str(), "wb")) { fprintf(o, "P6 640 400 255\n"); for (size_t k = 0; k < rgba.size(); k += 4) { const uint8_t px[3] = {rgba[k+3] ? rgba[k] : (uint8_t)20, rgba[k+3] ? rgba[k+1] : (uint8_t)24, rgba[k+3] ? rgba[k+2] : (uint8_t)32}; fwrite(px, 1, 3, o); } fclose(o); }
                }
                if (pipe.stats().state != TrackState::Good) { lost++; continue; }
                const Pose truth = mul(t0inv, f.pose), est = pipe.pose();
                const Pose diff = mul(inversePose(truth), est); const float tr = diff.R[0]+diff.R[4]+diff.R[8];
                const float deg = std::acos(std::clamp((tr-1)/2, -1.f, 1.f))*57.2958f, mm = norm(truth.applyInv(c)-est.applyInv(c));
                sumDeg += deg; sumMm += mm; maxDeg = std::max(maxDeg, deg); maxMm = std::max(maxMm, mm); lastDeg = deg; lastMm = mm; n++;
            }
            const double dt = std::chrono::duration<double>(std::chrono::steady_clock::now()-t0).count();
            printf("%-8s %5zu Bilder  verloren %4d  Winkel mittel %5.2f max %6.2f Ende %6.2f Grad  Mitte mittel %6.2f max %7.2f Ende %7.2f mm  Warnung %4d  %.1f ms/Bild\n", s.name.c_str(), s.frames.size(), lost, n ? sumDeg/n : 0, maxDeg, lastDeg, n ? sumMm/n : 0, maxMm, lastMm, warned, 1000*dt/s.frames.size());
            fflush(stdout);
        }
        return 0;
    }
    if (cmd == "ausrichten" && argc >= 3) {
        Project p; std::string e; if (!loadProject(p, argv[2], e)) { fprintf(stderr, "%s\n", e.c_str()); return 1; }
        for (ScanData& s : p.scans) { if (removeSupport(s)) printf("%s: Unterlage entfernt (oben %.2f %.2f %.2f)\n", s.name.c_str(), s.support.up.x, s.support.up.y, s.support.up.z); buildOverview(s); }
        size_t ref = 0; for (size_t i = 0; i < p.scans.size(); i++) if (p.scans[i].placed) { ref = i; break; }
        std::vector<size_t> order; for (size_t i = 0; i < p.scans.size(); i++) if (i != ref) order.push_back(i);
        if (argc > 5) { ref = (size_t)atoi(argv[4])-1; order = {(size_t)atoi(argv[5])-1}; }
        const Pose refPose = p.scans[ref].placed ? p.scans[ref].placement : Pose{};
        Mesh merged; auto add = [&](const ScanData& s, const Pose& P, Mesh& into, const float* col) {
            const uint32_t off = (uint32_t)into.vertices.size();
            for (const V3& v : s.overview->vertices) { into.vertices.push_back(P.apply(v)); if (col) for (int c = 0; c < 3; c++) into.colors.push_back((uint8_t)(col[c]*255)); }
            for (uint32_t i : s.overview->indices) into.indices.push_back(i+off); };
        const float cols[4][3] = {{.25f, .62f, .95f}, {.98f, .58f, .16f}, {.3f, .85f, .45f}, {.9f, .35f, .75f}};
        Mesh show; add(p.scans[ref], refPose, merged, nullptr); add(p.scans[ref], refPose, show, cols[0]); computeNormals(merged);
        const Table refTable = placedTable(scanTable(p.scans[ref]), refPose);
        for (size_t k = 0; k < order.size(); k++) {
            const size_t i = order[k]; ScanData& s = p.scans[i]; Mesh src = *s.overview; src.colors.clear(); computeNormals(src);
            const auto t0 = std::chrono::steady_clock::now();
            const auto r = registerAuto(src, merged, {}, scanTable(s), refTable);
            const double dt = std::chrono::duration<double>(std::chrono::steady_clock::now()-t0).count();
            printf("%s -> %s: %zu Vorschlaege, %.1f s\n", s.name.c_str(), p.scans[ref].name.c_str(), r.size(), dt);
            for (size_t j = 0; j < r.size(); j++) printf("  %zu: Ueberl. %3.0f%%  Rest %.3f  Widerspr. %4.1f%%  fest %.3f  %s%s  Punktzahl %.3f\n", j+1, 100*r[j].overlap, r[j].rmsMm, 100*r[j].conflict, r[j].pinned, r[j].confident ? "SICHER" : "", r[j].agreed ? " (beide Suchen)" : "", r[j].score);
            if (r.empty()) continue;
            const size_t pick = getenv("VF_VORSCHLAG") ? std::min(r.size(), (size_t)atoi(getenv("VF_VORSCHLAG")))-1 : 0;
            add(s, r[pick].srcToDst, merged, nullptr); computeNormals(merged); add(s, r[pick].srcToDst, show, cols[(k+1)%4]);
            s.placement = r[pick].srcToDst; s.placed = true;
        }
        if (argc > 3) { computeNormals(show); if (!saveMesh(show, argv[3], e)) fprintf(stderr, "%s\n", e.c_str()); else printf("Gespeichert: %s\n", argv[3]); }
        // With VF_FUSION=file.stl: the placements found are taken and all scans fused, as the fusion dialog does.
        if (const char* out = getenv("VF_FUSION")) {
            for (size_t k = 0; k < order.size(); k++) { (void)k; }
            Mesh fused; MeshOptions o; o.minWeight = 3; o.removeFragments = true; o.cleanUp = true; o.fillHolesMm = 0; o.keepTriangles = 1; o.upright = true;
            const float vox = getenv("VF_VOXEL") ? (float)atof(getenv("VF_VOXEL")) : 0.25f;
            if (!fuseProject(p, vox, o, fused, e)) { fprintf(stderr, "%s\n", e.c_str()); return 1; }
            repairManifold(fused);
            if (!saveMesh(fused, out, e)) fprintf(stderr, "%s\n", e.c_str()); else printf("Fusioniert (%.2f mm): %s\n", vox, out);
        }
        return 0;
    }
    if (cmd == "ausrichttest" && argc >= 3) {
        const std::string file = argv[2]; const int rounds = argc > 3 ? atoi(argv[3]) : 3; std::vector<BenchCase> cases; std::mt19937 g(42);
        if (file.size() > 7 && file.substr(file.size()-7) == ".vfproj") {
            // Real scans, placed by a person: every placed pair, the moving one turned at random.
            Project p; std::string e; if (!loadProject(p, file, e)) { fprintf(stderr, "%s\n", e.c_str()); return 1; }
            for (ScanData& s : p.scans) buildOverview(s);
            for (size_t i = 0; i < p.scans.size(); i++) for (size_t j = 0; j < p.scans.size(); j++) {
                if (i == j || !p.scans[i].placed || !p.scans[j].placed || !p.scans[i].overview || !p.scans[j].overview) continue;
                const Pose rel = mul(inversePose(p.scans[i].placement), p.scans[j].placement);   // j into i
                for (int k = 0; k < rounds; k++) {
                    BenchCase c; c.name = p.scans[j].name+" -> "+p.scans[i].name+(scanTable(p.scans[j]).known && scanTable(p.scans[i]).known ? " T" : ""); const Pose T = randomPose(g, 100);
                    c.src = *p.scans[j].overview; c.src.colors.clear(); for (V3& v : c.src.vertices) v = T.apply(v); computeNormals(c.src);
                    c.dst = *p.scans[i].overview; c.dst.colors.clear(); computeNormals(c.dst);
                    c.truth = mul(rel, inversePose(T)); c.srcUp = T.rot(p.scans[j].up); c.dstUp = p.scans[i].up;
                    if (!getenv("VF_OHNE_TISCH")) { c.srcTable = placedTable(scanTable(p.scans[j]), T); c.dstTable = scanTable(p.scans[i]); }
                    cases.push_back(std::move(c));
                }
            }
        } else {
            // A finished model with Z up, cut into what two scans would see: upright and turned over, on its side, or only turned.
            Mesh m; std::string e; if (file == "rinne") m = trackModel(); else if (!loadMeshFile(file, m, e)) { fprintf(stderr, "%s\n", e.c_str()); return 1; }
            if (m.triangleCount() > 300000) simplifyMesh(m, 300000);
            computeNormals(m);
            struct Cut { const char* name; V3 up; };
            const Cut cuts[] = {{"umgedreht", {0, 0, -1}}, {"auf der Seite +x", {1, 0, 0}}, {"auf der Seite -y", {0, -1, 0}}};
            std::normal_distribution<float> noise(0, 0.05f);
            const Mesh upright = scannedPart(m, {0, 0, 1}, 0.8f);
            for (const Cut& cut : cuts) {
                const Mesh other = scannedPart(m, cut.up, 0.8f);
                for (int k = 0; k < rounds; k++) {
                    BenchCase c; c.name = cut.name; c.dst = upright; c.src = other;
                    const Pose T = randomPose(g, 100); for (V3& v : c.src.vertices) v = T.apply(v+V3{noise(g), noise(g), noise(g)}); computeNormals(c.src);
                    c.truth = inversePose(T); c.dstUp = {0, 0, 1}; c.srcUp = T.rot(cut.up);
                    // Both stood on a table just below their lowest point.
                    if (!getenv("VF_OHNE_TISCH")) {
                        float lo = 1e9f; for (const V3& v : m.vertices) lo = std::min(lo, v.z); c.dstTable = {true, {0, 0, 1}, lo};
                        float ls = 1e9f; for (const V3& v : m.vertices) ls = std::min(ls, dot(v, cut.up)); c.srcTable = placedTable(Table{true, cut.up, ls}, T);
                    }
                    cases.push_back(std::move(c));
                }
            }
        }
        runBench(cases);
        return 0;
    }
    if (cmd == "tafel" && argc >= 3) {
        const float pitch = getenv("VF_RASTER") ? (float)atof(getenv("VF_RASTER")) : 30.f;
        std::vector<BoardMeasurement> all; StereoCalib calib;
        for (int a = 2; a < argc; a++) {
            if (!readCalib(argv[a], calib)) { fprintf(stderr, "%s: calib.txt fehlt\n", argv[a]); continue; }
            Pipeline p; p.setProfile(profileFor(ScanSize::Big)); StereoFrame f; std::vector<BoardMeasurement> ms;
            for (int i = 0; readPair(argv[a], i, f); i += 3) {
                // Dim without the projector: then the dots alone, from both images.
                double mean = 0; for (size_t k = 0; k < f.ref.size(); k += 97) mean += f.ref[k]; mean /= f.ref.size()/97.0;
                BoardMeasurement m; if (mean < 40) m = measureBoardStereo(f, calib, pitch); else { p.process(f, calib); m = measureBoard(f, p.depth(), calib, pitch); }
                if (m.ok) ms.push_back(m);
                if (getenv("VF_DEBUG") && i == 0) printf("  Bild 0: %s, %d Punkte, %d Paare\n", m.ok ? "ok" : "nicht gefunden", m.dots, m.pairs);
                if (ms.size() >= 20) break;
            }
            if (ms.empty()) { printf("%-40s Platte nicht gefunden\n", argv[a]); continue; }
            auto med = [&](auto get) { std::vector<float> v; for (auto& m : ms) v.push_back(get(m)); std::nth_element(v.begin(), v.begin()+v.size()/2, v.end()); return v[v.size()/2]; };
            BoardMeasurement r = ms[0]; r.spacingMm = med([](auto& m) { return m.spacingMm; }); r.distanceMm = med([](auto& m) { return m.distanceMm; }); r.shift = med([](auto& m) { return m.shift; });
            r.flatnessMm = med([](auto& m) { return m.flatnessMm; }); r.tiltDeg = med([](auto& m) { return m.tiltDeg; }); r.dots = (int)med([](auto& m) { return (float)m.dots; });
            float lo = 1e9f, hi = 0; for (auto& m : ms) { lo = std::min(lo, m.spacingMm); hi = std::max(hi, m.spacingMm); }
            printf("%-40s %3zu Bilder  Abstand %6.1f mm  Neigung %4.1f Grad  %3d Punkte  Raster %7.3f mm (%+.2f %%, Bilder %.3f..%.3f)  Ebenheit %.3f mm\n", argv[a], ms.size(), r.distanceMm, r.tiltDeg, r.dots, r.spacingMm, 100*(r.spacingMm/pitch-1), lo, hi, r.flatnessMm);
            all.push_back(r);
        }
        float scale = 1, offset = 0, res = 0;
        if (all.size() >= 2 && fitStereoCorrection(all, calib, pitch, scale, offset, res))
            printf("\nKorrektur: Massstab %.5f, Versatz der Disparitaet %+.2f px, Rest %.3f %% des Rasters\n", scale, offset, res);
        return 0;
    }
    if (cmd == "tafelvergleich" && argc >= 3) {
        // The same view with and without the projector: the dots give the board's true plane, the dense depth its own.
        const std::string on = std::string(argv[2])+"/an", off = std::string(argv[2])+"/aus"; StereoCalib calib;
        if (!readCalib(on, calib)) { fprintf(stderr, "calib.txt fehlt\n"); return 1; }
        std::vector<V3> truth; StereoFrame f; int n = 0;
        for (int i = 0; readPair(off, i, f) && n < 20; i += 2, n++) { const BoardMeasurement m = measureBoardStereo(f, calib, 30.f); if (m.ok) truth.insert(truth.end(), m.centers.begin(), m.centers.end()); }
        if (truth.size() < 20) { fprintf(stderr, "Platte ohne Projektor nicht gefunden\n"); return 1; }
        // Plane through all dot centers of all frames: the scanner stood still.
        auto fitPlane = [](const std::vector<V3>& pts, V3& nrm, float& h, float& rms) {
            V3 c{}; for (const V3& p : pts) c = c+p; c = c*(1.f/pts.size());
            double A[3][3] = {{0}}; for (const V3& p : pts) { const V3 q = p-c; const double w3[3] = {q.x, q.y, q.z}; for (int r = 0; r < 3; r++) for (int k = 0; k < 3; k++) A[r][k] += w3[r]*w3[k]; }
            const double tr = A[0][0]+A[1][1]+A[2][2]; double x[3] = {0, 0, 1};
            for (int it = 0; it < 100; it++) { double y[3]; for (int r = 0; r < 3; r++) { y[r] = tr*x[r]; for (int k = 0; k < 3; k++) y[r] -= A[r][k]*x[k]; } const double l = std::sqrt(y[0]*y[0]+y[1]*y[1]+y[2]*y[2]); for (int r = 0; r < 3; r++) x[r] = y[r]/l; }
            nrm = normalized(V3{(float)x[0], (float)x[1], (float)x[2]}); if (nrm.z > 0) nrm = nrm*-1.f; h = dot(nrm, c);
            double e = 0; for (const V3& p : pts) { const float r = dot(nrm, p)-h; e += r*r; } rms = (float)std::sqrt(e/pts.size()); };
        V3 nt; float ht, rt; fitPlane(truth, nt, ht, rt);
        // The dense depth in the region of the dots, frames averaged per cell, then mirrored back to the stereo formula's x.
        Pipeline p; p.setProfile(profileFor(ScanSize::Big)); std::vector<V3> dense; n = 0;
        float x0 = 1e9f, x1 = -1e9f, y0 = 1e9f, y1 = -1e9f; for (const V3& q : truth) { x0 = std::min(x0, q.x); x1 = std::max(x1, q.x); y0 = std::min(y0, q.y); y1 = std::max(y1, q.y); }
        for (int i = 0; readPair(on, i, f) && n < 10; i += 3, n++) {
            p.process(f, calib); const DepthMap& d = p.depth();
            for (int j = 0; j < d.height; j++) for (int k = 0; k < d.width; k++) {
                const float z = d.z[(size_t)j*d.width+k]; if (z <= 0) continue;
                // In the stereo formula's frame: x = (u - cxRef) z / f, unmirrored, at full resolution.
                const float u = k*(float)f.width/d.width, v = j*(float)f.height/d.height;
                const V3 q{(u-calib.cxRef())*z/calib.focalPx, (v-calib.cy())*z/calib.focalPx, z};
                if (q.x < x0 || q.x > x1 || q.y < y0 || q.y > y1) continue;
                if (std::fabs(dot(nt, q)-ht) < 5.f) dense.push_back(q);
            }
        }
        if (dense.size() < 1000) { fprintf(stderr, "Keine dichte Tiefe auf der Platte\n"); return 1; }
        V3 nd; float hd, rd; fitPlane(dense, nd, hd, rd);
        // Offset along the true normal at the middle of the dots, and the angle between both planes.
        V3 mid{}; for (const V3& q : truth) mid = mid+q; mid = mid*(1.f/truth.size());
        const float along = dot(nd, mid)-hd;   // how far the dense plane lies from the dots' middle, along its normal
        const float view = std::acos(std::min(1.f, std::fabs(dot(nt, normalized(mid)))))*57.2958f;
        printf("Abstand %.1f mm, Blickwinkel zur Platte %.1f Grad, %zu Punkte ohne / %zu Tiefenwerte mit Projektor\n", norm(mid), view, truth.size(), dense.size());
        printf("Dichte Tiefe gegen Punkte: Versatz %+.3f mm (positiv: zur Kamera hin), Winkel %.3f Grad, Rauschen %.3f mm (Punkte %.3f mm)\n", -along, std::acos(std::min(1.f, std::fabs(dot(nt, nd))))*57.2958f, rd, rt);
        return 0;
    }
    if (cmd == "messen" && argc >= 3) {
        Mesh m; std::string e;
        if (!loadStl(argv[2], m, e)) { fprintf(stderr, "%s\n", e.c_str()); return 1; }
        BoxReport r = measureBox(m);
        { float box[3]; tightBoxSize(m, box); printf("Kleinste Box (Z oben): %.2f x %.2f x %.2f mm\n", box[0], box[1], box[2]); }
        printf("%zu Dreiecke, %zu ebene Flaechen gefunden\n\n", m.triangleCount(), r.planes.size());
        for (size_t i = 0; i < r.planes.size(); i++)
            printf("Flaeche %zu: %8.0f mm2, Ebenheit %.3f mm\n", i+1, r.planes[i].areaMm2, r.planes[i].flatnessRmsMm);
        printf("\nKantenlaengen (Abstand gegenueberliegender Flaechen):\n"); int nd = 0;
        for (auto& q : r.pairs) if (q.parallel) { printf("  Flaeche %d-%d: %7.2f mm  (Parallelitaet %.2f Grad)\n", q.a+1, q.b+1, q.distanceMm, q.angleDeg); nd++; }
        if (!nd) puts("  keine gegenueberliegenden Flaechen gefunden");
        printf("\nWinkel zwischen angrenzenden Flaechen (Soll 90):\n");
        for (auto& q : r.pairs) if (!q.parallel) printf("  Flaeche %d-%d: %6.2f Grad\n", q.a+1, q.b+1, q.angleDeg);
        return 0;
    }
    return usage();
}
