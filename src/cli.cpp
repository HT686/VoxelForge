// Command line tool: device info, live depth check, recording, offline reconstruction.
#include <algorithm>
#include <chrono>
#include <vector>
#include <cstdio>
#include <string>
#include "device.hpp"
#include "measure.hpp"
#include "meshedit.hpp"
#include "pipeline.hpp"
#include "source.hpp"
#include <cmath>

static int usage() {
    puts("voxelforge-cli info\n"
         "voxelforge-cli depth [frames] [exposure] [fern]    Tiefe live messen, Statistik ausgeben\n"
         "voxelforge-cli record <ordner> <frames> [exposure] [fern] [farbe]\n"
         "voxelforge-cli reconstruct <ordner> <ausgabe.stl|obj|ply> [voxel_mm] [drehteller]\n"
         "voxelforge-cli optimieren <ein.stl|obj|ply> <aus.stl|obj|ply> [lochumfang_mm, -1 = alle]   Ein-Klick-Prozess\n"
         "voxelforge-cli abstand [sekunden]             Abstand wie in der App mitschreiben (Profil Klein)\n"
         "voxelforge-cli vergleich <ordner> [bilder] [voxel_mm]  Stereo auf CPU und Grafikkarte Bild fuer Bild vergleichen\n"
         "voxelforge-cli messen <scan.stl>              Flaechen, Kantenlaengen und Winkel eines Quaders");
    return 2;
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
        std::string e; if (!d.startStreams(e)) { fprintf(stderr, "Fehler: %s\n", e.c_str()); return 1; }
        d.setIrExposure(expo);
        writeCalib(argv[2], d.calib());
        StereoFrame f;
        for (int i = 0; i < 10; i++) d.nextPair(f, 1000);
        for (int i = 0; i < frames; i++) {
            if (!d.nextPair(f, 2000)) { fprintf(stderr, "Timeout\n"); return 1; }
            if (!writePair(argv[2], i, f, e)) { fprintf(stderr, "%s\n", e.c_str()); return 1; }
        }
        printf("%d Bildpaare in %s\n", frames, argv[2]);
        return 0;
    }
    if (cmd == "reconstruct" && argc >= 4) {
        StereoCalib c; if (!readCalib(argv[2], c)) { fprintf(stderr, "calib.txt fehlt\n"); return 1; }
        PipelineOptions po; if (argc > 4) po.voxelMm = (float)atof(argv[4]);
        Pipeline p(po); p.setVoxelQuality(po.voxelMm); p.setScanning(true);
        if (argc > 5 && std::string(argv[5]) == "drehteller") p.setTurntable(true);
        StereoFrame f; int n = 0; float ms = 0, st = 0;
        while (readPair(argv[2], n, f)) { p.process(f, c); ms += p.stats().totalMs; st += p.stats().stereoMs; n++; }
        if (!n) { fprintf(stderr, "Keine Bildpaare\n"); return 1; }
        const auto& s = p.stats();
        printf("Bilder=%d integriert=%d verloren=%d  Gesamt %.1f ms/Bild (Stereo %.1f)  Bloecke=%zu  Speicher=%.0f MB\n", n, s.integratedFrames, s.lostFrames, ms/n, st/n, s.blocks, s.memoryBytes/1e6);
        Mesh m; MeshOptions mo; mo.upright = true; p.buildMesh(mo, m);
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
    if (cmd == "messen" && argc >= 3) {
        Mesh m; std::string e;
        if (!loadStl(argv[2], m, e)) { fprintf(stderr, "%s\n", e.c_str()); return 1; }
        BoxReport r = measureBox(m);
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
