// vsbass_test.cpp
// Offline checks for the TB VS Bass 100 DSP. Builds on Linux or macOS:
//   make -C Tests run      (writes demo WAVs to Tests/renders)
//
// 1. Small-signal behaviour of the preamp: Gain, Bass Boost, Bass, Treble,
//    Middle, Presence
// 2. Valve Sound
// 3. Mains switch (dry pass-through, exact), Cab switch, switches while playing
// 4. Power amp into both cabs: clipping level, current limit, sag and recovery,
//    load impedance
// 5. Stability sweep: control settings x 10 mV-4 V x 41 Hz-15 kHz, plus loud noise
// 6. Automation of every control while playing
// 7. CPU per second of audio
// 8. Demo renders (32-bit float WAV)

#include "../Source/DSP/BassAmp.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <algorithm>
#include <ctime>
#include <string>
#include <vector>

using namespace hhvb;

static const double kPiT = 3.14159265358979323846;
static const int kSr = 44100;
static int failures = 0;

static void check (bool ok, const char* what)
{
    std::printf ("  %s  %s\n", ok ? "ok  " : "FAIL", what);
    if (! ok) ++failures;
}

static bool allFinite (const std::vector<float>& x, double limit)
{
    for (size_t i = 0; i < x.size(); ++i)
        if (! (x[i] == x[i]) || std::fabs (x[i]) > limit) return false;
    return true;
}

static double peakFrom (const std::vector<float>& x, size_t from)
{
    double p = 0.0;
    for (size_t i = from; i < x.size(); ++i) p = std::max (p, (double) std::fabs (x[i]));
    return p;
}

static double rmsFrom (const std::vector<float>& x, size_t from)
{
    double s = 0.0;
    for (size_t i = from; i < x.size(); ++i) s += (double) x[i] * x[i];
    return std::sqrt (s / (double) (x.size() - from));
}

static std::vector<float> sine (double f, double amp, double sec)
{
    std::vector<float> v ((size_t) (sec * kSr));
    for (size_t i = 0; i < v.size(); ++i) v[i] = (float) (amp * std::sin (2.0 * kPiT * f * i / kSr));
    return v;
}

static std::vector<float> run (BassAmp& a, const std::vector<float>& in)
{
    std::vector<float> out (in.size());
    const int block = 256;
    for (size_t pos = 0; pos < in.size(); pos += block)
    {
        const int n = (int) ((in.size() - pos) < (size_t) block ? (in.size() - pos) : block);
        a.process (&in[pos], &out[pos], n);
    }
    return out;
}

// Preamp small-signal gains (dB) at f, run at the oversampled rate:
// IC2 output and module output (before the Master Volume).
struct PreGains { double ic2, mod; };

static PreGains pre (BassAmp& a, double f, double amp = 0.001)
{
    a.resetPreamp();
    const double fs = a.oversampledRate();
    const int n = (int) (fs * (f < 100.0 ? 1.0 : 0.5));
    double p1 = 0.0, pm = 0.0;
    for (int i = 0; i < n; ++i)
    {
        double c, m;
        a.probePreamp (amp * std::sin (2.0 * kPiT * f * i / fs), c, m);
        if (i > n / 2)
        {
            p1 = std::max (p1, std::fabs (c));
            pm = std::max (pm, std::fabs (m));
        }
    }
    PreGains g;
    g.ic2 = 20.0 * std::log10 (p1 / amp + 1e-30);
    g.mod = 20.0 * std::log10 (pm / amp + 1e-30);
    return g;
}

// Magnitude of harmonic h of f in the module output (relative to the
// fundamental, dB), at the oversampled rate.
static double harmonicDb (BassAmp& a, double f, double amp, int h)
{
    a.resetPreamp();
    const double fs = a.oversampledRate();
    const int settle = (int) (fs * 0.5);
    const int periods = 20;
    const int n = (int) (fs / f * periods + 0.5);
    double re1 = 0.0, im1 = 0.0, reh = 0.0, imh = 0.0;
    for (int i = 0; i < settle + n; ++i)
    {
        double c, m;
        a.probePreamp (amp * std::sin (2.0 * kPiT * f * i / fs), c, m);
        if (i >= settle)
        {
            const double ph = 2.0 * kPiT * f * (i - settle) / fs;
            re1 += m * std::cos (ph); im1 += m * std::sin (ph);
            reh += m * std::cos (h * ph); imh += m * std::sin (h * ph);
        }
    }
    return 10.0 * std::log10 ((reh * reh + imh * imh + 1e-30) / (re1 * re1 + im1 * im1 + 1e-30));
}

static void writeWav (const std::string& path, const std::vector<float>& x, int sr)
{
    FILE* f = std::fopen (path.c_str(), "wb");
    if (! f) { std::printf ("  could not write %s\n", path.c_str()); return; }
    const unsigned int dataBytes = (unsigned int) (x.size() * 4);
    const unsigned int riff = 36 + dataBytes;
    const unsigned short fmt = 3, ch = 1, bits = 32, align = 4;
    const unsigned int rate = (unsigned int) sr, bps = rate * 4, fmtLen = 16;
    std::fwrite ("RIFF", 1, 4, f); std::fwrite (&riff, 4, 1, f);
    std::fwrite ("WAVEfmt ", 1, 8, f); std::fwrite (&fmtLen, 4, 1, f);
    std::fwrite (&fmt, 2, 1, f); std::fwrite (&ch, 2, 1, f);
    std::fwrite (&rate, 4, 1, f); std::fwrite (&bps, 4, 1, f);
    std::fwrite (&align, 2, 1, f); std::fwrite (&bits, 2, 1, f);
    std::fwrite ("data", 1, 4, f); std::fwrite (&dataBytes, 4, 1, f);
    std::fwrite (&x[0], 4, x.size(), f);
    std::fclose (f);
}

// Karplus-Strong plucked string as a stand-in for a DI bass track.
static void pluck (std::vector<float>& buf, size_t start, double freq, double amp, double dur)
{
    const int period = (int) (kSr / freq + 0.5);
    std::vector<double> line (period);
    unsigned int seed = (unsigned int) (freq * 1000.0);
    for (int i = 0; i < period; ++i)
    {
        seed = seed * 1664525u + 1013904223u;
        line[i] = ((seed >> 8) / 16777216.0 * 2.0 - 1.0) * amp;
    }
    // a softer pluck: smooth the excitation a little
    for (int p = 0; p < 2; ++p)
        for (int i = 1; i < period; ++i) line[i] = 0.5 * (line[i] + line[i - 1]);
    const size_t n = (size_t) (dur * kSr);
    int pos = 0;
    for (size_t i = 0; i < n && start + i < buf.size(); ++i)
    {
        const int nx = (pos + 1) % period;
        const double v = line[pos];
        line[pos] = 0.4995 * (line[pos] + line[nx]);
        pos = nx;
        buf[start + i] += (float) v;
    }
}

static std::vector<float> riff (double amp)
{
    std::vector<float> g ((size_t) (6.0 * kSr), 0.0f);
    const double notes[] = { 41.2, 41.2, 55.0, 61.7, 41.2, 73.4, 82.4, 49.0, 55.0, 61.7, 41.2 };
    for (int k = 0; k < 11; ++k)
        pluck (g, (size_t) (k * 0.4 * kSr), notes[k], amp, 0.5);
    pluck (g, (size_t) (4.5 * kSr), 41.2, amp * 1.2, 1.5);
    return g;
}

//==============================================================================
static void testPreamp()
{
    std::printf ("1. Preamp small-signal behaviour\n");

    {
        BassAmp a;
        a.prepare (kSr);
        check (a.circuitsBuilt(), "every netlist built with all its nodes");
    }

    // Gain: monotonic with a wide range (IC1 dual action)
    {
        double prev = -1e9, g0 = 0.0, g10 = 0.0;
        bool mono = true;
        for (int s = 0; s <= 10; ++s)
        {
            BassAmp a;
            a.setGain (s / 10.0);
            a.prepare (kSr);
            const double v = pre (a, 400.0, 1.0e-5).ic2;
            if (v <= prev) mono = false;
            prev = v;
            if (s == 0) g0 = v;
            if (s == 10) g10 = v;
        }
        std::printf ("    Gain 0 -> 10 @400 Hz, IC2 out: %.1f -> %.1f dB\n", g0, g10);
        check (mono && g10 - g0 > 40.0, "Gain monotonic over 0..10, range > 40 dB");
    }

    // Bass and Treble (passive Baxandall, 100K lin)
    {
        double lo[3], hi[3], tl[3], th[3];
        for (int s = 0; s < 3; ++s)
        {
            BassAmp b, t;
            b.setGain (0.3); t.setGain (0.3);
            b.setBass (s * 0.5); t.setTreble (s * 0.5);
            b.prepare (kSr); t.prepare (kSr);
            lo[s] = pre (b, 40.0).ic2;  hi[s] = pre (b, 2000.0).ic2;
            th[s] = pre (t, 6000.0).ic2; tl[s] = pre (t, 100.0).ic2;
        }
        std::printf ("    bass -5/0/+5 @40 Hz: %.1f %.1f %.1f dB (2 kHz %.1f..%.1f)\n", lo[0], lo[1], lo[2], hi[0], hi[2]);
        check (lo[2] - lo[0] > 12.0 && lo[1] > lo[0] && lo[2] > lo[1] && std::fabs (hi[2] - hi[0]) < 4.0,
               "Bass moves 40 Hz by > 12 dB, monotonic, 2 kHz moves < 4 dB");
        std::printf ("    treble -5/0/+5 @6 kHz: %.1f %.1f %.1f dB (100 Hz %.1f..%.1f)\n", th[0], th[1], th[2], tl[0], tl[2]);
        check (th[2] - th[0] > 12.0 && th[1] > th[0] && th[2] > th[1] && std::fabs (tl[2] - tl[0]) < 3.0,
               "Treble moves 6 kHz by > 12 dB, monotonic, 100 Hz moves < 3 dB");
    }

    // Bass Boost: R21 1K5 across R11 (gentle on the drawing: the 100K Bass pot
    // dominates; most lift in the low mids with Bass turned up)
    {
        BassAmp off, on;
        off.setGain (0.3); on.setGain (0.3);
        off.setBass (1.0); on.setBass (1.0);
        on.setBassBoost (true);
        off.prepare (kSr); on.prepare (kSr);
        const double d200 = pre (on, 200.0).ic2 - pre (off, 200.0).ic2;
        const double d4k = pre (on, 4000.0).ic2 - pre (off, 4000.0).ic2;
        std::printf ("    Bass Boost on (Bass +5): 200 Hz %+.1f dB, 4 kHz %+.1f dB\n", d200, d4k);
        check (d200 > 2.0 && d200 > d4k + 1.0, "Bass Boost lifts the lows (200 Hz > 2 dB, more than the top)");
    }

    // Middle and Presence (module)
    {
        double m[3], p[3];
        for (int s = 0; s < 3; ++s)
        {
            BassAmp a, b;
            a.setGain (0.3); b.setGain (0.3);
            a.setMiddle (s * 0.5); b.setPresence (s * 0.5);
            a.prepare (kSr); b.prepare (kSr);
            m[s] = pre (a, 500.0).mod - pre (a, 500.0).ic2;
            p[s] = pre (b, 5000.0).mod - pre (b, 5000.0).ic2;
        }
        std::printf ("    middle -5/0/+5 @500 Hz (module): %.1f %.1f %.1f dB\n", m[0], m[1], m[2]);
        check (m[2] - m[0] > 15.0 && m[1] > m[0] && m[2] > m[1], "Middle cuts and boosts 500 Hz, monotonic");
        std::printf ("    presence -5/0/+5 @5 kHz (module): %.1f %.1f %.1f dB\n", p[0], p[1], p[2]);
        check (p[2] - p[0] > 12.0 && p[1] > p[0] && p[2] > p[1], "Presence cuts and boosts 5 kHz, monotonic");
    }
}

static void testValveSound()
{
    std::printf ("2. Valve Sound\n");
    BassAmp off, on;
    off.setGain (0.8); on.setGain (0.8);
    on.setValveSound (true);
    off.prepare (kSr); on.prepare (kSr);
    const double h2off = harmonicDb (off, 110.0, 0.5, 2), h2on = harmonicDb (on, 110.0, 0.5, 2);
    const double h3off = harmonicDb (off, 110.0, 0.5, 3);
    std::printf ("    110 Hz driven hard: 2nd harmonic off %.1f dB, on %.1f dB (3rd off %.1f dB)\n", h2off, h2on, h3off);
    check (h3off > -30.0, "IC2 clips when driven hard (the V-S sustain)");
    check (h2on > h2off + 10.0 && h2on > -40.0, "Valve Sound adds even harmonics (asymmetric, valve-like)");
}

static void testSwitches()
{
    std::printf ("3. Mains and Cab switches\n");
    // Mains off: the input, delayed by the reported latency, exactly
    {
        BassAmp a;
        a.setPower (false);
        a.prepare (kSr);
        std::vector<float> in = sine (110.0, 0.3, 0.3);
        const std::vector<float> o = run (a, in);
        const int L = BassAmp::latencySamples();
        double d = 0.0;
        for (size_t i = (size_t) L; i < o.size(); ++i) d = std::max (d, (double) std::fabs (o[i] - in[i - L]));
        std::printf ("    latency %d samples\n", L);
        check (d < 1.0e-6, "mains off passes the dry input, delayed by the reported latency");
    }
    // Cab switch: the two cabs sound different (miked), both play
    {
        BassAmp c15, c10;
        c10.setCab (BassAmp::kCab4x10);
        c15.prepare (kSr); c10.prepare (kSr);
        const std::vector<float> lo = sine (60.0, 0.02, 1.0), hi = sine (3000.0, 0.02, 1.0);
        const double l15 = rmsFrom (run (c15, lo), kSr / 2), l10 = rmsFrom (run (c10, lo), kSr / 2);
        c15.reset(); c10.reset();
        const double h15 = rmsFrom (run (c15, hi), kSr / 2), h10 = rmsFrom (run (c10, hi), kSr / 2);
        const double tilt15 = 20.0 * std::log10 (h15 / l15), tilt10 = 20.0 * std::log10 (h10 / l10);
        std::printf ("    3 kHz vs 60 Hz: 1x15 %+.1f dB, 4x10 %+.1f dB\n", tilt15, tilt10);
        check (l15 > 1e-4 && l10 > 1e-4, "both cabs play");
        check (tilt10 > tilt15 + 4.0, "the 4x10 is brighter than the 1x15, the 1x15 deeper");
    }
    // Low-end weight (miked, flat settings): 60-80 Hz close to the mids, but
    // no modern sub bass (25 Hz well down)
    for (int c = 0; c < 2; ++c)
    {
        double lv[4];
        const double fq[4] = { 25.0, 60.0, 80.0, 400.0 };
        for (int i = 0; i < 4; ++i)
        {
            BassAmp a;
            a.setCab (c);
            a.prepare (kSr);
            lv[i] = 20.0 * std::log10 (rmsFrom (run (a, sine (fq[i], 0.005, 1.0)), kSr / 2) + 1e-30);
        }
        std::printf ("    %s miked, flat: 25 Hz %+.1f, 60 Hz %+.1f, 80 Hz %+.1f dB re 400 Hz\n",
                     c ? "4x10" : "1x15", lv[0] - lv[3], lv[1] - lv[3], lv[2] - lv[3]);
        char msg[128];
        std::snprintf (msg, sizeof (msg), "%s: 80 Hz within %s dB of 400 Hz, 25 Hz more than 12 dB below 80 Hz",
                       c ? "4x10" : "1x15", c ? "7" : "4");
        check (lv[2] - lv[3] > (c ? -7.0 : -4.0) && lv[2] - lv[0] > 12.0, msg);
    }
    // Switching while playing: no NaN, no runaway
    {
        BassAmp a;
        a.prepare (kSr);
        const std::vector<float> in = sine (55.0, 0.3, 3.0);
        std::vector<float> out (in.size());
        const int block = 128;
        for (size_t pos = 0, k = 0; pos < in.size(); pos += block, ++k)
        {
            if (k % 37 == 0) a.setValveSound ((k / 37) % 2 == 1);
            if (k % 53 == 0) a.setBassBoost ((k / 53) % 2 == 1);
            if (k % 61 == 0) a.setCab ((int) ((k / 61) % 2));
            if (k % 83 == 0) a.setPower ((k / 83) % 3 != 2);
            if (k % 97 == 0) a.setSpeaker ((k / 97) % 2 == 0);
            const int n = (int) std::min ((size_t) block, in.size() - pos);
            a.process (&in[pos], &out[pos], n);
        }
        check (allFinite (out, 4.0), "every switch toggled while playing: finite, bounded");
    }
}

static void testPowerAmp()
{
    std::printf ("4. Power amp\n");
    for (int c = 0; c < 2; ++c)
    {
        BassAmp a;
        a.setGain (0.7);
        a.setMasterVolume (1.0);
        a.setSpeaker (false);
        a.setCab (c);
        a.prepare (kSr);
        const std::vector<float> in = sine (110.0, 0.3, 1.0);
        a.clearPeakAmps();
        const std::vector<float> o = run (a, in);
        const double vpk = peakFrom (o, o.size() / 2) * 40.0;
        std::printf ("    %s flat out: %.1f V peak, %.2f A peak, rails %.1f V\n", c ? "4x10" : "1x15",
                     vpk, a.peakSpeakerAmps(), a.railPositive (c));
        check (vpk > 38.0 && vpk < 49.0, c ? "4x10: clips at 38-49 V peak (about 100 W into 8 ohms)"
                                           : "1x15: clips at 38-49 V peak (about 100 W into 8 ohms)");
        check (a.peakSpeakerAmps() < 7.51, "speaker current within the V-I limit");
        check (a.railPositive (c) < 48.5, "rails sag under full load");
        std::vector<float> quiet (kSr, 0.0f);
        run (a, quiet);
        check (a.railPositive (c) > 47.0, "rails recover after the note");
    }

    // Load impedance: about 8 ohms nominal, a peak at the cab's resonance
    for (int c = 0; c < 2; ++c)
    {
        BassAmp a;
        double zmin = 1e9, zmax = 0.0, fmax = 0.0;
        for (double f = 20.0; f < 5000.0; f *= 1.02)
        {
            const double z = a.cabImpedance (c, f);
            if (f > 150.0 && f < 1000.0) zmin = std::min (zmin, z);
            if (z > zmax) { zmax = z; fmax = f; }
        }
        std::printf ("    %s load: minimum %.1f ohms (150 Hz-1 kHz), peak %.0f ohms at %.0f Hz\n",
                     c ? "4x10" : "1x15", zmin, zmax, fmax);
        check (zmin > 5.0 && zmin < 8.5 && zmax > 25.0, c ? "4x10 is an 8 ohm load with its resonance peak"
                                                           : "1x15 is an 8 ohm load with its resonance peak");
    }

    // Moderate settings play below clipping
    BassAmp b;
    b.setSpeaker (false);
    b.prepare (kSr);
    const std::vector<float> ob = run (b, sine (110.0, 0.05, 0.5));
    const double vb = peakFrom (ob, ob.size() / 2) * 40.0;
    std::printf ("    50 mV bass, Gain 5, Master 3.5: %.2f V peak\n", vb);
    check (vb > 0.1 && vb < 35.0, "moderate settings play below clipping");
}

static void testStability()
{
    std::printf ("5. Stability sweep\n");
    unsigned int seed = 12345u;
    bool ok = true;
    const double amps[] = { 0.01, 0.3, 1.5, 4.0 };
    const double freqs[] = { 41.0, 330.0, 2500.0, 15000.0 };
    int runs = 0;
    for (int k = 0; k < 24; ++k)
    {
        BassAmp a;
        double v[6];
        for (int i = 0; i < 6; ++i)
        {
            seed = seed * 1664525u + 1013904223u;
            v[i] = (k < 2) ? (double) k : (seed >> 8) / 16777216.0;
        }
        a.setGain (v[0]); a.setBass (v[1]); a.setMiddle (v[2]); a.setTreble (v[3]);
        a.setPresence (v[4]); a.setMasterVolume (v[5]);
        a.setValveSound (k % 2 == 0); a.setBassBoost (k % 3 != 0); a.setCab (k % 4 == 1);
        a.setSpeaker (k % 5 != 1);
        a.prepare (k % 3 == 0 ? 96000.0 : kSr);
        for (int ai = 0; ai < 4; ++ai)
            for (int fi = 0; fi < 4; ++fi)
            {
                const std::vector<float> o = run (a, sine (freqs[fi], amps[ai], 0.15));
                if (! allFinite (o, 3.0)) ok = false;
                ++runs;
            }
        std::vector<float> noise ((size_t) (0.3 * kSr));
        for (size_t i = 0; i < noise.size(); ++i)
        {
            seed = seed * 1664525u + 1013904223u;
            noise[i] = (float) (((seed >> 8) / 16777216.0 * 2.0 - 1.0) * 4.0);
        }
        if (! allFinite (run (a, noise), 3.0)) ok = false;
    }
    // Very low and very high host rates (hosts and validators try 1 kHz - 768 kHz)
    {
        const double rates[] = { 1000.0, 8000.0, 11025.0, 22050.0, 192000.0, 384000.0, 768000.0 };
        for (int k = 0; k < 7; ++k)
        {
            BassAmp a;
            a.setValveSound (k % 2 == 0); a.setGain (1.0); a.setMasterVolume (1.0); a.setCab (k % 2);
            a.prepare (rates[k]);
            std::vector<float> noise ((size_t) (rates[k] * 0.05) + 64);
            for (size_t i = 0; i < noise.size(); ++i)
            {
                seed = seed * 1664525u + 1013904223u;
                noise[i] = (float) (((seed >> 8) / 16777216.0 * 2.0 - 1.0) * 0.5);
            }
            if (! allFinite (run (a, noise), 3.0)) { ok = false; std::printf ("    unstable at %.0f Hz\n", rates[k]); }
        }
    }
    std::printf ("    %d sine runs + 24 loud noise runs + 7 sample rates from 1 kHz to 768 kHz\n", runs);
    check (ok, "no NaN, no runaway, output under +10 dBFS for every setting");
}

static void testAutomation()
{
    std::printf ("6. Automation\n");
    BassAmp a;
    a.prepare (kSr);
    const std::vector<float> in = riff (0.3);
    std::vector<float> out (in.size());
    const int block = 64;
    for (size_t pos = 0, k = 0; pos < in.size(); pos += block, ++k)
    {
        const double t = (double) pos / kSr;
        const double s = 0.5 + 0.5 * std::sin (t * 7.0);
        a.setGain (s); a.setBass (1.0 - s); a.setMiddle (s); a.setTreble (s);
        a.setPresence (1.0 - s); a.setMasterVolume (0.2 + 0.6 * s);
        a.setInputGainDb (-12.0 + 24.0 * s); a.setOutputGainDb (-6.0 * s);
        const int n = (int) std::min ((size_t) block, in.size() - pos);
        a.process (&in[pos], &out[pos], n);
    }
    check (allFinite (out, 3.0), "every knob swept while playing: finite, bounded");
}

static void testCpu()
{
    std::printf ("7. CPU\n");
    BassAmp a;
    a.setValveSound (true);
    a.prepare (kSr);
    const std::vector<float> in = riff (0.3);
    const std::clock_t t0 = std::clock();
    run (a, in);
    const double sec = (double) (std::clock() - t0) / CLOCKS_PER_SEC;
    const double perSec = sec / ((double) in.size() / kSr);
    std::printf ("    %.1f ms per second of audio (Valve Sound on, one side)\n", perSec * 1000.0);
#if defined(__SANITIZE_ADDRESS__)
    std::printf ("    (sanitizer build: timing not checked)\n");
#else
    check (perSec < 0.5, "runs well within real time");
#endif
}

static void renders (const char* dir)
{
    std::printf ("8. Demo renders\n");
    const std::vector<float> g = riff (0.25);
    struct R { const char* name; double gain; bool boost; double bass, mid, treb, pres; bool valve; double master; int cab; };
    const R rs[] = {
        { "clean_1x15",        0.35, false, 0.5, 0.5, 0.5, 0.5, false, 0.35, 0 },
        { "clean_4x10",        0.35, false, 0.5, 0.5, 0.5, 0.5, false, 0.35, 1 },
        { "bass_boost_1x15",   0.35, true,  0.6, 0.5, 0.5, 0.5, false, 0.35, 0 },
        { "growl_4x10",        0.75, false, 0.6, 0.7, 0.6, 0.65, false, 0.40, 1 },
        { "valve_sound_1x15",  0.70, false, 0.55, 0.6, 0.5, 0.5, true,  0.40, 0 },
        { "valve_sound_4x10",  0.80, true,  0.5, 0.65, 0.55, 0.6, true,  0.45, 1 },
        { "scooped_4x10",      0.40, true,  0.7, 0.2, 0.7, 0.6, false, 0.40, 1 },
    };
    for (size_t i = 0; i < sizeof (rs) / sizeof (rs[0]); ++i)
    {
        BassAmp a;
        a.setGain (rs[i].gain); a.setBassBoost (rs[i].boost); a.setBass (rs[i].bass);
        a.setMiddle (rs[i].mid); a.setTreble (rs[i].treb); a.setPresence (rs[i].pres);
        a.setValveSound (rs[i].valve); a.setMasterVolume (rs[i].master); a.setCab (rs[i].cab);
        a.prepare (kSr);
        const std::vector<float> o = run (a, g);
        const std::string path = std::string (dir) + "/" + rs[i].name + ".wav";
        writeWav (path, o, kSr);
        std::printf ("    %-18s peak %.3f  rms %.3f\n", rs[i].name, peakFrom (o, 0), rmsFrom (o, 0));
    }
    writeWav (std::string (dir) + "/dry_bass.wav", g, kSr);
}

int main (int argc, char** argv)
{
    const char* dir = argc > 1 ? argv[1] : "renders";
    testPreamp();
    testValveSound();
    testSwitches();
    testPowerAmp();
    testStability();
    testAutomation();
    testCpu();
    renders (dir);
    std::printf ("\n%s (%d failure%s)\n", failures ? "FAILED" : "ALL PASSED", failures, failures == 1 ? "" : "s");
    return failures ? 1 : 0;
}
