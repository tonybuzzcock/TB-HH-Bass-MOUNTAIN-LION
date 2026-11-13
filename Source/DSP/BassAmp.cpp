// BassAmp.cpp
// Component values are from the H/H drawing C/AC00015 "V-S Bassamp Circuit
// Diagram" (issue 5): the AS00016 Ch 1 board, the AS000/B module's wiring and
// the AS00014/2 output driver board. Reference designators in comments.

#include "BassAmp.h"

#include <cmath>
#include <cstring>

namespace hhvb
{

namespace
{
    const int kPotUpdateInterval = 16;

    // 741 op-amps on the +/-16 V rails (PL16Z zeners ZD1/ZD2 fed through
    // R19/R20 620R from +/-48 V)
    const double kGbw741   = 1.0e6;    // Hz
    const double kSlew741  = 0.5;      // V/us
    const double kSwing741 = 14.0;     // V
    const double kKnee741  = 0.8;      // V
    const double kRout741  = 75.0;     // ohms, open-loop output resistance

    const double kPickupR  = 8000.0;   // bass pickup DC resistance (estimate)
    const double kPotMin   = 10.0;     // wiper contact / end resistance
    const double kOpen     = 1.0e9;    // a switch contact that is open

    // AS000/B module (behavioural; the drawing shows only its pins)
    const double kMidFreq   = 500.0;   // Hz, Middle centre (estimate)
    const double kMidQ      = 0.7;
    const double kMidRange  = 12.0;    // dB either way (estimate)
    const double kPresFreq  = 2500.0;  // Hz, Presence shelf (estimate)
    const double kPresRange = 10.0;    // dB either way (estimate)
    const double kModuleOut = 0.27;    // module output into the Master (estimate,
                                       // the level TB VS Musician's mixer gives)

    // Power amp (AS00014/2)
    const double kPaGain     = 1.0 + 14700.0 / 390.0;   // R5 10K + R33 4K7 over R3 390R
    const double kRailNoLoad = 50.0;   // V, +/-48 V nominal, a little higher at idle
    const double kRailC      = 4700.0e-6;   // C1, C2 reservoirs, 4700u 50 V
    const double kRailRs     = 1.5;    // 180 VA transformer + KBH06 bridge (estimate)
    const double kRailDrop   = 2.5;    // V lost in TIP29C/30C, 2N3773s and 0R33 emitter resistors
    const double kRailKnee   = 1.5;    // V
    const double kPaSlew     = 2.0;    // V/us at the output (estimate)
    const double kIMax       = 7.5;    // A, Q4/Q5 V-I limiter, roughly (estimate)
    const double kIQuiet     = 0.06;   // A per rail at idle

    const double kVFullScale = 40.0;   // V peak at the speaker = 1.0 out

    // The two cabs, both 8 ohms. Electrical driver models (Thiele-Small style:
    // Re, voice-coil inductance with eddy losses, motional impedance at the
    // in-box resonance) and the acoustic voice. All estimates for the owner's
    // H/H 15" (1977) and Trace Elliot 4x10 (1984).
    struct CabSpec
    {
        double re, le, rEddy, fs, qms, qes;          // electrical
        double lowF, lowQ, lowDb;                    // box low-end bump
        double midF, midQ, midDb;                    // cone / box character
        double brkF, brkQ, brkDb;                    // breakup
        double dipF, dipQ, dipDb;                    // upper-mid dip
        double lpF;                                  // top-end roll-off
        double shelfF, shelfDb;                      // low-end weight (asked for after the first test)
    };
    const CabSpec kCabs[2] = {
        // 1x15: deep, slow top, rolls off early
        { 5.6, 1.4e-3, 12.0, 45.0, 4.0, 0.50,
          80.0, 0.9, 3.5,   700.0, 1.0, 1.5,   1300.0, 1.4, 4.0,   2400.0, 1.2, -4.0,   3000.0,
          100.0, 6.0 },
        // 4x10: tighter low end, more upper mids and attack
        { 6.0, 0.45e-3, 18.0, 68.0, 3.0, 0.60,
          110.0, 1.0, 2.5,   900.0, 0.9, 2.0,   2600.0, 1.3, 5.0,   4200.0, 1.5, -3.0,   5200.0,
          110.0, 9.0 }
    };

    inline double clamp01 (double x) { return x < 0.0 ? 0.0 : (x > 1.0 ? 1.0 : x); }
    inline double dbToGain (double dB) { return std::pow (10.0, dB / 20.0); }
    inline double potR (double r) { return r < kPotMin ? kPotMin : r; }
}

// Log ("audio") taper: 10% of the track at mid-rotation.
double BassAmp::audioTaper (double x)
{
    const double k = 4.394449154672439;
    return (std::exp (k * x) - 1.0) / (std::exp (k) - 1.0);
}

BassAmp::PowerStage::PowerStage()
    : spkOut (0), spkCone (0), railPos (kRailNoLoad), railNeg (kRailNoLoad),
      vOutPrev (0.0), peakAmps (0.0), climHit (false), coneVPrev (0.0), cabNorm (1.0),
      running (true)
{
}

BassAmp::BassAmp()
    : fsBase (44100.0), fsOs (176400.0), built (false),
      ic1Out (0), ic2Out (0), eGainFb (-1), eGainOut (-1), eBoost (-1),
      eBassTop (-1), eBassBot (-1), eTrebTop (-1), eTrebBot (-1),
      slewStep (1.0), cabMix (0.0), spkMix (1.0),
      dryPos (0), powerMix (1.0),
      boost (false), valve (false), powerOn (true), speakerOn (true),
      cabSel (kCab1x15), boostApplied (false),
      inGainT (1.0), inGainC (1.0), outGainT (1.0), outGainC (1.0),
      smoothCoeff (0.001), potCountdown (0)
{
    const double defaults[kNumKnobs] = { 0.5, 0.5, 0.5, 0.5, 0.5, 0.35 };
    for (int i = 0; i < kNumKnobs; ++i)
        knobTarget[i] = knobCur[i] = defaults[i];
    std::memset (dry, 0, sizeof (dry));
    buildCircuits();
}

//==============================================================================
// The circuits

void BassAmp::buildCircuits()
{
    // ---- IC1 gain stage: R1 10K and C1 0.1u into the Q1/Q2 2N5089 long-tail
    // pair, which is IC1's input stage (Q1 base +, Q2 base -), R3 100K bias.
    // Feedback R9 22K || C4 68p to Q2's base; from there R8 2K7, with R22 1K
    // linked across it (pins 26-27), to C3 6u8 and the Gain pot's CW end
    // (pin 25); the wiper (24) is grounded. The effects input's R2 10K is
    // left open (nothing plugged in).
    {
        Netlist& n = ic1; n.clear();
        const int n1 = n.addNode(), n2 = n.addNode(), n3 = n.addNode(), n4 = n.addNode(),
                  n5 = n.addNode(), o = n.addNode();
        n.addInput (n1, kPickupR + 10000.0);   // pickup + R1 10K
        n.addC (n1, n2, 0.1e-6);               // C1
        n.addR (n2, 0, 100000.0);              // R3
        n.addR (o, n3, 22000.0);               // R9
        n.addC (o, n3, 68.0e-12);              // C4
        n.addR (n3, n4, 2700.0);               // R8
        n.addR (n3, n4, 1000.0);               // R22 via LINK 26-27
        n.addC (n4, n5, 6.8e-6);               // C3
        eGainFb = n.addR (n5, 0, 12500.0);     // VR2 Gain, CW end (25) to wiper (24)
        n.setOpAmp (n2, n3, o, kGbw741, kSlew741, kSwing741, kKnee741);
        ic1Out = o;
    }

    // ---- Bass Boost, tone stack and IC2. C5 6u8, R10 470R to the stack top,
    // which the Gain pot's other leg (pin 23 to the wiper) shunts to ground.
    // R11 10K to the Bass pot top; S3 Bass Boost puts R21 1K5 across it.
    // Bass VR5 100K lin with C6/C7 0.047u, R12 10K. Treble: R14 10K, VR6
    // 100K lin, R15 10K, wiper through C8 4700p. Bass wiper through R13 100K.
    // C9 0.1u into IC2 (inverting, + to ground) with R16 1M5 feedback.
    {
        Netlist& n = ic2; n.clear();
        const int s = n.addNode(), a = n.addNode(), t = n.addNode(), b18 = n.addNode(),
                  b17 = n.addNode(), b16 = n.addNode(), t21 = n.addNode(), t20 = n.addNode(),
                  t19 = n.addNode(), N = n.addNode(), m = n.addNode(), o = n.addNode();
        n.addInput (s, kRout741);
        n.addC (s, a, 6.8e-6);                 // C5
        n.addR (a, t, 470.0);                  // R10
        eGainOut = n.addR (t, 0, 12500.0);     // VR2 Gain, pin 23 to wiper
        n.addR (t, b18, 10000.0);              // R11
        eBoost = n.addR (t, b18, kOpen);       // R21 1K5 through S3 (pin 22)
        eBassTop = n.addR (b18, b17, 50000.0); // VR5 Bass 100K lin
        eBassBot = n.addR (b17, b16, 50000.0);
        n.addC (b18, b17, 0.047e-6);           // C6
        n.addC (b17, b16, 0.047e-6);           // C7
        n.addR (b16, 0, 10000.0);              // R12
        n.addR (t, t21, 10000.0);              // R14
        eTrebTop = n.addR (t21, t20, 50000.0); // VR6 Treble 100K lin
        eTrebBot = n.addR (t20, t19, 50000.0);
        n.addR (t19, 0, 10000.0);              // R15
        n.addC (t20, N, 4700.0e-12);           // C8
        n.addR (b17, N, 100000.0);             // R13
        n.addC (N, m, 0.1e-6);                 // C9
        n.addR (m, o, 1.5e6);                  // R16
        n.setOpAmp (0, m, o, kGbw741, kSlew741, kSwing741, kKnee741);
        ic2Out = o;
    }

    buildSpeaker (kCab1x15);
    buildSpeaker (kCab4x10);

    built = ic1Out > 0 && ic2Out > 0 && eTrebBot >= 0 && eBoost >= 0
         && pa[0].spkCone > 0 && pa[1].spkCone > 0;
}

// Output network (L1 with R29 15R, Zobel R30 15R + C14 0.047u) into one cab.
void BassAmp::buildSpeaker (int c)
{
    const CabSpec& k = kCabs[c];
    Netlist& n = pa[c].speaker; n.clear();
    const int a = n.addNode(), o = n.addNode(), z = n.addNode(), p = n.addNode(), q = n.addNode();
    n.addInput (a, 0.01);                      // power amp output (feedback holds this node)
    n.addL (a, o, 3.0e-6);                     // L1 (value not marked; typical)
    n.addR (a, o, 15.0);                       // R29
    n.addR (o, z, 15.0);                       // R30 Zobel
    n.addC (z, 0, 0.047e-6);                   // C14
    const double ws = 2.0 * kPi * k.fs;
    const double cmes = k.qes / (ws * k.re);
    const double lces = 1.0 / (ws * ws * cmes);
    const double res = k.re * k.qms / k.qes;
    n.addR (o, p, k.re);                       // voice coil resistance
    n.addL (p, q, k.le);                       // voice coil inductance
    n.addR (p, q, k.rEddy);                    // eddy-current losses
    n.addR (q, 0, res);                        // motional: mechanical losses
    n.addL (q, 0, lces);                       //           suspension compliance
    n.addC (q, 0, cmes);                       //           moving mass
    pa[c].spkOut = o;
    pa[c].spkCone = q;
}

double BassAmp::cabImpedance (int c, double f) const
{
    if (c < 0 || c >= kNumCabs) c = 0;
    const CabSpec& k = kCabs[c];
    const double w = 2.0 * kPi * f;
    const double ws = 2.0 * kPi * k.fs;
    const double cmes = k.qes / (ws * k.re);
    const double lces = 1.0 / (ws * ws * cmes);
    const double res = k.re * k.qms / k.qes;
    // Motional: R || L || C
    const double gRe = 1.0 / res, gIm = w * cmes - 1.0 / (w * lces);
    const double den = gRe * gRe + gIm * gIm;
    const double zmRe = gRe / den, zmIm = -gIm / den;
    // Voice coil: L || Reddy
    const double xl = w * k.le, re2 = k.rEddy;
    const double d2 = re2 * re2 + xl * xl;
    const double zlRe = re2 * xl * xl / d2, zlIm = re2 * re2 * xl / d2;
    const double zr = k.re + zlRe + zmRe, zi = zlIm + zmIm;
    return std::sqrt (zr * zr + zi * zi);
}

//==============================================================================
void BassAmp::prepare (double sr)
{
    fsBase = sr;
    fsOs = sr * Oversampler4x::kFactor;
    smoothCoeff = 1.0 - std::exp (-1.0 / (0.02 * sr));

    ic1.prepare (fsOs);
    ic2.prepare (fsOs);

    // Valve Sound and module filters
    valvePre.setLowPass (fsOs, 3500.0, 0.6);    // round off the clipped edges
    valvePost.setLowPass1 (fsOs, 5000.0);
    valveDc.setHighPass1 (fsOs, 8.0);
    moduleHP.setHighPass1 (fsOs, 1.0 / (2.0 * kPi * 47.0e-6 * 16000.0));   // C10 47u into R17 + R18

    slewStep = kPaSlew * 1.0e6 / fsOs;

    for (int c = 0; c < kNumCabs; ++c)
    {
        PowerStage& s = pa[c];
        const CabSpec& k = kCabs[c];
        s.speaker.prepare (fsOs);

        // Power amp: input coupling C1 0.22u into R1 1K + R2 15K; feedback leg
        // R3 390R + C2 6u8; 741 gain-bandwidth over the closed-loop gain.
        s.paInHP.setHighPass1 (fsOs, 1.0 / (2.0 * kPi * 0.22e-6 * 16000.0));
        s.paFbHP.setHighPass1 (fsOs, 1.0 / (2.0 * kPi * 390.0 * 6.8e-6));
        s.paGbwLP.setLowPass1 (fsOs, kGbw741 / kPaGain);

        // The cab's acoustic voice (base rate, after the physical low end)
        s.cab[0].setPeak (fsBase, k.lowF, k.lowQ, k.lowDb);
        s.cab[1].setPeak (fsBase, k.midF, k.midQ, k.midDb);
        s.cab[2].setPeak (fsBase, k.brkF, k.brkQ, k.brkDb);
        s.cab[3].setPeak (fsBase, k.dipF, k.dipQ, k.dipDb);
        const double lpF = fsBase * 0.45 < k.lpF ? fsBase * 0.45 : k.lpF;
        s.cab[4].setLowPass (fsBase, lpF, 0.7);
        s.cab[5].setLowPass (fsBase, lpF * 1.2, 1.0);
        // Extra weight from about 40 to 100 Hz, asked for after the first test (the
        // drawing's values leave the bottom end lean). Below ~35 Hz the cab still
        // rolls off like a real speaker: weight, not modern sub bass.
        s.cab[6].setLowShelf (fsBase, k.shelfF, k.shelfDb);

        // Normalise the miked cab so 1 kHz matches the speaker-terminal level
        // (a lower tone at very low host rates, where 1 kHz is above Nyquist).
        const double fNorm = fsBase * 0.2 < 1000.0 ? fsBase * 0.2 : 1000.0;
        s.speaker.reset();
        for (int i = 0; i < PowerStage::kCabBands; ++i) s.cab[i].reset();
        double prev = 0.0, peak = 0.0;
        const int nBase = (int) (0.12 * fsBase);
        const int F = Oversampler4x::kFactor;
        for (int i = 0; i < nBase; ++i)
        {
            double last = 0.0;
            for (int j = 0; j < F; ++j)
            {
                const double tt = (double) (i * F + j) / fsOs;
                const double vin = std::sin (2.0 * kPi * fNorm * tt);
                s.speaker.process (&vin);
                const double vc = s.speaker.v (s.spkCone);
                last = (vc - prev) * fsOs;
                prev = vc;
            }
            double y = last;
            for (int b = 0; b < PowerStage::kCabBands; ++b) y = s.cab[b].process (y);
            if (i > nBase / 2 && std::fabs (y) > peak) peak = std::fabs (y);
        }
        s.cabNorm = peak > 0.0 ? 1.0 / peak : 1.0;
    }

    reset();
}

void BassAmp::resetStage (PowerStage& s)
{
    s.speaker.reset();
    s.paInHP.reset(); s.paFbHP.reset(); s.paGbwLP.reset();
    for (int i = 0; i < PowerStage::kCabBands; ++i) s.cab[i].reset();
    s.osTerm.reset(); s.osSpk.reset();
    s.railPos = s.railNeg = kRailNoLoad;
    s.vOutPrev = 0.0; s.coneVPrev = 0.0;
    s.climHit = false;
}

void BassAmp::reset()
{
    for (int i = 0; i < kNumKnobs; ++i) knobCur[i] = knobTarget[i];
    inGainC = inGainT;
    outGainC = outGainT;
    applySwitches();
    applyPots();
    applyModuleEq();

    resetPreamp();
    for (int c = 0; c < kNumCabs; ++c) resetStage (pa[c]);
    osMain.reset();
    cabMix = cabSel == kCab4x10 ? 1.0 : 0.0;
    pa[kCab1x15].running = (cabSel == kCab1x15);
    pa[kCab4x10].running = (cabSel == kCab4x10);
    spkMix = speakerOn ? 1.0 : 0.0;
    powerMix = powerOn ? 1.0 : 0.0;
    std::memset (dry, 0, sizeof (dry));
    dryPos = 0;
    potCountdown = 0;
}

void BassAmp::resetPreamp()
{
    ic1.reset(); ic2.reset();
    midEq.reset(); presEq.reset();
    valveDc.reset(); valvePre.reset(); valvePost.reset(); moduleHP.reset();
}

//==============================================================================
// Pots and switches

void BassAmp::applyPots()
{
    const double R25 = 25000.0, R100 = 100000.0;

    // VR2 Gain, 25K reverse log ("R/LOG"), dual action: the CW leg shunts the
    // feedback (more gain clockwise), the other leg shunts the stage output.
    {
        const double rfb = R25 * audioTaper (1.0 - knobCur[kGain]);
        ic1.setR (eGainFb, potR (rfb));
        ic2.setR (eGainOut, potR (R25 - rfb));
    }
    // Bass and Treble, 100K linear: clockwise moves the wiper to the top.
    {
        const double b = knobCur[kBass], t = knobCur[kTreb];
        ic2.setR (eBassTop, potR (R100 * (1.0 - b)));
        ic2.setR (eBassBot, potR (R100 * b));
        ic2.setR (eTrebTop, potR (R100 * (1.0 - t)));
        ic2.setR (eTrebBot, potR (R100 * t));
    }
    ic1.rebuild(); ic2.rebuild();
}

void BassAmp::applySwitches()
{
    ic2.setR (eBoost, boost ? 1500.0 : kOpen);   // S3 puts R21 1K5 across R11
    ic2.rebuild();
    boostApplied = boost;
}

// AS000/B V-S tone correction module: its insides are not on the drawing;
// these sections do what its controls are wired to do (25K lin Middle and
// Presence pots, Valve Sound switch).
void BassAmp::applyModuleEq()
{
    midEq.setPeak (fsOs, kMidFreq, kMidQ, (knobCur[kMid] - 0.5) * 2.0 * kMidRange);
    presEq.setHighShelf (fsOs, kPresFreq, (knobCur[kPres] - 0.5) * 2.0 * kPresRange);
}

//==============================================================================
// Setters

void BassAmp::setGain (double v)          { knobTarget[kGain] = clamp01 (v); }
void BassAmp::setBassBoost (bool on)      { boost = on; }
void BassAmp::setBass (double v)          { knobTarget[kBass] = clamp01 (v); }
void BassAmp::setMiddle (double v)        { knobTarget[kMid]  = clamp01 (v); }
void BassAmp::setTreble (double v)        { knobTarget[kTreb] = clamp01 (v); }
void BassAmp::setPresence (double v)      { knobTarget[kPres] = clamp01 (v); }
void BassAmp::setValveSound (bool on)     { valve = on; }
void BassAmp::setMasterVolume (double v)  { knobTarget[kMVol] = clamp01 (v); }
void BassAmp::setPower (bool on)          { powerOn = on; }
void BassAmp::setCab (int c)              { cabSel = (c == kCab4x10) ? kCab4x10 : kCab1x15; }
void BassAmp::setSpeaker (bool on)        { speakerOn = on; }
void BassAmp::setInputGainDb (double dB)  { inGainT = dbToGain (dB); }
void BassAmp::setOutputGainDb (double dB) { outGainT = dbToGain (dB); }

double BassAmp::peakSpeakerAmps() const
{
    return pa[0].peakAmps > pa[1].peakAmps ? pa[0].peakAmps : pa[1].peakAmps;
}

void BassAmp::clearPeakAmps() { pa[0].peakAmps = pa[1].peakAmps = 0.0; }

//==============================================================================
// Signal path

double BassAmp::preamp (double g)
{
    ic1.process (&g);
    const double v = ic1.v (ic1Out);
    ic2.process (&v);
    return ic2.v (ic2Out);
}

double BassAmp::module (double x)
{
    x = moduleHP.process (x);          // C10 47u, R17 1K into the module
    if (valve)
    {
        // Asymmetric soft saturation and smoothing in place of IC2's hard edge
        // (the same Valve Sound section as TB VS Musician).
        double u = valvePre.process (x) * (1.0 / kSwing741);
        const double bias = 0.3, drive = 1.8;
        const double t0 = std::tanh (bias);
        const double slope = drive * (1.0 - t0 * t0);
        u = (std::tanh (drive * u + bias) - t0) / slope;
        x = valvePost.process (valveDc.process (u)) * kSwing741 * 1.6;
    }
    return presEq.process (midEq.process (x)) * kModuleOut;
}

void BassAmp::probePreamp (double vIn, double& ic2v, double& mod)
{
    ic2v = preamp (vIn);
    mod = module (ic2v);
}

double BassAmp::powerAmp (PowerStage& s, double vIn)
{
    // Closed-loop response: (1 + Rf/R3 * HP(R3, C2)) with the input coupling
    // high-pass and the 741's bandwidth.
    const double x = s.paInHP.process (vIn);
    double v = x + (kPaGain - 1.0) * s.paFbHP.process (x);
    v = s.paGbwLP.process (v);

    // Slew rate
    double d = v - s.vOutPrev;
    if (d > slewStep) d = slewStep; else if (d < -slewStep) d = -slewStep;
    v = s.vOutPrev + d;

    // Clipping against the sagging rails
    const double lim = (v >= 0.0 ? s.railPos : s.railNeg) - kRailDrop;
    const double lin = lim - kRailKnee;
    const double av = std::fabs (v);
    if (av > lin)
    {
        const double y = lin + kRailKnee * (1.0 - std::exp (-(av - lin) / kRailKnee));
        v = v < 0.0 ? -y : y;
    }

    // V-I current limiting into the actual speaker load
    double i0, gIn;
    s.speaker.inputCurrentTerms (i0, gIn);
    double i = i0 + gIn * v;
    s.climHit = false;
    if (i > kIMax)       { v = (kIMax - i0) / gIn;  i = kIMax;  s.climHit = true; }
    else if (i < -kIMax) { v = (-kIMax - i0) / gIn; i = -kIMax; s.climHit = true; }

    s.speaker.process (&v);
    s.vOutPrev = v;
    if (std::fabs (i) > s.peakAmps) s.peakAmps = std::fabs (i);

    // Reservoir capacitors: each rail supplies its half of the load and is
    // recharged through the transformer and bridge.
    const double dt = 1.0 / fsOs;
    const double iPos = kIQuiet + (i > 0.0 ? i : 0.0);
    const double iNeg = kIQuiet + (i < 0.0 ? -i : 0.0);
    const double chP = s.railPos < kRailNoLoad ? (kRailNoLoad - s.railPos) / kRailRs : 0.0;
    const double chN = s.railNeg < kRailNoLoad ? (kRailNoLoad - s.railNeg) / kRailRs : 0.0;
    s.railPos += (chP - iPos) * dt / kRailC;
    s.railNeg += (chN - iNeg) * dt / kRailC;
    if (s.railPos < 5.0) s.railPos = 5.0;
    if (s.railNeg < 5.0) s.railNeg = 5.0;

    return s.speaker.v (s.spkOut);
}

void BassAmp::process (const float* in, float* out, int numSamples)
{
    const int F = Oversampler4x::kFactor;
    const double a = smoothCoeff;
    const double aBlock = 1.0 - std::pow (1.0 - a, (double) kPotUpdateInterval);

    double up[F], term[kNumCabs][F], spk[kNumCabs][F];

    for (int n = 0; n < numSamples; ++n)
    {
        if (--potCountdown <= 0)
        {
            potCountdown = kPotUpdateInterval;
            if (boost != boostApplied) applySwitches();
            bool moved = false, modMoved = false;
            for (int k = 0; k < kNumKnobs; ++k)
            {
                const double d = knobTarget[k] - knobCur[k];
                if (std::fabs (d) > 1.0e-5)
                {
                    knobCur[k] += d * aBlock;
                    if (std::fabs (knobTarget[k] - knobCur[k]) < 1.0e-5) knobCur[k] = knobTarget[k];
                    if (k == kMid || k == kPres) modMoved = true;
                    else if (k != kMVol) moved = true;
                }
            }
            if (moved) applyPots();
            if (modMoved) applyModuleEq();

            // The chosen cab wakes from rest; the other sleeps once it is faded out.
            if (! pa[cabSel].running) { resetStage (pa[cabSel]); pa[cabSel].running = true; }
            const int other = 1 - cabSel;
            const double otherGain = other == kCab4x10 ? cabMix : 1.0 - cabMix;
            if (pa[other].running && otherGain < 1.0e-4) pa[other].running = false;
        }

        inGainC  += (inGainT - inGainC) * a;
        outGainC += (outGainT - outGainC) * a;
        const double mVol = audioTaper (knobCur[kMVol]);

        const double x = (double) in[n] * inGainC;
        osMain.upsample (x, up);

        for (int k = 0; k < F; ++k)
        {
            const double drive = module (preamp (up[k])) * mVol;
            // Terminal and miked outputs both run so the Speaker switch crossfades cleanly.
            for (int c = 0; c < kNumCabs; ++c)
            {
                PowerStage& s = pa[c];
                if (! s.running) { term[c][k] = spk[c][k] = 0.0; continue; }
                term[c][k] = powerAmp (s, drive);
                const double vc = s.speaker.v (s.spkCone);
                spk[c][k] = (vc - s.coneVPrev) * fsOs;   // cone acceleration ~ radiated pressure
                s.coneVPrev = vc;
            }
        }

        double yTerm[kNumCabs], yAc[kNumCabs];
        for (int c = 0; c < kNumCabs; ++c)
        {
            PowerStage& s = pa[c];
            if (! s.running) { yTerm[c] = yAc[c] = 0.0; continue; }
            yTerm[c] = s.osTerm.downsample (term[c]);
            double y = s.osSpk.downsample (spk[c]);
            for (int b = 0; b < PowerStage::kCabBands; ++b) y = s.cab[b].process (y);
            yAc[c] = y * s.cabNorm;
        }
        cabMix += ((cabSel == kCab4x10 ? 1.0 : 0.0) - cabMix) * a;
        spkMix += ((speakerOn ? 1.0 : 0.0) - spkMix) * a;
        const double t  = yTerm[0] + (yTerm[1] - yTerm[0]) * cabMix;
        const double ac = yAc[0] + (yAc[1] - yAc[0]) * cabMix;
        const double y = (t + (ac - t) * spkMix) * (outGainC / kVFullScale);

        // Mains switch: off passes the input straight through, delayed to line up with the amp.
        dry[dryPos] = (double) in[n];
        int rd = dryPos - latencySamples();
        if (rd < 0) rd += kDryLen;
        const double dryOut = dry[rd];
        dryPos = (dryPos + 1) % kDryLen;
        powerMix += ((powerOn ? 1.0 : 0.0) - powerMix) * a;

        double o = dryOut + (y - dryOut) * powerMix;
        if (! (std::fabs (o) < 1.0e6)) { reset(); o = 0.0; }   // NaN or infinity: start again from rest
        out[n] = (float) o;
    }
}

} // namespace hhvb
