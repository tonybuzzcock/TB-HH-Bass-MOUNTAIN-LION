// BassAmp.h
// TB VS Bass 100: model of the H/H VS Bass 100 bass amplifier (drawing
// C/AC00015 "V-S Bassamp Circuit Diagram", issue 5), the whole amplifier, one
// mono signal path.
//
//   bass -> AS00016: R1 10K, 2N5089 pair + 741 IC1 with the Gain pot (R22 1K
//           linked across R8) -> Bass Boost (S3, R21 1K5) -> passive Baxandall
//           Bass/Treble -> 741 IC2 (1M5 feedback)
//        -> AS000/B V-S tone correction module: Middle, Presence, Valve Sound
//        -> Master Volume (25K log)
//        -> AS00014/2 100 W power amp (+/-48 V, 180 VA transformer)
//        -> 8 ohm cab: the owner's H/H 1x15 (1977) or Trace Elliot 4x10 (1984)
//
// The AS00016 board and the power amp are modelled from their component values
// (the Netlist class, 4x oversampled), the power amp from its behaviour (gain,
// sagging rails, slew, V-I limit) into the electrical loudspeaker load. The
// AS000/B module is drawn only as a box with pins, so its insides are
// modelled from what its controls do. Each cab is an electrical driver model
// plus a fixed acoustic voice. Both run while the Cab switch crossfades;
// the one not selected then sleeps (and wakes from rest when chosen again).
//
// Calibration: 1.0 at the input is 1 V peak from the bass. Output 1.0 is 40 V
// peak at the speaker terminals (100 W into 8 ohms); with Speaker on, the
// miked cab is scaled to match at 1 kHz.
// Plain C++98, no allocation or locks in process(). One instance per channel.

#ifndef TBVB_BASSAMP_H
#define TBVB_BASSAMP_H

#include "Netlist.h"
#include "Oversampler4x.h"
#include "Biquad.h"

namespace hhvb
{

class BassAmp
{
public:
    enum Cab { kCab1x15 = 0, kCab4x10 = 1, kNumCabs = 2 };

    BassAmp();

    void prepare (double sampleRate);
    void reset();

    // Front panel, left to right. Knobs are 0..1 (Gain and Master 0 to 10 on
    // the dial; tone knobs 0.5 = the centre "0" mark). Changes are smoothed.
    void setGain (double v);
    void setBassBoost (bool on);       // S3
    void setBass (double v);
    void setMiddle (double v);
    void setTreble (double v);
    void setPresence (double v);
    void setValveSound (bool on);      // S2 (or the footswitch)
    void setMasterVolume (double v);
    void setPower (bool on);           // mains switch: off = the dry signal passes

    // Plugin-only controls
    void setCab (int cab);
    void setSpeaker (bool on);
    void setInputGainDb (double dB);
    void setOutputGainDb (double dB);

    void process (const float* in, float* out, int numSamples);

    static int latencySamples() { return Oversampler4x::latencySamples(); }

    // For tests
    double railPositive (int cab) const { return pa[cab].railPos; }
    double railNegative (int cab) const { return pa[cab].railNeg; }
    double peakSpeakerAmps() const;
    void   clearPeakAmps();
    bool   circuitsBuilt() const { return built; }

    // Probe: one oversampled sample of the preamp alone (IC2 output, and the
    // module output before the Master Volume). For the small-signal tests.
    void   probePreamp (double vIn, double& ic2, double& module);
    void   resetPreamp();
    double oversampledRate() const { return fsOs; }
    // Small-signal impedance magnitude of a cab's load at f (ohms), for tests.
    double cabImpedance (int cab, double f) const;

    static double audioTaper (double x);

private:
    enum Knob { kGain, kBass, kMid, kTreb, kPres, kMVol, kNumKnobs };

    // One power amp driving one cab (two run side by side for the Cab switch)
    struct PowerStage
    {
        Netlist speaker;
        int spkOut, spkCone;
        Biquad paInHP, paFbHP, paGbwLP;
        double railPos, railNeg, vOutPrev, peakAmps;
        bool climHit;
        double coneVPrev;
        enum { kCabBands = 7 };
        Biquad cab[kCabBands];
        double cabNorm;
        Oversampler4x osTerm, osSpk;
        bool running;            // the cab not selected sleeps once the crossfade is over
        PowerStage();
    };

    void buildCircuits();
    void buildSpeaker (int cab);
    void applyPots();
    void applySwitches();
    void applyModuleEq();
    double preamp (double g);
    double module (double x);
    double powerAmp (PowerStage& s, double vIn);
    void   resetStage (PowerStage& s);

    double fsBase, fsOs;
    bool built;

    // AS00016
    Netlist ic1, ic2;
    int ic1Out, ic2Out;
    int eGainFb, eGainOut, eBoost;
    int eBassTop, eBassBot, eTrebTop, eTrebBot;

    // AS000/B module (behavioural)
    Biquad midEq, presEq, valveDc, valvePre, valvePost, moduleHP;

    // Power stages, one per cab
    PowerStage pa[kNumCabs];
    Oversampler4x osMain;
    double slewStep;
    double cabMix;       // 0 = 1x15, 1 = 4x10 (crossfaded)
    double spkMix;       // 0 = speaker terminals, 1 = miked cab (crossfaded)

    // Dry path for the mains switch (delayed by the oversampler latency)
    enum { kDryLen = 64 };
    double dry[kDryLen];
    int dryPos;
    double powerMix;     // 1 = amp, 0 = dry

    // Controls
    double knobTarget[kNumKnobs], knobCur[kNumKnobs];
    bool boost, valve, powerOn, speakerOn;
    int cabSel;
    bool boostApplied;
    double inGainT, inGainC, outGainT, outGainC;
    double smoothCoeff;
    int potCountdown;
};

} // namespace hhvb

#endif
