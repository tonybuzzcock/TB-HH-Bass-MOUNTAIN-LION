// Params.h
// Parameter table shared by the VST wrapper and the editor.
//
// Order follows the VS Bass 100 front panel left to right (Gain, Bass Boost,
// Bass, Middle, Treble, Presence, Valve Sound, Master Volume), then the mains
// switch, then the plugin-only controls.

#ifndef TBVB_PARAMS_H
#define TBVB_PARAMS_H

namespace tbvb
{

enum Param
{
    // Front panel, left to right
    kGain = 0, kBassBoost, kBass, kMiddle, kTreble, kPresence, kValveSound, kMasterVolume,
    kMains,
    // Plugin only (bottom strip of the editor)
    kCab, kSpeaker, kInput, kOutput,
    kNumParams
};

enum Kind { kKnob, kTone, kSwitch, kCabSw, kDb12, kDb24 };

const char* const kParamNames[kNumParams] = {
    "Gain", "BassBst", "Bass", "Middle", "Treble", "Presence", "ValveSnd", "M Volume",
    "Mains",
    "Cab", "Speaker", "Input", "Output"
};

const int kParamKind[kNumParams] = {
    kKnob, kSwitch, kTone, kTone, kTone, kTone, kSwitch, kKnob,
    kSwitch,
    kCabSw, kSwitch, kDb12, kDb24
};

const float kDefaults[kNumParams] = {
    0.5f, 0.0f, 0.5f, 0.5f, 0.5f, 0.5f, 0.0f, 0.35f,
    1.0f,
    0.0f, 1.0f, 0.5f, 0.5f
};

const double kInputRangeDb  = 12.0;
const double kOutputRangeDb = 24.0;

inline bool isOn (float v) { return v >= 0.5f; }

} // namespace tbvb

#endif
