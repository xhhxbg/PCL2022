#include "ICPOptions.h"

ICPOptions::ICPOptions() {

    MaxNoIt = 2000;
    IdxFixedPointClouds = 0;
    NoOfTransfParam = 7; // similarity transformation
    HullVoxelSize = 0.01;
    UniformSamplingDistance = 0.02;
    PlaneSearchRadius = 0.1;
    MaxDeltaAngle = 1;
    MaxDistance = 0.05;
    MaxSigmaMad = 1e10;
    MaxRoughness = 1e-30;
    MinOverlapping = 1024;
    Plot = true;

    RandomSubsampling = false;
    NormalSubsampling = false;
    MaxLeverageSubsampling = false;
}