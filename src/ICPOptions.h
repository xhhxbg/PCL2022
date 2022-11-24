#ifndef MY_GRAND_PROJECT_ICPOPTIONS_H
#define MY_GRAND_PROJECT_ICPOPTIONS_H

class ICPOptions {
public:
    ICPOptions();

    int MaxNoIt;
    int IdxFixedPointClouds;
    int NoOfTransfParam; // similarity transformation
    int MaxDeltaAngle;
    int MinOverlapping;

    float MaxDistance;
    float MaxSigmaMad;
    float HullVoxelSize;
    float UniformSamplingDistance;
    float PlaneSearchRadius;
    float MaxRoughness;

    bool Plot;
    bool RandomSubsampling;
    bool NormalSubsampling;
    bool MaxLeverageSubsampling;
};

#endif //MY_GRAND_PROJECT_ICPOPTIONS_H
