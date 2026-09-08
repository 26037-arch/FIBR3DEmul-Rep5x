#pragma once

#include <array>
#include <string>
#include <vector>

struct Rep5xAxisProfile
{
    std::string joint_name;
    double scene_sign;
    double zero_offset_si;
    double soft_min_si;
    double soft_max_si;
    bool continuous;
};

class Rep5xMachineProfile
{
public:
    Rep5xMachineProfile();
    static Rep5xMachineProfile LoadJson(const std::string& path);

    double lc_m;
    double lb_m;
    std::array<Rep5xAxisProfile, 5> axes; // semantic/scene order X,Y,Z,C,B
};

class Rep5xMotionAdapter
{
public:
    explicit Rep5xMotionAdapter(const Rep5xMachineProfile& profile);

    // FIBR3D Trajectory source order is X,Y,Z,B,C and is already m/rad.
    // Returned scene joint order is X,Y,Z,C,B and remains m/rad.
    std::vector<double> ToSceneJoints(const std::vector<double>& fibr_sample) const;
    const Rep5xMachineProfile& profile() const { return profile_; }

private:
    Rep5xMachineProfile profile_;
};
