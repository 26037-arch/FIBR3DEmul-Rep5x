#include "Rep5xMotionAdapter.h"

#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>

#include <cmath>
#include <sstream>
#include <stdexcept>

namespace
{
    const double kPi = 3.1415926535897932384626433832795;

    Rep5xAxisProfile default_axis(const char* name, double sign, double min_value,
                                  double max_value, bool continuous, bool rotary)
    {
        const double scale = rotary ? kPi / 180.0 : 1.0 / 1000.0;
        Rep5xAxisProfile axis;
        axis.joint_name = name;
        axis.scene_sign = sign;
        axis.zero_offset_si = 0.0;
        axis.soft_min_si = min_value * scale;
        axis.soft_max_si = max_value * scale;
        axis.continuous = continuous;
        return axis;
    }

    Rep5xAxisProfile read_axis(const boost::property_tree::ptree& root,
                               const char* key, bool rotary)
    {
        const std::string prefix = std::string("axes.") + key + ".";
        const double scale = rotary ? kPi / 180.0 : 1.0 / 1000.0;
        Rep5xAxisProfile axis;
        axis.joint_name = root.get<std::string>(prefix + "scene_joint");
        axis.scene_sign = root.get<double>(prefix + "scene_sign");
        if (axis.scene_sign != -1.0 && axis.scene_sign != 1.0)
            throw std::runtime_error(prefix + "scene_sign must be +1 or -1");
        axis.zero_offset_si = root.get<double>(prefix + "zero_offset") * scale;
        axis.continuous = root.get<bool>(prefix + "continuous");
        axis.soft_min_si = axis.continuous ? 0.0 : root.get<double>(prefix + "soft_min") * scale;
        axis.soft_max_si = axis.continuous ? 0.0 : root.get<double>(prefix + "soft_max") * scale;
        return axis;
    }

    void check_axis(const Rep5xAxisProfile& axis, double value, const char* name)
    {
        if (!std::isfinite(value))
            throw std::runtime_error(std::string("Rep5x motion rejected NaN/Infinity on axis ") + name);
        if (!axis.continuous && (value < axis.soft_min_si || value > axis.soft_max_si))
        {
            std::ostringstream message;
            message << "Rep5x soft limit exceeded on axis " << name << ": " << value
                    << " SI, allowed [" << axis.soft_min_si << ", " << axis.soft_max_si << "]";
            throw std::out_of_range(message.str());
        }
    }
}

Rep5xMachineProfile::Rep5xMachineProfile()
    : lc_m(0.0), lb_m(54.67 / 1000.0)
{
    axes[0] = default_axis("Rep5x_X_joint", 1.0, 0.0, 200.0, false, false);
    axes[1] = default_axis("Rep5x_Y_bed_joint", -1.0, -40.0, 200.0, false, false);
    axes[2] = default_axis("Rep5x_Z_joint", 1.0, 0.0, 174.6, false, false);
    axes[3] = default_axis("Rep5x_C_joint", 1.0, 0.0, 0.0, true, true);
    axes[4] = default_axis("Rep5x_B_joint", 1.0, -135.0, 135.0, false, true);
}

Rep5xMachineProfile Rep5xMachineProfile::LoadJson(const std::string& path)
{
    boost::property_tree::ptree root;
    boost::property_tree::read_json(path, root);
    if (root.get<int>("schema_version") != 1)
        throw std::runtime_error("unsupported Rep5x machine profile schema_version");

    Rep5xMachineProfile result;
    result.lc_m = root.get<double>("kinematics.lc_mm") / 1000.0;
    result.lb_m = root.get<double>("kinematics.lb_mm") / 1000.0;
    result.axes[0] = read_axis(root, "X", false);
    result.axes[1] = read_axis(root, "Y", false);
    result.axes[2] = read_axis(root, "Z", false);
    result.axes[3] = read_axis(root, "C", true);
    result.axes[4] = read_axis(root, "B", true);
    return result;
}

Rep5xMotionAdapter::Rep5xMotionAdapter(const Rep5xMachineProfile& profile)
    : profile_(profile)
{
}

std::vector<double> Rep5xMotionAdapter::ToSceneJoints(const std::vector<double>& sample) const
{
    if (sample.size() != 5)
        throw std::runtime_error("FIBR3D/Rep5x JSON schema mismatch: trajectory sample must contain 5 values [X,Y,Z,B,C]");

    const double x = sample[0];
    const double y = sample[1];
    const double z = sample[2];
    const double b = sample[3];
    const double c = sample[4];
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) ||
        !std::isfinite(b) || !std::isfinite(c))
        throw std::runtime_error("Rep5x motion rejected NaN/Infinity in FIBR3D trajectory sample");

    const double machine_x = x - std::sin(c) * profile_.lc_m + std::cos(c) * std::sin(b) * profile_.lb_m;
    const double machine_y = y + (std::cos(c) - 1.0) * profile_.lc_m + std::sin(c) * std::sin(b) * profile_.lb_m;
    const double machine_z = z + (std::cos(b) - 1.0) * profile_.lb_m;

    check_axis(profile_.axes[0], machine_x, "X");
    check_axis(profile_.axes[1], machine_y, "Y");
    check_axis(profile_.axes[2], machine_z, "Z");
    check_axis(profile_.axes[3], c, "C");
    check_axis(profile_.axes[4], b, "B");

    std::vector<double> scene(5);
    scene[0] = (machine_x + profile_.axes[0].zero_offset_si) * profile_.axes[0].scene_sign;
    scene[1] = (machine_y + profile_.axes[1].zero_offset_si) * profile_.axes[1].scene_sign;
    scene[2] = (machine_z + profile_.axes[2].zero_offset_si) * profile_.axes[2].scene_sign;
    scene[3] = (c + profile_.axes[3].zero_offset_si) * profile_.axes[3].scene_sign;
    scene[4] = (b + profile_.axes[4].zero_offset_si) * profile_.axes[4].scene_sign;
    return scene;
}
