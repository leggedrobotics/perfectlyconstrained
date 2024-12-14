#pragma once

#include <vector>
#include <unordered_map>

namespace pointmatcher_ros
{

/**
 * @brief Enumerate containing keys that represent colors.
 * 
 */
enum class ColorKey
{
    kWhite = 0,
    kRed,
    kGreen,
    kBlue,
    kCyan,
    kYellow,
    kGold,
    kGrey,
    kLavender,
    kOrange,
    kBlack
};

/**
 * @brief Structure that maps RGB color values to the keys in the 'ColorKey' enumerate.
 * @remark It is expected that this structure stays in-sync with the ColorKey enumerate, with a 1-to-1 mapping between color and values.
 * 
 */
struct RgbaColorMap
{
    using Values = std::vector<float>;

    RgbaColorMap() = default;

    Values operator[](const ColorKey key) const { return rgbColorValues_.at(key); }

    const std::unordered_map<ColorKey, Values> rgbColorValues_ = { { ColorKey::kWhite, { 1, 1, 1, 1 } },
                                                                   { ColorKey::kRed, { 1, 0, 0, 1 } },
                                                                   { ColorKey::kGreen, { 0, 1, 0, 1 } },
                                                                   { ColorKey::kBlue, { 0, 0, 1, 1 } },
                                                                   { ColorKey::kCyan, { 0, 1, 1, 1 } },
                                                                   { ColorKey::kYellow, { 1, 1, 0.2, 1 } },
                                                                   { ColorKey::kGold, { 0.898, 0.784, 0.462, 1 } },
                                                                   { ColorKey::kGrey, { 0.705, 0.674, 0.678, 1 } },
                                                                   { ColorKey::kLavender, { 0.560, 0.501, 0.674, 1 } },
                                                                   { ColorKey::kOrange, { 1, 0.501, 0, 1 } },
                                                                   { ColorKey::kBlack, { 0, 0, 0, 1 } } };
}; // namespace pointmatcher_ros


} // namespace pointmatcher_ros