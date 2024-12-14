
#include <gtest/gtest.h>

#include <Eigen/Core>

#include "../../utest.h"

class PointContributionLocalizabilityDetectionTest : public ::testing::Test
{
public:
    // General aliases.
    using DataPoints = typename PM::DataPoints;
    using Label = typename DataPoints::Label;
    using Labels = typename DataPoints::Labels;

    // Test data.
    std::vector<DataPoints> pointClouds_;

    void SetUp() override {}

};


TEST_F(PointContributionLocalizabilityDetectionTest, TrivialCase) // NOLINT
{

}

