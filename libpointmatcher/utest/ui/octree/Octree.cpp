
#include <gtest/gtest.h>

#include <memory>
#include <queue>

#include <Eigen/Core>

#include "../../utest.h"

#include "pointmatcher/DataPointsFilters/utils/octree/Octree.h"

class OctreeTest : public ::testing::Test
{
public:
    // General aliases.
    using DataPoints = typename PM::DataPoints;
    using Label = typename DataPoints::Label;
    using Labels = typename DataPoints::Labels;
    // Octree aliases.
    using Octree = Octree_<NumericType, 3>;
    using Point = Octree::Point;
    using Vector = PM::Vector;
    using OctreeDataContainer = Octree::DataContainer;

    // Octree construction parameters.
    double maxSizeByNode{ 0.01 };
    size_t maxPointByNode{ 1 };
    bool buildParallel{ false };
    bool centerAtOrigin{ false };

    // Test data.
    std::vector<DataPoints> pointClouds_;

    void SetUp() override {}

    void printOctreeContent(Octree& octreeRoot)
    {
        // Iterative traversal of octree with Breadth-First-Search.
        std::queue<Octree*> queue;
        queue.push(&octreeRoot);
        size_t nodeIndex{ 0u };
        while (!queue.empty())
        {
            // Fetch front of the queue and pop element.
            Octree* octreeNode{ queue.front() };
            queue.pop();

            std::cout << "Node index = " << nodeIndex << "\n";
            std::cout << "Depth = " << octreeNode->getDepth() << "\n";
            std::cout << "Is root? " << octreeNode->isRoot() << "\n";
            std::cout << "Is leaf? " << octreeNode->isLeaf() << "\n";
            std::cout << "Is empty? " << octreeNode->isEmpty() << "\n";
            std::cout << "Bounding box radius = " << octreeNode->getRadius() << "\n";
            std::cout << "Bounding box center = " << octreeNode->getCenter().transpose() << "\n";
            auto* points{ octreeNode->getData() };
            size_t numPoints{ points->size() };
            std::cout << "Number of points = " << numPoints << "\n";
            if (numPoints != 0)
            {
                std::cout << "Points:\n";
                for (size_t i{ 0u }; i < numPoints; ++i)
                {
                    std::cout << "  i = " << (*points)[i] << "\n";
                }
            }
            std::cout << "##########################\n";

            // Enqueue children.
            const size_t numChildren{ octreeNode->nbCells };
            std::cout << "Number of children nodes " << numChildren << std::endl;
            for (size_t i{ 0u }; i < numChildren; ++i)
            {
                Octree* child = (*octreeNode)[i];
                if (child == nullptr)
                {
                    std::cout << "NULL PTR, SKIP" << std::endl;
                    continue;
                }

                queue.push(child);
            }
            std::cout << "\n";
            nodeIndex++;
        }
    }


    DataPoints generateEmptyPointCloudContainer(const size_t pointCount, const bool is3dPointCloud)
    {
        // Features.
        Labels featLabels;
        featLabels.emplace_back(Label("x", 1));
        featLabels.emplace_back(Label("y", 1));
        if (is3dPointCloud)
        {
            featLabels.emplace_back(Label("z", 1));
        }
        featLabels.emplace_back(Label("pad", 1));

        // Descriptors.
        Labels descLabels;

        // Create point cloud.
        return DataPoints(featLabels, descLabels, pointCount);
    }

    void generateOnePointPointClouds()
    {
        const size_t numPoints{ 1u };

        // Point cloud with one point at the origin.
        {
            DataPoints pointCloud{ generateEmptyPointCloudContainer(numPoints, true) };
            pointCloud.features(0, 0) = 0.0;
            pointCloud.features(1, 0) = 0.0;
            pointCloud.features(2, 0) = 0.0;

            // Store raw and filtered point cloud..
            pointClouds_.push_back(pointCloud);
        }

        // Point cloud with one point at (1,1,-1).
        {
            DataPoints pointCloud{ generateEmptyPointCloudContainer(numPoints, true) };
            pointCloud.features(0, 0) = 1.0;
            pointCloud.features(1, 0) = 1.0;
            pointCloud.features(2, 0) = -1.0;

            // Store raw and filtered point cloud..
            pointClouds_.push_back(pointCloud);
        }
    }

    void generateFourPointPointClouds()
    {
        const size_t numPoints{ 4u };

        // Point cloud with four points, centered around the origin.
        {
            DataPoints pointCloud{ generateEmptyPointCloudContainer(numPoints, true) };
            // Point 0 (1,1,0)
            pointCloud.features(0, 0) = 1.0;
            pointCloud.features(1, 0) = 1.0;
            pointCloud.features(2, 0) = 0.0;
            // Point 1 (-1,1,0)
            pointCloud.features(0, 1) = -1.0;
            pointCloud.features(1, 1) = 1.0;
            pointCloud.features(2, 1) = 0.0;
            // Point 2 (1,-1,0)
            pointCloud.features(0, 2) = 1.0;
            pointCloud.features(1, 2) = -1.0;
            pointCloud.features(2, 2) = 0.0;
            // Point 3 (-1,-1,0)
            pointCloud.features(0, 3) = -1.0;
            pointCloud.features(1, 3) = -1.0;
            pointCloud.features(2, 3) = 0.0;

            // Store raw and filtered point cloud..
            pointClouds_.push_back(pointCloud);
        }

        // Point cloud with four points, centered around the origin, but not quite.
        {
            DataPoints pointCloud{ generateEmptyPointCloudContainer(numPoints, true) };
            // Point 0 (1,1,0)
            pointCloud.features(0, 0) = 1.5;
            pointCloud.features(1, 0) = 1.0;
            pointCloud.features(2, 0) = 0.1;
            // Point 1 (-1,1,0)
            pointCloud.features(0, 1) = -1.5;
            pointCloud.features(1, 1) = 1.0;
            pointCloud.features(2, 1) = 0.1;
            // Point 2 (1,-1,0)
            pointCloud.features(0, 2) = 1.0;
            pointCloud.features(1, 2) = -1.5;
            pointCloud.features(2, 2) = -0.0;
            // Point 3 (-1,-1,0)
            pointCloud.features(0, 3) = -1.0;
            pointCloud.features(1, 3) = -1.5;
            pointCloud.features(2, 3) = 0.1;

            // Store raw and filtered point cloud..
            pointClouds_.push_back(pointCloud);
        }
    }

    void assertOnOctreeNodeContent(Octree& octree, const size_t depth, const bool isRoot, const bool isLeaf, const bool isEmpty,
                                   const float radius, const Point& center, const OctreeDataContainer& points)
    {
        ASSERT_EQ(octree.getDepth(), depth);
        ASSERT_EQ(octree.isRoot(), isRoot);
        ASSERT_EQ(octree.isLeaf(), isLeaf);
        ASSERT_EQ(octree.isEmpty(), isEmpty);
        ASSERT_EQ(octree.getRadius(), radius);
        ASSERT_TRUE(octree.getCenter().isApprox(center))
            << "Expected center " << center.transpose() << "\n actual " << octree.getCenter().transpose();
        ASSERT_EQ(octree.getData()->size(), points.size());
    }
};

TEST_F(OctreeTest, EmptyOctree) // NOLINT
{
    Octree octree;
    assertOnOctreeNodeContent(octree, 0u, true, true, true, 0, Point::Zero(), {});
}

/* One point octree test.
 * In this test, we expect the octree to have 0 depth, contain a single node (which is both root and leaf), and 'own'
 * the only point in the point cloud data. The octree bounding box is expected to have radius = 0, while the center
 * is expected to correspond to the only point in the cloud. */
TEST_F(OctreeTest, OnePointOctreeArbitraryCenter) // NOLINT
{
    generateOnePointPointClouds();

    // Point cloud with one point at the origin.
    {
        // Expected octree characteristics.
        const size_t depth{ 0u };
        const bool isRoot{ true };
        const bool isLeaf{ true };
        const bool isEmpty{ false };
        const float radius{ 0 };
        const Point center{ Point::Zero() };
        const OctreeDataContainer points{ 0 };

        // Build octree
        const auto& pointCloud{ pointClouds_[0] };
        Octree octree;
        octree.build(pointCloud, maxPointByNode, maxSizeByNode, buildParallel, centerAtOrigin);

        // Assert on results.
        assertOnOctreeNodeContent(octree, depth, isRoot, isLeaf, isEmpty, radius, center, points);
    }


    // Point cloud with one point at (1,1,-1).
    {
        // Expected octree characteristics.
        const size_t depth{ 0u };
        const bool isRoot{ true };
        const bool isLeaf{ true };
        const bool isEmpty{ false };
        const float radius{ 0 };
        const Point center{ Point{ 1, 1, -1 } };
        const OctreeDataContainer points{ 0 };

        // Build octree
        const auto& pointCloud{ pointClouds_[1] };
        Octree octree;
        octree.build(pointCloud, maxPointByNode, maxSizeByNode, buildParallel, centerAtOrigin);

        // Assert on results.
        assertOnOctreeNodeContent(octree, depth, isRoot, isLeaf, isEmpty, radius, center, points);
    }
}

TEST_F(OctreeTest, OnePointOctreeOriginCenter) // NOLINT
{
    centerAtOrigin = true;
    generateOnePointPointClouds();

    // Point cloud with one point at the origin.
    {
        // Expected octree characteristics.
        const size_t depth{ 0u };
        const bool isRoot{ true };
        const bool isLeaf{ true };
        const bool isEmpty{ false };
        const float radius{ 0 };
        const Point center{ Point::Zero() };
        const OctreeDataContainer points{ 0 };

        // Build octree
        const auto& pointCloud{ pointClouds_[0] };
        Octree octree;
        octree.build(pointCloud, maxPointByNode, maxSizeByNode, buildParallel, centerAtOrigin);

        // Assert on results.
        assertOnOctreeNodeContent(octree, depth, isRoot, isLeaf, isEmpty, radius, center, points);
    }


    // Point cloud with one point at (1,1,-1).
    {
        // Expected octree characteristics.
        const size_t depth{ 0u };
        const bool isRoot{ true };
        const bool isLeaf{ true };
        const bool isEmpty{ false };
        const float radius{ 0 };
        const Point center{ Point{ 0, 0, -0 } };
        const OctreeDataContainer points{ 0 };

        // Build octree
        const auto& pointCloud{ pointClouds_[1] };
        Octree octree;
        octree.build(pointCloud, maxPointByNode, maxSizeByNode, buildParallel, centerAtOrigin);

        // Assert on results.
        assertOnOctreeNodeContent(octree, depth, isRoot, isLeaf, isEmpty, radius, center, points);
    }
}

/* Four point octree test.
 * In this test, we expect the octree to have 1 depth, contain a 1+8 nodes(a root and 8 children), 
 * with the points from the cloud distributed between the children. */
TEST_F(OctreeTest, FourPointOctreeArbitraryCenter) // NOLINT
{
    generateFourPointPointClouds();

    // Point cloud with four points, centered around the origin.
    {
        // Build octree
        const auto& pointCloud{ pointClouds_[0] };
        Octree octree;
        octree.build(pointCloud, maxPointByNode, maxSizeByNode, buildParallel, centerAtOrigin);

        // printOctreeContent(octree);

        // Assert on top-level octree node.
        // Assertion function signature:
        //  assertOnOctreeNodeContent(octree, depth, isRoot, isLeaf, isEmpty, radius, center, points);
        assertOnOctreeNodeContent(octree, 0u, true, false, true, 1, Point::Zero(), {});
        assertOnOctreeNodeContent(*(octree[0]), 1u, false, true, false, 0.5, Point(-0.5, -0.5, -0.5), { 3 });
        assertOnOctreeNodeContent(*(octree[1]), 1u, false, true, false, 0.5, Point(0.5, -0.5, -0.5), { 2 });
        assertOnOctreeNodeContent(*(octree[2]), 1u, false, true, false, 0.5, Point(-0.5, 0.5, -0.5), { 1 });
        assertOnOctreeNodeContent(*(octree[3]), 1u, false, true, false, 0.5, Point(0.5, 0.5, -0.5), { 0 });
        assertOnOctreeNodeContent(*(octree[4]), 1u, false, true, true, 0.5, Point(-0.5, -0.5, 0.5), {});
        assertOnOctreeNodeContent(*(octree[5]), 1u, false, true, true, 0.5, Point(0.5, -0.5, 0.5), {});
        assertOnOctreeNodeContent(*(octree[6]), 1u, false, true, true, 0.5, Point(-0.5, 0.5, 0.5), {});
        assertOnOctreeNodeContent(*(octree[7]), 1u, false, true, true, 0.5, Point(0.5, 0.5, 0.5), {});
    }

    // Point cloud with four points, centered around the origin, but not quite.
    {
        const auto& pointCloud{ pointClouds_[1] };
        Octree octree;
        octree.build(pointCloud, maxPointByNode, maxSizeByNode, buildParallel, centerAtOrigin);

        // printOctreeContent(octree);

        // Compute expected parameters of bounding box.
        Vector minValues{ pointCloud.features.rowwise().minCoeff() };
        Vector maxValues{ pointCloud.features.rowwise().maxCoeff() };
        Point radii{ maxValues.head(3) - minValues.head(3) };
        const Point expectedOctreeCenter{ minValues.head(3) + radii * 0.5 };
        const float expectedOctreeRadius{ 2 };

        // Assert on top-level octree node.
        // Assertion function signature:
        //  assertOnOctreeNodeContent(octree, depth, isRoot, isLeaf, isEmpty, radius, center, points);
        assertOnOctreeNodeContent(octree, 0u, true, false, true, expectedOctreeRadius, expectedOctreeCenter, {});
        assertOnOctreeNodeContent(*(octree[0]), 1u, false, true, true, 1.0, Point(-1, -1.25, -0.95), {});
        assertOnOctreeNodeContent(*(octree[1]), 1u, false, true, false, 1.0, Point(1, -1.25, -0.95), { 2 });
        assertOnOctreeNodeContent(*(octree[2]), 1u, false, true, true, 1.0, Point(-1, 0.75, -0.95), {});
        assertOnOctreeNodeContent(*(octree[3]), 1u, false, true, true, 1.0, Point(1, 0.75, -0.95), {});
        assertOnOctreeNodeContent(*(octree[4]), 1u, false, true, false, 1.0, Point(-1, -1.25, 1.05), { 3 });
        assertOnOctreeNodeContent(*(octree[5]), 1u, false, true, true, 1.0, Point(1, -1.25, 1.05), {});
        assertOnOctreeNodeContent(*(octree[6]), 1u, false, true, false, 1.0, Point(-1, 0.75, 1.05), { 1 });
        assertOnOctreeNodeContent(*(octree[7]), 1u, false, true, false, 1.0, Point(1, 0.75, 1.05), { 0 });
    }
}

TEST_F(OctreeTest, FourPointOctreeOriginCenter) // NOLINT
{
    centerAtOrigin = true;
    generateFourPointPointClouds();

    // Point cloud with four points, centered around the origin.
    {
        // Build octree
        const auto& pointCloud{ pointClouds_[0] };
        Octree octree;
        octree.build(pointCloud, maxPointByNode, maxSizeByNode, buildParallel, centerAtOrigin);

        // printOctreeContent(octree);

        // Assert on top-level octree node.
        // Assertion function signature:
        //  assertOnOctreeNodeContent(octree, depth, isRoot, isLeaf, isEmpty, radius, center, points);
        assertOnOctreeNodeContent(octree, 0u, true, false, true, 1, Point::Zero(), {});
        assertOnOctreeNodeContent(*(octree[0]), 1u, false, true, false, 0.5, Point(-0.5, -0.5, -0.5), { 3 });
        assertOnOctreeNodeContent(*(octree[1]), 1u, false, true, false, 0.5, Point(0.5, -0.5, -0.5), { 2 });
        assertOnOctreeNodeContent(*(octree[2]), 1u, false, true, false, 0.5, Point(-0.5, 0.5, -0.5), { 1 });
        assertOnOctreeNodeContent(*(octree[3]), 1u, false, true, false, 0.5, Point(0.5, 0.5, -0.5), { 0 });
        assertOnOctreeNodeContent(*(octree[4]), 1u, false, true, true, 0.5, Point(-0.5, -0.5, 0.5), {});
        assertOnOctreeNodeContent(*(octree[5]), 1u, false, true, true, 0.5, Point(0.5, -0.5, 0.5), {});
        assertOnOctreeNodeContent(*(octree[6]), 1u, false, true, true, 0.5, Point(-0.5, 0.5, 0.5), {});
        assertOnOctreeNodeContent(*(octree[7]), 1u, false, true, true, 0.5, Point(0.5, 0.5, 0.5), {});
    }

    // Point cloud with four points, centered around the origin, but not quite.
    {
        const auto& pointCloud{ pointClouds_[1] };
        Octree octree;
        octree.build(pointCloud, maxPointByNode, maxSizeByNode, buildParallel, centerAtOrigin);

        // printOctreeContent(octree);

        // Compute expected parameters of bounding box.
        const Point expectedOctreeCenter{ 0.0, 0.0, 0.0 };
        const float expectedOctreeRadius{ 2 };

        // Assert on top-level octree node.
        // Assertion function signature:
        //  assertOnOctreeNodeContent(octree, depth, isRoot, isLeaf, isEmpty, radius, center, points);
        assertOnOctreeNodeContent(octree, 0u, true, false, true, expectedOctreeRadius, expectedOctreeCenter, {});
        assertOnOctreeNodeContent(*(octree[0]), 1u, false, true, true, 1.0, Point(-1, -1, -1), {});
        assertOnOctreeNodeContent(*(octree[1]), 1u, false, true, false, 1.0, Point(1, -1, -1), { 2 });
        assertOnOctreeNodeContent(*(octree[2]), 1u, false, true, true, 1.0, Point(-1, 1, -1), {});
        assertOnOctreeNodeContent(*(octree[3]), 1u, false, true, true, 1.0, Point(1, 1, -1), {});
        assertOnOctreeNodeContent(*(octree[4]), 1u, false, true, false, 1.0, Point(-1, -1, 1), { 3 });
        assertOnOctreeNodeContent(*(octree[5]), 1u, false, true, true, 1.0, Point(1, -1, 1), {});
        assertOnOctreeNodeContent(*(octree[6]), 1u, false, true, false, 1.0, Point(-1, 1, 1), { 1 });
        assertOnOctreeNodeContent(*(octree[7]), 1u, false, true, false, 1.0, Point(1, 1, 1), { 0 });
    }
}
