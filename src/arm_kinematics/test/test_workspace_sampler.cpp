// Unit tests for workspace_sampler.h. EstimateVolumeByVoxelCounting is
// pure and RNG-free, so it's tested directly against hand-picked points
// (exact expected voxel counts, not just "some positive number"). The
// sampling side (SampleReachableWorkspace) can't be checked against a
// hand-computed reference the way M1's FK tests are -- there's no
// independent ground truth for "the reachable workspace" to compare
// against -- so it's instead checked for internal consistency (returned
// stats match the returned points exactly), determinism (reproducibility
// matters here: the report's numbers should be regeneratable), and a
// genuine independent geometric bound: no chain of rigid links can reach
// farther than the sum of its own segment lengths, computed here by
// walking the URDF directly (not reusing ForwardKinematicsSolver's
// internals) -- the same "independent second computation" philosophy as
// test_forward_kinematics.cpp.

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include <ros/package.h>
#include <urdf/model.h>

#include "arm_kinematics/forward_kinematics.h"
#include "arm_kinematics/workspace_sampler.h"

using arm_kinematics::EstimateVolumeByVoxelCounting;
using arm_kinematics::SampleReachableWorkspace;

namespace {

urdf::Model LoadArmModel() {
  urdf::Model model;
  std::string arm_interface_path = ros::package::getPath("arm_interface");
  EXPECT_FALSE(arm_interface_path.empty()) << "arm_interface package not found";
  EXPECT_TRUE(model.initFile(arm_interface_path + "/urdf/arm.urdf"));
  return model;
}

// Independent computation of the kinematic chain's maximum possible reach
// (sum of segment lengths root -> tip): walks the URDF's link->parent_joint
// chain directly, the same traversal ForwardKinematicsSolver's own header
// comment describes it using internally, but reimplemented here rather
// than calling into it -- ||p_tip|| <= this bound for ANY joint
// configuration, regardless of joint limits, since it's just the triangle
// inequality applied along the chain of rigid segments.
double ChainReachUpperBound(const urdf::Model& model, const std::string& root_link,
                            const std::string& tip_link) {
  double bound = 0.0;
  urdf::LinkConstSharedPtr link = model.getLink(tip_link);
  EXPECT_TRUE(link != nullptr) << "tip_link not found: " << tip_link;
  while (link && link->name != root_link) {
    EXPECT_TRUE(link->parent_joint != nullptr)
        << "link '" << link->name << "' has no parent joint but isn't root_link";
    if (!link->parent_joint) break;
    const auto& origin = link->parent_joint->parent_to_joint_origin_transform.position;
    bound += std::sqrt(origin.x * origin.x + origin.y * origin.y + origin.z * origin.z);
    link = model.getLink(link->parent_joint->parent_link_name);
  }
  return bound;
}

}  // namespace

TEST(EstimateVolumeByVoxelCounting, EmptyInputGivesZeroVolume) {
  auto result = EstimateVolumeByVoxelCounting({}, 0.01);
  EXPECT_EQ(result.occupied_voxel_count, 0u);
  EXPECT_DOUBLE_EQ(result.volume_m3, 0.0);
}

TEST(EstimateVolumeByVoxelCounting, SinglePointGivesExactlyOneVoxel) {
  auto result = EstimateVolumeByVoxelCounting({Eigen::Vector3d(0.123, -0.456, 0.789)}, 0.01);
  EXPECT_EQ(result.occupied_voxel_count, 1u);
  EXPECT_NEAR(result.volume_m3, 0.01 * 0.01 * 0.01, 1e-15);
}

TEST(EstimateVolumeByVoxelCounting, PointsInTheSameVoxelCountOnce) {
  // Both within a 0.1m voxel starting at (0,0,0): [0, 0.1) each axis.
  std::vector<Eigen::Vector3d> points = {Eigen::Vector3d(0.01, 0.01, 0.01),
                                          Eigen::Vector3d(0.09, 0.02, 0.05)};
  auto result = EstimateVolumeByVoxelCounting(points, 0.1);
  EXPECT_EQ(result.occupied_voxel_count, 1u);
}

TEST(EstimateVolumeByVoxelCounting, PointsInDifferentVoxelsCountSeparately) {
  std::vector<Eigen::Vector3d> points = {Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Vector3d(1.0, 0.0, 0.0),
                                          Eigen::Vector3d(2.0, 0.0, 0.0)};
  auto result = EstimateVolumeByVoxelCounting(points, 0.1);
  EXPECT_EQ(result.occupied_voxel_count, 3u);
  EXPECT_NEAR(result.volume_m3, 3.0 * 0.1 * 0.1 * 0.1, 1e-12);
}

TEST(EstimateVolumeByVoxelCounting, HandlesNegativeCoordinatesConsistently) {
  // A point just below zero and one just above, 0.1m apart in a 0.1m grid,
  // straddle the origin voxel boundary -- must land in different voxels,
  // not be aliased onto the same one by careless negative-floor handling.
  std::vector<Eigen::Vector3d> points = {Eigen::Vector3d(-0.05, 0.0, 0.0), Eigen::Vector3d(0.05, 0.0, 0.0)};
  auto result = EstimateVolumeByVoxelCounting(points, 0.1);
  EXPECT_EQ(result.occupied_voxel_count, 2u);
}

class WorkspaceSamplerTest : public ::testing::Test {
 protected:
  void SetUp() override {
    model_ = LoadArmModel();
    solver_ = std::make_unique<arm_kinematics::ForwardKinematicsSolver>(model_, "base_link",
                                                                          "end_effector");
  }

  urdf::Model model_;
  std::unique_ptr<arm_kinematics::ForwardKinematicsSolver> solver_;
};

TEST_F(WorkspaceSamplerTest, ReturnsExactlyNumSamplesPoints) {
  auto result = SampleReachableWorkspace(*solver_, 500, 0.02);
  EXPECT_EQ(result.points.size(), 500u);
}

TEST_F(WorkspaceSamplerTest, SameSeedIsFullyReproducible) {
  auto a = SampleReachableWorkspace(*solver_, 300, 0.02, /*rng_seed=*/7);
  auto b = SampleReachableWorkspace(*solver_, 300, 0.02, /*rng_seed=*/7);
  ASSERT_EQ(a.points.size(), b.points.size());
  for (size_t i = 0; i < a.points.size(); ++i) {
    EXPECT_TRUE(a.points[i].isApprox(b.points[i], 1e-15))
        << "same seed should reproduce byte-for-byte identical samples (the report's numbers "
           "need to be regeneratable)";
  }
  EXPECT_DOUBLE_EQ(a.min_reach, b.min_reach);
  EXPECT_DOUBLE_EQ(a.max_reach, b.max_reach);
}

TEST_F(WorkspaceSamplerTest, DifferentSeedsGiveDifferentSamples) {
  auto a = SampleReachableWorkspace(*solver_, 300, 0.02, /*rng_seed=*/1);
  auto b = SampleReachableWorkspace(*solver_, 300, 0.02, /*rng_seed=*/2);
  EXPECT_FALSE(a.points[0].isApprox(b.points[0], 1e-9));
}

TEST_F(WorkspaceSamplerTest, MinMaxReachAndBboxAreConsistentWithReturnedPoints) {
  auto result = SampleReachableWorkspace(*solver_, 2000, 0.02, /*rng_seed=*/3);

  double expected_min = std::numeric_limits<double>::infinity();
  double expected_max = 0.0;
  Eigen::Vector3d expected_bbox_min = Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());
  Eigen::Vector3d expected_bbox_max = Eigen::Vector3d::Constant(-std::numeric_limits<double>::infinity());
  for (const auto& p : result.points) {
    expected_min = std::min(expected_min, p.norm());
    expected_max = std::max(expected_max, p.norm());
    expected_bbox_min = expected_bbox_min.cwiseMin(p);
    expected_bbox_max = expected_bbox_max.cwiseMax(p);
  }

  EXPECT_DOUBLE_EQ(result.min_reach, expected_min);
  EXPECT_DOUBLE_EQ(result.max_reach, expected_max);
  EXPECT_TRUE(result.bbox_min.isApprox(expected_bbox_min, 1e-15));
  EXPECT_TRUE(result.bbox_max.isApprox(expected_bbox_max, 1e-15));
}

TEST_F(WorkspaceSamplerTest, VolumeEstimateMatchesDirectVoxelCountOfTheSamePoints) {
  auto result = SampleReachableWorkspace(*solver_, 2000, 0.03, /*rng_seed=*/4);
  auto direct = EstimateVolumeByVoxelCounting(result.points, 0.03);
  EXPECT_EQ(result.volume.occupied_voxel_count, direct.occupied_voxel_count);
  EXPECT_DOUBLE_EQ(result.volume.volume_m3, direct.volume_m3);
}

TEST_F(WorkspaceSamplerTest, NoSampleExceedsTheChainsGeometricReachBound) {
  // Independent bound (see ChainReachUpperBound), not derived from
  // GetActuatedJointLimits/ComputePose at all -- catches a genuine
  // correctness bug (e.g. sampling the wrong joint, a unit error) rather
  // than just re-checking the sampler against itself.
  double bound = ChainReachUpperBound(model_, "base_link", "end_effector");
  auto result = SampleReachableWorkspace(*solver_, 5000, 0.02, /*rng_seed=*/5);
  EXPECT_LE(result.max_reach, bound + 1e-9)
      << "no configuration of a rigid chain can reach farther than the sum of its own segment "
         "lengths (" << bound << " m)";
}

TEST_F(WorkspaceSamplerTest, RejectsNonPositiveNumSamples) {
  EXPECT_THROW(SampleReachableWorkspace(*solver_, 0, 0.02), std::invalid_argument);
  EXPECT_THROW(SampleReachableWorkspace(*solver_, -5, 0.02), std::invalid_argument);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
