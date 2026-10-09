#include <gtest/gtest.h>
#include <cmath>
#include "arm_interface/gripper_object_model.h"

using arm_interface::GripperLoadWithContact;
using arm_interface::GripperMeasuredWithContact;

TEST(GripperObjectModel, NoContactPassesTheCommandThrough) {
  const double none = std::nan("");
  EXPECT_DOUBLE_EQ(GripperMeasuredWithContact(0.0, none), 0.0);
  EXPECT_DOUBLE_EQ(GripperMeasuredWithContact(0.02, none), 0.02);
  EXPECT_DOUBLE_EQ(GripperLoadWithContact(0.0, none, 5.0, 2.5), 2.5);
}

TEST(GripperObjectModel, ClosingStopsAtTheContact) {
  EXPECT_DOUBLE_EQ(GripperMeasuredWithContact(0.02, 0.012), 0.02);   // still open wider than the object
  EXPECT_DOUBLE_EQ(GripperMeasuredWithContact(0.012, 0.012), 0.012);
  EXPECT_DOUBLE_EQ(GripperMeasuredWithContact(0.0, 0.012), 0.012);   // commanded past it: blocked
}

TEST(GripperObjectModel, LoadGrowsWithTheSqueezeAndSaturates) {
  EXPECT_DOUBLE_EQ(GripperLoadWithContact(0.02, 0.012, 5.0, 2.5), 2.5);      // not in contact
  EXPECT_NEAR(GripperLoadWithContact(0.004, 0.012, 5.0, 2.5), 42.5, 1e-9);  // 8 mm past the contact x 5 %/mm + baseline
  EXPECT_DOUBLE_EQ(GripperLoadWithContact(-1.0, 0.012, 5.0, 2.5), 100.0);   // saturated
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
