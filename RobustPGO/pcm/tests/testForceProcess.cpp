/**
 * @file    testDoOptimize.cpp
 * @brief   Unit test for pcm and optimize conditions
 * @author  Yun Chang
 */

#include <CppUnitLite/TestHarness.h>
#include <random>

#include "RobustPGO/pcm/pcm.h" 

/* ************************************************************************* */
TEST(ForceOptimize, LC)
{
  // test that when opdemtry edge is received added but return false
  OutlierRemoval *pcm = new PCM<gtsam::Pose3>(1.0, 1.0);
  pcm->setQuiet();

  static const gtsam::SharedNoiseModel& noise = 
      gtsam::noiseModel::Isotropic::Variance(6, 0.01);   

  gtsam::NonlinearFactorGraph nfg; 
  gtsam::Values est; 

  // initialize first (w/ prior)
  gtsam::Values init_vals;
  gtsam::NonlinearFactorGraph init_factors; 
  init_vals.insert(0, gtsam::Pose3());
  init_factors.add(gtsam::PriorFactor<gtsam::Pose3>(0, gtsam::Pose3(), noise));
  pcm->process(init_factors, init_vals, nfg, est);

  EXPECT(gtsam::assert_equal(size_t(1), nfg.size()));
  EXPECT(gtsam::assert_equal(size_t(1), est.size()));

  // add odometry edges
  gtsam::Values vals;
  gtsam::NonlinearFactorGraph factors; 
  gtsam::Pose3 odom = gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1,0,0));
  vals.insert(1, odom);
  factors.add(gtsam::BetweenFactor<gtsam::Pose3>(0, 1, odom, noise));
  pcm->process(factors, vals, nfg, est);
  vals = gtsam::Values();
  factors = gtsam::NonlinearFactorGraph();
  vals.insert(2, odom.compose(odom));
  factors.add(gtsam::BetweenFactor<gtsam::Pose3>(1, 2, odom, noise));
  bool do_optimize = pcm->process(factors, vals, nfg, est);
  EXPECT(gtsam::assert_equal(size_t(3), nfg.size()));
  EXPECT(gtsam::assert_equal(size_t(3), est.size()));
  EXPECT(do_optimize == false);

  // now try bad loop closure with reg update 
  gtsam::NonlinearFactorGraph lc_factors; 
  lc_factors.add(gtsam::BetweenFactor<gtsam::Pose3>(2,0,gtsam::Pose3(),noise));
  do_optimize = pcm->process(lc_factors, gtsam::Values(), nfg, est);
  EXPECT(gtsam::assert_equal(size_t(3), nfg.size()));
  EXPECT(gtsam::assert_equal(size_t(3), est.size()));
  EXPECT(do_optimize == false);

  // now try bad loop closure with forced update 
  do_optimize = pcm->processForcedLoopclosure(lc_factors, gtsam::Values(), nfg, est);
  EXPECT(gtsam::assert_equal(size_t(4), nfg.size()));
  EXPECT(gtsam::assert_equal(size_t(3), est.size()));
  EXPECT(do_optimize == true);

  // now add good loop closure with regular (forced update should remain)
  lc_factors = gtsam::NonlinearFactorGraph();
  lc_factors.add(gtsam::BetweenFactor<gtsam::Pose3>(1,0,odom.inverse(),noise));
  do_optimize = pcm->processForcedLoopclosure(lc_factors, gtsam::Values(), nfg, est);
  EXPECT(gtsam::assert_equal(size_t(5), nfg.size()));
  EXPECT(gtsam::assert_equal(size_t(3), est.size()));
  EXPECT(do_optimize == true);
}


/* ************************************************************************* */
  int main() { TestResult tr; return TestRegistry::runAllTests(tr);}
/* ************************************************************************* */
