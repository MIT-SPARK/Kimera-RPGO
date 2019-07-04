/**
 * @file    testDoOptimize.cpp
 * @brief   Unit test for pcm and optimize conditions
 * @author  Yun Chang
 */

#include <CppUnitLite/TestHarness.h>
#include <random>

#include "RobustPGO/pcm/pcm.h" 

/* ************************************************************************* */
TEST(DoOptimize, Odometry)
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

  EXPECT(gtsam::assert_equal(nfg.size(), size_t(1)));
  EXPECT(gtsam::assert_equal(est.size(), size_t(1)));

  // add odometry 
  gtsam::Values vals;
  gtsam::NonlinearFactorGraph factors; 
  gtsam::Pose3 odom = gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1,0,0));
  vals.insert(1, odom);
  factors.add(gtsam::BetweenFactor<gtsam::Pose3>(0, 1, odom, noise));
  bool do_optimize = pcm->process(factors, vals, nfg, est);
  EXPECT(gtsam::assert_equal(nfg.size(), size_t(2)));
  EXPECT(gtsam::assert_equal(est.size(), size_t(2)));
  EXPECT(do_optimize == false);
}

TEST(DoOptimize, OdometryNoPrior)
{
  // test that when opdemtry edge is received added but return false
  OutlierRemoval *pcm = new PCM<gtsam::Pose3>(1.0, 1.0);
  pcm->setQuiet();

  static const gtsam::SharedNoiseModel& noise = 
      gtsam::noiseModel::Isotropic::Variance(6, 0.01);   

  gtsam::NonlinearFactorGraph nfg; 
  gtsam::Values est; 

  // initialize first (w/o prior)
  gtsam::Values init_vals;
  init_vals.insert(0, gtsam::Pose3());
  pcm->process(gtsam::NonlinearFactorGraph(), init_vals, nfg, est);

  EXPECT(gtsam::assert_equal(nfg.size(), size_t(0)));
  EXPECT(gtsam::assert_equal(est.size(), size_t(1)));

  // add odometry 
  gtsam::Values vals;
  gtsam::NonlinearFactorGraph factors; 
  gtsam::Pose3 odom = gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1,0,0));
  vals.insert(1, odom);
  factors.add(gtsam::BetweenFactor<gtsam::Pose3>(0, 1, odom, noise));
  bool do_optimize = pcm->process(factors, vals, nfg, est);
  EXPECT(gtsam::assert_equal(nfg.size(), size_t(1)));
  EXPECT(gtsam::assert_equal(est.size(), size_t(2)));
  EXPECT(do_optimize == false);
}
/* ************************************************************************* */
TEST(DoOptimize, LoopClosure)
{
  // test that when loop closure edge is received added and return true 
  OutlierRemoval *pcm = new PCM<gtsam::Pose3>(1.0, 1.0);
  pcm->setQuiet();

  static const gtsam::SharedNoiseModel& noise = 
      gtsam::noiseModel::Isotropic::Variance(6, 0.01);   

  gtsam::NonlinearFactorGraph nfg; 
  gtsam::Values est; 

  // initialize first (w/o prior)
  gtsam::Values init_vals;
  init_vals.insert(0, gtsam::Pose3());
  pcm->process(gtsam::NonlinearFactorGraph(), init_vals, nfg, est);

  // add odometry 
  gtsam::Values vals;
  gtsam::NonlinearFactorGraph factors; 
  gtsam::Pose3 odom = gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1,0,0));
  vals.insert(1, odom);
  factors.add(gtsam::BetweenFactor<gtsam::Pose3>(0, 1, odom, noise));
  pcm->process(factors, vals, nfg, est);

  // and aother odom
  vals = gtsam::Values();
  factors = gtsam::NonlinearFactorGraph(); //reset
  gtsam::Pose3 odom2 = gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(-1,0,0));
  vals.insert(2, odom.compose(odom2));
  factors.add(gtsam::BetweenFactor<gtsam::Pose3>(1, 2, odom2, noise));
  pcm->process(factors, vals, nfg, est);

  EXPECT(gtsam::assert_equal(nfg.size(), size_t(2)));
  EXPECT(gtsam::assert_equal(est.size(), size_t(3)));

  // loop closure
  factors = gtsam::NonlinearFactorGraph(); //reset
  factors.add(gtsam::BetweenFactor<gtsam::Pose3>(2, 0, gtsam::Pose3(), noise));
  bool do_optimize = pcm->process(factors, gtsam::Values(), nfg, est);

  EXPECT(gtsam::assert_equal(nfg.size(), size_t(3)));
  EXPECT(gtsam::assert_equal(est.size(), size_t(3)));
  EXPECT(do_optimize == true);
}

/* ************************************************************************* */
  int main() { TestResult tr; return TestRegistry::runAllTests(tr);}
/* ************************************************************************* */
