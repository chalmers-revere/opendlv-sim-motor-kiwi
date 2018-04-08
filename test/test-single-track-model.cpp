/*
 * Copyright (C) 2018 Ola Benderius
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#define CATCH_CONFIG_MAIN  // This tells Catch to provide a main() - only do this in one cpp file
#include "catch.hpp"

#include "cluon-complete.hpp"
#include "opendlv-standard-message-set.hpp"

#include "single-track-model.hpp"

TEST_CASE("Test single track model, zero speed should always give zero output.") {
  SingleTrackModel stm;

  opendlv::proxy::GroundSteeringRequest gsr;
  gsr.groundSteering(0.3f);
  stm.setGroundSteeringAngle(gsr);

  opendlv::proxy::PedalPositionRequest ppr;
  ppr.position(0.0f);
  stm.setPedalPosition(ppr);

  opendlv::sim::KinematicState ks = stm.step(0.01);
  REQUIRE(ks.vx() == Approx(0.0f));
  REQUIRE(ks.vy() == Approx(0.0f));
  REQUIRE(ks.vz() == Approx(0.0f));
  REQUIRE(ks.rollRate() == Approx(0.0f));
  REQUIRE(ks.pitchRate() == Approx(0.0f));
  REQUIRE(ks.yawRate() == Approx(0.0f));
}

TEST_CASE("Test single track model, going straight ahead should give positive longitudinal constat behaviour.") {
  SingleTrackModel stm;

  opendlv::proxy::GroundSteeringRequest gsr;
  gsr.groundSteering(0.0f);
  stm.setGroundSteeringAngle(gsr);

  opendlv::proxy::PedalPositionRequest ppr;
  ppr.position(0.2f);
  stm.setPedalPosition(ppr);

  float prevLongitudinalSpeed{0.0f};
  float longitudinalSpeed{0.0f};
  for (uint16_t i{0}; i < 100; i++) {
    opendlv::sim::KinematicState ks = stm.step(0.01);
    prevLongitudinalSpeed = longitudinalSpeed;
    longitudinalSpeed = ks.vx();
    REQUIRE(longitudinalSpeed >= 0.0f);

    REQUIRE(ks.vy() == Approx(0.0f));
    REQUIRE(ks.vz() == Approx(0.0f));
    REQUIRE(ks.rollRate() == Approx(0.0f));
    REQUIRE(ks.pitchRate() == Approx(0.0f));
    REQUIRE(ks.yawRate() == Approx(0.0f));
  }
  std::cout << longitudinalSpeed << " " << prevLongitudinalSpeed << std::endl;
  REQUIRE(longitudinalSpeed == Approx(prevLongitudinalSpeed).epsilon(0.01));
}

TEST_CASE("Test single track model, going in reverse should give negative longitudinal constat behaviour.") {
  SingleTrackModel stm;

  opendlv::proxy::GroundSteeringRequest gsr;
  gsr.groundSteering(0.0f);
  stm.setGroundSteeringAngle(gsr);

  opendlv::proxy::PedalPositionRequest ppr;
  ppr.position(-0.2f);
  stm.setPedalPosition(ppr);

  float prevLongitudinalSpeed{0.0f};
  float longitudinalSpeed{0.0f};
  for (uint16_t i{0}; i < 100; i++) {
    opendlv::sim::KinematicState ks = stm.step(0.01);
    prevLongitudinalSpeed = longitudinalSpeed;
    longitudinalSpeed = ks.vx();
    REQUIRE(longitudinalSpeed <= 0.0f);

    REQUIRE(ks.vy() == Approx(0.0f));
    REQUIRE(ks.vz() == Approx(0.0f));
    REQUIRE(ks.rollRate() == Approx(0.0f));
    REQUIRE(ks.pitchRate() == Approx(0.0f));
    REQUIRE(ks.yawRate() == Approx(0.0f));
  }
  REQUIRE(longitudinalSpeed == Approx(prevLongitudinalSpeed).epsilon(0.01));
}

TEST_CASE("Test single track model, steering to the left should give movement to the left and CCW rotation, and find steady state.") {
  SingleTrackModel stm;

  opendlv::proxy::GroundSteeringRequest gsr;
  gsr.groundSteering(0.3f);
  stm.setGroundSteeringAngle(gsr);

  opendlv::proxy::PedalPositionRequest ppr;
  ppr.position(0.2f);
  stm.setPedalPosition(ppr);

  float prevLateralSpeed{0.0f};
  float prevLongitudinalSpeed{0.0f};
  float prevYawRate{0.0f};
  float lateralSpeed{0.0f};
  float longitudinalSpeed{0.0f};
  float yawRate{0.0f};
  for (uint16_t i{0}; i < 100; i++) {
    opendlv::sim::KinematicState ks = stm.step(0.01);
    prevLateralSpeed = lateralSpeed;
    prevLongitudinalSpeed = longitudinalSpeed;
    prevYawRate = yawRate;
    lateralSpeed = ks.vy();
    longitudinalSpeed = ks.vx();
    yawRate = ks.yawRate();
    REQUIRE(lateralSpeed >= 0.0f);
    REQUIRE(yawRate >= 0.0f);
  }
  REQUIRE(lateralSpeed == Approx(prevLateralSpeed).epsilon(0.01));
  REQUIRE(longitudinalSpeed == Approx(prevLongitudinalSpeed).epsilon(0.01));
  REQUIRE(yawRate == Approx(prevYawRate).epsilon(0.01));
}

TEST_CASE("Test single track model, steering to the left while reversing should give movement to the left and CW rotation, and find steady state.") {
  SingleTrackModel stm;

  opendlv::proxy::GroundSteeringRequest gsr;
  gsr.groundSteering(0.3f);
  stm.setGroundSteeringAngle(gsr);

  opendlv::proxy::PedalPositionRequest ppr;
  ppr.position(-0.2f);
  stm.setPedalPosition(ppr);

  float prevLateralSpeed{0.0f};
  float prevLongitudinalSpeed{0.0f};
  float prevYawRate{0.0f};
  float lateralSpeed{0.0f};
  float longitudinalSpeed{0.0f};
  float yawRate{0.0f};
  for (uint16_t i{0}; i < 100; i++) {
    opendlv::sim::KinematicState ks = stm.step(0.01);
    prevLateralSpeed = lateralSpeed;
    prevLongitudinalSpeed = longitudinalSpeed;
    prevYawRate = yawRate;
    lateralSpeed = ks.vy();
    longitudinalSpeed = ks.vx();
    yawRate = ks.yawRate();
    REQUIRE(lateralSpeed >= 0.0f);
    REQUIRE(yawRate >= 0.0f);
  }
  REQUIRE(lateralSpeed == Approx(prevLateralSpeed).epsilon(0.01));
  REQUIRE(longitudinalSpeed == Approx(prevLongitudinalSpeed).epsilon(0.01));
  REQUIRE(yawRate == Approx(prevYawRate).epsilon(0.01));
}

TEST_CASE("Test single track model, steering to the right should give movement to the right and CW rotation, and find steady state.") {
  SingleTrackModel stm;

  opendlv::proxy::GroundSteeringRequest gsr;
  gsr.groundSteering(-0.3f);
  stm.setGroundSteeringAngle(gsr);

  opendlv::proxy::PedalPositionRequest ppr;
  ppr.position(0.2f);
  stm.setPedalPosition(ppr);

  float prevLateralSpeed{0.0f};
  float prevLongitudinalSpeed{0.0f};
  float prevYawRate{0.0f};
  float lateralSpeed{0.0f};
  float longitudinalSpeed{0.0f};
  float yawRate{0.0f};
  for (uint16_t i{0}; i < 100; i++) {
    opendlv::sim::KinematicState ks = stm.step(0.01);
    prevLateralSpeed = lateralSpeed;
    prevLongitudinalSpeed = longitudinalSpeed;
    prevYawRate = yawRate;
    lateralSpeed = ks.vy();
    longitudinalSpeed = ks.vx();
    yawRate = ks.yawRate();
    REQUIRE(lateralSpeed <= 0.0f);
    REQUIRE(yawRate <= 0.0f);
  }
  REQUIRE(lateralSpeed == Approx(prevLateralSpeed).epsilon(0.01));
  REQUIRE(longitudinalSpeed == Approx(prevLongitudinalSpeed).epsilon(0.01));
  REQUIRE(yawRate == Approx(prevYawRate).epsilon(0.01));
}

TEST_CASE("Test single track model, steering to the right while reversing should give movement to the right and CCW rotation, and find steady state.") {
  SingleTrackModel stm;

  opendlv::proxy::GroundSteeringRequest gsr;
  gsr.groundSteering(-0.3f);
  stm.setGroundSteeringAngle(gsr);

  opendlv::proxy::PedalPositionRequest ppr;
  ppr.position(-0.2f);
  stm.setPedalPosition(ppr);

  float prevLateralSpeed{0.0f};
  float prevLongitudinalSpeed{0.0f};
  float prevYawRate{0.0f};
  float lateralSpeed{0.0f};
  float longitudinalSpeed{0.0f};
  float yawRate{0.0f};
  for (uint16_t i{0}; i < 100; i++) {
    opendlv::sim::KinematicState ks = stm.step(0.01);
    prevLateralSpeed = lateralSpeed;
    prevLongitudinalSpeed = longitudinalSpeed;
    prevYawRate = yawRate;
    lateralSpeed = ks.vy();
    longitudinalSpeed = ks.vx();
    yawRate = ks.yawRate();
    REQUIRE(lateralSpeed <= 0.0f);
    REQUIRE(yawRate <= 0.0f);
  }
  REQUIRE(lateralSpeed == Approx(prevLateralSpeed).epsilon(0.01));
  REQUIRE(longitudinalSpeed == Approx(prevLongitudinalSpeed).epsilon(0.01));
  REQUIRE(yawRate == Approx(prevYawRate).epsilon(0.01));
}

TEST_CASE("Test single track model, steer to the left with slowly increasing amplitude.") {
  SingleTrackModel stm;

  opendlv::proxy::GroundSteeringRequest gsr;
  gsr.groundSteering(0.0f);
  stm.setGroundSteeringAngle(gsr);

  opendlv::proxy::PedalPositionRequest ppr;
  ppr.position(0.1f);
  stm.setPedalPosition(ppr);

  for (uint16_t i{0}; i < 100; i++) {
    opendlv::sim::KinematicState ks = stm.step(0.01);
    float lateralSpeed = ks.vy();
    float yawRate = ks.yawRate();
    REQUIRE(lateralSpeed >= 0.0f);
    REQUIRE(lateralSpeed < 10.0f);
    REQUIRE(yawRate >= 0.0f);
    REQUIRE(yawRate < 10.0f);
  
    if (gsr.groundSteering() < 0.5f) {
      gsr.groundSteering(gsr.groundSteering() + 0.01f);
      stm.setGroundSteeringAngle(gsr);
    }
  }
}

TEST_CASE("Test single track model, steer to the left with slowly increasing speed.") {
  SingleTrackModel stm;

  opendlv::proxy::GroundSteeringRequest gsr;
  gsr.groundSteering(0.2f);
  stm.setGroundSteeringAngle(gsr);

  opendlv::proxy::PedalPositionRequest ppr;
  ppr.position(0.0f);
  stm.setPedalPosition(ppr);

  for (uint16_t i{0}; i < 100; i++) {
    opendlv::sim::KinematicState ks = stm.step(0.01);
    float lateralSpeed = ks.vy();
    float yawRate = ks.yawRate();

    REQUIRE(lateralSpeed >= 0.0f);
    REQUIRE(lateralSpeed < 10.0f);
    REQUIRE(yawRate >= 0.0f);
    REQUIRE(yawRate < 10.0f);
  
    ppr.position(ppr.position() + 0.01f);
    stm.setPedalPosition(ppr);
  }
}
