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

TEST_CASE("Test single track model, zero input should give zero output.") {
  SingleTrackModel stm;

  opendlv::proxy::GroundSteeringRequest gsr;
  gsr.groundSteering(0.0);
  stm.setGroundSteeringAngle(gsr);

  opendlv::proxy::PedalPositionRequest ppr;
  ppr.position(0.0);
  stm.setPedalPosition(ppr);

  opendlv::sim::KinematicState ks = stm.step(0.1);
  float sum = ks.vx() + ks.vy() + ks.vz() + ks.rollRate() + ks.pitchRate() + ks.yawRate(); 
  REQUIRE(sum == Approx(0.0f));
}
