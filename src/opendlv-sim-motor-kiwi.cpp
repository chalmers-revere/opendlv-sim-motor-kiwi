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

#include "cluon-complete.hpp"
#include "opendlv-standard-message-set.hpp"
#include "single-track-model.hpp"

int32_t main(int32_t argc, char **argv) {
  int32_t retCode{0};
  auto commandlineArguments = cluon::getCommandlineArguments(argc, argv);
  if (0 == commandlineArguments.count("cid") 
      || 0 == commandlineArguments.count("freq") 
      || 0 == commandlineArguments.count("frame-id")) {
    std::cerr << argv[0] << " is a dynamics model for the Chalmers Kiwi "
      << "platform." << std::endl
      << "Usage:   " << argv[0] << " --frame-id=<ID of frame (used for "
      << "integration)> --freq=<Model frequency> --cid=<od4 session> "
      << "[--timemod=<Time scale modifier for simulation speed. Default: 1.0>] "
      << "[--input-id=<Sender stamp of control signals. Default: 0>]" 
      << std::endl
      << "Example: " << argv[0] << " --frame-id=0 --freq=100 --cid=111" 
      << std::endl;
    retCode = 1;
  } else {
    bool const VERBOSE{commandlineArguments.count("verbose") != 0};
    uint16_t const CID = std::stoi(commandlineArguments["cid"]);
    uint32_t const FRAME_ID = std::stoi(commandlineArguments["frame-id"]);
    uint32_t const INPUT_ID = (commandlineArguments.count("input-id") != 0) ?
      std::stoi(commandlineArguments["input-id"]) : 0;
    float const TIMEMOD{(commandlineArguments["timemod"].size() != 0) 
      ? static_cast<float>(std::stof(commandlineArguments["timemod"])) : 1.0f};
    float const FREQ = std::stof(commandlineArguments["freq"]);
    double const DT = 1.0 / FREQ;
    
    SingleTrackModel singleTrackModel;

    auto onGroundSteeringRequest{[&singleTrackModel, &INPUT_ID](
        cluon::data::Envelope &&envelope)
      {
        uint32_t const senderStamp = envelope.senderStamp();
        if (senderStamp == INPUT_ID) {
          auto groundSteeringAngleRequest = 
            cluon::extractMessage<opendlv::proxy::GroundSteeringRequest>(
                std::move(envelope));
          singleTrackModel.setGroundSteeringAngle(groundSteeringAngleRequest);
        }
      }};
    auto onPedalPositionRequest{[&singleTrackModel, &INPUT_ID](
        cluon::data::Envelope &&envelope)
      {
        uint32_t const senderStamp = envelope.senderStamp();
        if (senderStamp == INPUT_ID) {
          auto pedalPositionRequest = 
            cluon::extractMessage<opendlv::proxy::PedalPositionRequest>(
                std::move(envelope));
          singleTrackModel.setPedalPosition(pedalPositionRequest);
        }
      }};

    cluon::OD4Session od4{CID};
    od4.dataTrigger(opendlv::proxy::GroundSteeringRequest::ID(),
        onGroundSteeringRequest);
    od4.dataTrigger(opendlv::proxy::PedalPositionRequest::ID(),
        onPedalPositionRequest);

    auto atFrequency{[&FRAME_ID, &VERBOSE, &DT, &singleTrackModel, &od4]() 
      -> bool
      {
        opendlv::sim::KinematicState kinematicState = singleTrackModel.step(DT);

        cluon::data::TimeStamp sampleTime = cluon::time::now();
        od4.send(kinematicState, sampleTime, FRAME_ID);
        if (VERBOSE) {
          std::cout << "Kinematic state with id " << FRAME_ID
            << " is at velocity [vx=" << kinematicState.vx() 
            << ", vy=" << kinematicState.vy() << ", vz=" << kinematicState.vz() 
            << "] with the rotation rate [rollRate=" 
            << kinematicState.rollRate() << ", pitchRate=" 
            << kinematicState.pitchRate() << ", yawRate=" 
            << kinematicState.yawRate() << "]." << std::endl;
        }
        return true;
      }};

    od4.timeTrigger(TIMEMOD * FREQ, atFrequency);
  }
  return retCode;
}
