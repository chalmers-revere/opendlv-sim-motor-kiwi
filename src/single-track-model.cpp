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

#include <cmath>
#include <iostream>

#include "single-track-model.hpp"

SingleTrackModel::SingleTrackModel() noexcept:
  m_groundSteeringAngleMutex{},
  m_pedalPositionMutex{},
  m_longitudinalSpeed{0.0f},
  m_lateralSpeed{0.0f},
  m_yawRate{0.0f},
  m_groundSteeringAngle{0.0f},
  m_pedalPosition{0.0f}
{
}

void SingleTrackModel::setGroundSteeringAngle(opendlv::proxy::GroundSteeringRequest const &groundSteeringAngle) noexcept
{
  std::lock_guard<std::mutex> lock(m_groundSteeringAngleMutex);
  m_groundSteeringAngle = groundSteeringAngle.groundSteering();
}

void SingleTrackModel::setPedalPosition(opendlv::proxy::PedalPositionRequest const &pedalPosition) noexcept
{
  std::lock_guard<std::mutex> lock(m_pedalPositionMutex);
  m_pedalPosition = pedalPosition.position();
}

opendlv::sim::KinematicState SingleTrackModel::step(double dt) noexcept
{
  double const pedalForceGainForward{26.0};
  double const pedalForceGainReverse{5.0};

  double const resistanceGain{4.0};

  double const mass{1.113};
  double const momentOfInertiaZ{0.01};
  double const wheelBase{0.224};
  double const frontToCog{0.12};
  double const rearToCog{wheelBase - frontToCog};
  double const corneringStiffnessFront{15.0};
  double const corneringStiffnessRear{15.0};
  
  float groundSteeringAngleCopy;
  float pedalPositionCopy;
  {
    std::lock_guard<std::mutex> lock1(m_groundSteeringAngleMutex);
    std::lock_guard<std::mutex> lock2(m_pedalPositionMutex);
    groundSteeringAngleCopy = m_groundSteeringAngle;
    pedalPositionCopy = m_pedalPosition;
  }

  groundSteeringAngleCopy = (groundSteeringAngleCopy / 0.290888f) * 0.6f;

  pedalPositionCopy = (pedalPositionCopy < 0.14f) ? pedalPositionCopy : 0.14f;
  pedalPositionCopy = (pedalPositionCopy > -0.9f) ? pedalPositionCopy : -0.9f;
  groundSteeringAngleCopy = (groundSteeringAngleCopy < 0.6f) 
    ? groundSteeringAngleCopy : 0.6f;
  groundSteeringAngleCopy = (groundSteeringAngleCopy > -0.6f) 
    ? groundSteeringAngleCopy : -0.6f;

  double propulsionForce;
  double resistanceForce;
  if (pedalPositionCopy > 0.0) {
    propulsionForce = pedalPositionCopy * pedalForceGainForward;
    resistanceForce = m_longitudinalSpeed * resistanceGain;
  } else {
    propulsionForce = pedalPositionCopy * pedalForceGainReverse;
    resistanceForce = m_longitudinalSpeed * resistanceGain;
  }
    
  double const longitudinalSpeedDot = 
    (propulsionForce - resistanceForce) / mass;
    
  m_longitudinalSpeed += longitudinalSpeedDot * dt;

  if (std::abs(m_longitudinalSpeed) > 0.1f) {
    double const slipAngleFront = groundSteeringAngleCopy 
      - (m_lateralSpeed + frontToCog * m_yawRate) 
      / std::abs(m_longitudinalSpeed);
    double const slipAngleRear = (rearToCog * m_yawRate - m_lateralSpeed) 
      / std::abs(m_longitudinalSpeed);

    double const lateralSpeedDot = (corneringStiffnessFront * slipAngleFront 
        + corneringStiffnessRear * slipAngleRear)
      / mass - m_longitudinalSpeed * m_yawRate;
    
    double const yawRateDot = (frontToCog * corneringStiffnessFront * slipAngleFront 
        - rearToCog * corneringStiffnessRear * slipAngleRear)
      / momentOfInertiaZ;
    
    m_lateralSpeed += lateralSpeedDot * dt;
    m_yawRate += yawRateDot * dt;
  } else {
    m_lateralSpeed = 0.0f;
    m_yawRate = 0.0f;
  }

  opendlv::sim::KinematicState kinematicState;
  kinematicState.vx(static_cast<float>(m_longitudinalSpeed));
  kinematicState.vy(static_cast<float>(m_lateralSpeed));
  kinematicState.yawRate(static_cast<float>(m_yawRate));

  return kinematicState;
}
