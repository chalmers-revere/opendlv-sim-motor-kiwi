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

#include "single-track-model.hpp"

SingleTrackModel::SingleTrackModel() noexcept:
  m_groundSteeringAngleMutex{},
  m_pedalPositionMutex{},
  m_longitudinalSpeed{0.0},
  m_lateralSpeed{0.0},
  m_yawRate{0.0},
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
  double const g{9.82};

  double const mass{1.0};
  double const momentOfInertiaZ{3.0};
  double const length{0.3};
  double const frontToCog{0.15};
  double const rearToCog{length - frontToCog};
  double const frictionCoefficient{0.1};

  double const pedalSpeedGain{0.8};

  double const magicFormulaCAlpha{1000.0};
  double const magicFormulaC{1000.0};
  double const magicFormulaE{1000.0};
  
  float groundSteeringAngleCopy;
  float pedalPositionCopy;
  {
    std::lock_guard<std::mutex> lock1(m_groundSteeringAngleMutex);
    std::lock_guard<std::mutex> lock2(m_pedalPositionMutex);
    groundSteeringAngleCopy = m_groundSteeringAngle;
    pedalPositionCopy = m_pedalPosition;
  }


  double slipAngleFront = 0.0;
  double slipAngleRear = 0.0;
  if (std::abs(m_longitudinalSpeed) > 0.001) {
    slipAngleFront = groundSteeringAngleCopy - std::atan(
        (m_lateralSpeed + frontToCog * m_yawRate) / std::abs(m_longitudinalSpeed));
    slipAngleRear = -std::atan((m_lateralSpeed - rearToCog * m_yawRate) / 
        std::abs(m_longitudinalSpeed));
  }

  double const forceFrontZ = mass * g * (frontToCog / (frontToCog + length));
  double const forceRearZ = mass * g * (length / (frontToCog + length));

  double const forceFrontY = magicFormula(slipAngleFront, forceFrontZ,
      frictionCoefficient, magicFormulaCAlpha, magicFormulaC, magicFormulaE);
  double const forceRearY = magicFormula(slipAngleRear, forceRearZ,
      frictionCoefficient, magicFormulaCAlpha, magicFormulaC, magicFormulaE);

  double const lateralSpeedDot = 
    (forceFrontY * std::cos(groundSteeringAngleCopy) + forceRearY) / mass -
    m_yawRate * m_lateralSpeed;

  double const yawRateDot = (length * forceFrontY * 
      std::cos(groundSteeringAngleCopy) - frontToCog * forceRearY) /
    momentOfInertiaZ;

  m_longitudinalSpeed = pedalPositionCopy * pedalSpeedGain;
  m_lateralSpeed += lateralSpeedDot * dt;
  m_yawRate += yawRateDot * dt;

  opendlv::sim::KinematicState kinematicState;
  kinematicState.vx(static_cast<float>(m_longitudinalSpeed));
  kinematicState.vy(static_cast<float>(m_lateralSpeed));
  kinematicState.yawRate(static_cast<float>(m_yawRate));

  return kinematicState;
}

double SingleTrackModel::magicFormula(double const &a_slipAngle, double const &a_forceZ, 
    double const &a_frictionCoefficient, double const &a_cAlpha, double const &a_c, 
    double const &a_e) const
{
  double const b = a_cAlpha / (a_c * a_frictionCoefficient * a_forceZ);
  double const forceY = a_frictionCoefficient * a_forceZ * std::sin(a_c *
     std::atan(b * a_slipAngle - a_e * (b * a_slipAngle - std::atan(b * a_slipAngle))));
  return forceY;
}
