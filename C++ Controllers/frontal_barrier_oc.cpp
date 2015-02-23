#include "frontal_barrier_oc.h"
#include <argos3/core/utility/logging/argos_log.h>
#include <argos3/core/utility/math/angles.h>
#include <argos3/core/utility/math/vector3.h>
#include <iostream>

using namespace argos;

/****************************************/
/****************************************/

static const UInt8 BLOCKING_SYSTEM_MAX_COUNT = 20;
static const CColor DEFAULT_HUMAN_LEFT_COLOR = CColor::CYAN;
static const CColor DEFAULT_HUMAN_RIGHT_COLOR = CColor::MAGENTA;
static const CColor DEFAULT_AGENT_GOOD_COLOR = CColor::GREEN;
static const CColor DEFAULT_AGENT_BAD_COLOR = CColor::RED;

/****************************************/
/****************************************/

CEpuckFrontalBarrierOC::CEpuckFrontalBarrierOC() :
    m_fDefaultWheelsSpeed(0),
    m_fHumanPotentialGain(0),
    m_fHumanPotentialDistance(0),
    m_cHumanLeftColor(DEFAULT_HUMAN_LEFT_COLOR),
    m_cHumanRightColor(DEFAULT_HUMAN_RIGHT_COLOR),
    m_fAgentPotentialGain(0),
    m_fAgentPotentialDistance(0),
    m_cAgentGoodColor(DEFAULT_AGENT_GOOD_COLOR),
    m_cAgentBadColor(DEFAULT_AGENT_BAD_COLOR),
    m_fGravityPotentialGain(0),
    m_pcWheelsActuator(NULL),
    m_pcProximitySensor(NULL),
    m_pcRGBLED(NULL),
    m_unBSDirection(0),
    m_unBSCount(0) {
}

/****************************************/
/****************************************/

void CEpuckFrontalBarrierOC::ParseParams(TConfigurationNode& t_node) {
    CVector3 cHumanLeftColor(DEFAULT_HUMAN_LEFT_COLOR.GetRed(), DEFAULT_HUMAN_LEFT_COLOR.GetGreen(), DEFAULT_HUMAN_LEFT_COLOR.GetBlue());
    CVector3 cHumanRightColor(DEFAULT_HUMAN_RIGHT_COLOR.GetRed(), DEFAULT_HUMAN_RIGHT_COLOR.GetGreen(), DEFAULT_HUMAN_RIGHT_COLOR.GetBlue());
    CVector3 cAgentGoodColor(DEFAULT_AGENT_GOOD_COLOR.GetRed(), DEFAULT_AGENT_GOOD_COLOR.GetGreen(), DEFAULT_AGENT_GOOD_COLOR.GetBlue());
    CVector3 cAgentBadColor(DEFAULT_AGENT_BAD_COLOR.GetRed(), DEFAULT_AGENT_BAD_COLOR.GetGreen(), DEFAULT_AGENT_BAD_COLOR.GetBlue());
    try {
        /* Default wheel speed */
        GetNodeAttributeOrDefault(t_node, "defaultSpeed", m_fDefaultWheelsSpeed, m_fDefaultWheelsSpeed);
        /* Human potential gain */
        GetNodeAttributeOrDefault(t_node, "humanPotentialGain", m_fHumanPotentialGain, m_fHumanPotentialGain);
        /* Human potential distance */
        GetNodeAttributeOrDefault(t_node, "humanPotentialDistance", m_fHumanPotentialDistance, m_fHumanPotentialDistance);
        /* Human potential gain */
        GetNodeAttributeOrDefault(t_node, "agentPotentialGain", m_fAgentPotentialGain, m_fAgentPotentialGain);
        /* Human potential distance */
        GetNodeAttributeOrDefault(t_node, "agentPotentialDistance", m_fAgentPotentialDistance, m_fAgentPotentialDistance);
        /* Gravity potential gain */
        GetNodeAttributeOrDefault(t_node, "gravityPotentialGain", m_fGravityPotentialGain, m_fGravityPotentialGain);

        /* Human agent left color */
        GetNodeAttributeOrDefault(t_node, "humanLeftColor", cHumanLeftColor, cHumanLeftColor);
        m_cHumanLeftColor.Set((UInt8) cHumanLeftColor.GetX(), (UInt8) cHumanLeftColor.GetY(), (UInt8) cHumanLeftColor.GetZ());

        /* Human agent right color */
        GetNodeAttributeOrDefault(t_node, "humanRightColor", cHumanRightColor, cHumanRightColor);
        m_cHumanRightColor.Set((UInt8) cHumanRightColor.GetX(), (UInt8) cHumanRightColor.GetY(), (UInt8) cHumanRightColor.GetZ());

        /* Agent good color */
        GetNodeAttributeOrDefault(t_node, "agentGoodColor", cAgentGoodColor, cAgentGoodColor);
        m_cAgentGoodColor.Set((UInt8) cAgentGoodColor.GetX(), (UInt8) cAgentGoodColor.GetY(), (UInt8) cAgentGoodColor.GetZ());

        /* Agent bad color */
        GetNodeAttributeOrDefault(t_node, "agentBadColor", cAgentBadColor, cAgentBadColor);
        m_cAgentBadColor.Set((UInt8) cAgentBadColor.GetX(), (UInt8) cAgentBadColor.GetY(), (UInt8) cAgentBadColor.GetZ());
    } catch (CARGoSException& ex) {
        THROW_ARGOSEXCEPTION_NESTED("Error parsing <params>", ex);
    }
}

/****************************************/
/****************************************/

void CEpuckFrontalBarrierOC::Init(TConfigurationNode& t_node) {
	/* parse the xml tree <params> */
    ParseParams(t_node);

    try {
        m_pcWheelsActuator = GetActuator<CCI_EPuckWheelsActuator>("epuck_wheels");
    } catch (CARGoSException ex) {}
    try {
        m_pcProximitySensor = GetSensor<CCI_EPuckProximitySensor>("epuck_proximity");
    } catch (CARGoSException ex) {}
    try {
        m_pcOmnidirectionalCameraSensor = GetSensor<CCI_EPuckOmnidirectionalCameraSensor>("epuck_omnidirectional_camera");
        // Mandatory to enable the sensor to get data.
        if (m_pcOmnidirectionalCameraSensor != NULL)
            m_pcOmnidirectionalCameraSensor->Enable();
    } catch (CARGoSException ex) {
        LOG << "Pb with ommidirectional camera initialization." << std::endl;
    }
    try {
        m_pcRGBLED = GetActuator<CCI_EPuckRGBLEDsActuator>("epuck_rgb_leds");

        if (m_pcRGBLED != NULL) {
            m_pcRGBLED->SetColors(m_cAgentGoodColor);
        }
    } catch (CARGoSException ex) {}
}

/****************************************/
/****************************************/

void CEpuckFrontalBarrierOC::Reset() {

    if (m_pcOmnidirectionalCameraSensor != NULL) {
        m_pcOmnidirectionalCameraSensor->Enable();
    }
    if (m_pcRGBLED != NULL) {
        m_pcRGBLED->SetColors(m_cAgentGoodColor);
    }
}

/****************************************/
/****************************************/

void CEpuckFrontalBarrierOC::ControlStep() {
    /*
    // Simple direction method:
    CVector2 cResultVector;
    ComputeDirection(cResultVector);
    CRadians cDirectionAngle = cResultVector.Angle();


    if (cDirectionAngle > (CRadians::PI / 36.0f) && cDirectionAngle < CRadians::PI) { // Left
        if (m_pcWheelsActuator != NULL) {
            m_pcWheelsActuator->SetLinearVelocity(-m_fDefaultWheelsSpeed, m_fDefaultWheelsSpeed);
        }
    } else if (cDirectionAngle < (-CRadians::PI / 36.0f)) {                           // Right
        if (m_pcWheelsActuator != NULL) {
            m_pcWheelsActuator->SetLinearVelocity(m_fDefaultWheelsSpeed, -m_fDefaultWheelsSpeed);
        }
    } else {
        if (m_pcWheelsActuator != NULL) {
            m_pcWheelsActuator->SetLinearVelocity(m_fDefaultWheelsSpeed, m_fDefaultWheelsSpeed);
        }
    }*/

    // Advanced direction method:
    if (m_unBSCount < BLOCKING_SYSTEM_MAX_COUNT) {
        NormalMode();
    } else {
        BlockedMode();
    }

/*    if (m_pcRABSensor != NULL) {
        const CCI_EPuckRangeAndBearingSensor::TPackets& packets = m_pcRABSensor->GetPackets();

        for (size_t i = 0; i < packets.size(); ++i) {
            if (packets[i]->Data[0] != 0)
                LOG << packets[i]->Data[0] << " " << packets[i]->Range << " "  << packets[i]->Bearing << std::endl;
            else
                LOG << "Error " << packets[i]->Data[0] << std::endl;
        }
    }*/
}

/****************************************/
/****************************************/

/* Called if the robot is in normal mode, not blocked. */
void CEpuckFrontalBarrierOC::NormalMode() {
    CVector2 cObstacleVector, cResultVector;
    Real fMaxValue = 0;

    const CCI_EPuckProximitySensor::TReadings& tProximityReadings = m_pcProximitySensor->GetReadings();
    UInt8 unMaxIndex = tProximityReadings.size() - 1;

    for(size_t i = 0; i < 2; ++i) {
        CVector2 cLeftVector(tProximityReadings[i].Value, tProximityReadings[i].Angle);
        CVector2 cRightVector(tProximityReadings[unMaxIndex-i].Value, tProximityReadings[unMaxIndex-i].Angle);

        cObstacleVector += cLeftVector;
        cObstacleVector += cRightVector;

        fMaxValue = (tProximityReadings[i].Value > fMaxValue) ? tProximityReadings[i].Value : fMaxValue;
        fMaxValue = (tProximityReadings[unMaxIndex-i].Value > fMaxValue) ? tProximityReadings[unMaxIndex-i].Value : fMaxValue;
    }

    if (fMaxValue > 0.15) { // Obstacle to deal with
        cResultVector -= cObstacleVector;
    } else {                // No obstacle
        ComputeDirection(cResultVector);

        m_unBSCount = 0; // No obstacle detected, so the blocked mode counter is set to 0.
    }

    // We have the direction and speed. Let's apply it on the wheels:
    CRadians cDirectionAngle = cResultVector.Angle();
    Real fSpeed = cResultVector.Length();
    fSpeed = (fSpeed > 10.0) ? 10 : fSpeed; // E-pucks cannot go faster than 10 cm/s.
    UInt8 unNewDirection;

    if (cDirectionAngle > CRadians::ZERO && cDirectionAngle < CRadians::PI) { // Left
        unNewDirection = 1;
        if (m_pcWheelsActuator != NULL) {
            // First parameter = speed * cos(angle) = x
            m_pcWheelsActuator->SetLinearVelocity(fSpeed * Cos(cDirectionAngle), fSpeed);
        }
    } else {                                                                  // Right
        unNewDirection = -1;
        if (m_pcWheelsActuator != NULL) {
            // Second parameter = speed * cos(angle) = x
            m_pcWheelsActuator->SetLinearVelocity(fSpeed, fSpeed * Cos(cDirectionAngle));
        }
    }

    if (m_unBSDirection != unNewDirection) {
        m_unBSCount++;
        m_unBSDirection = unNewDirection;
    }
}

/****************************************/
/****************************************/

/* Adds the interesting directions to the final direction vector. */
void CEpuckFrontalBarrierOC::ComputeDirection(CVector2& c_result_vector) const {
    // Add the potentials:
    c_result_vector += HumanPotential(); // Certain distance to human
    c_result_vector += GravityPotential(); // Move in front of human
    c_result_vector += AgentRepulsionPotential(); // Repulsion among agents
    c_result_vector += DefaultPotential(); // If no human found, makes the agents move
}

/****************************************/
/****************************************/

const CVector2 CEpuckFrontalBarrierOC::HumanPotential() const {
    CVector2 vector;

    if (m_pcOmnidirectionalCameraSensor != NULL) {
        const CCI_EPuckOmnidirectionalCameraSensor::SReadings& readings = m_pcOmnidirectionalCameraSensor->GetReadings();
        const CCI_EPuckOmnidirectionalCameraSensor::TBlobList& blobs = readings.BlobList;

        if (blobs.size() > 0) {
            Real fMinimum = -100.0; // Arbitrary negative number
            SInt16 unMinimumIndex = -1;

            for (size_t i = 0; i < blobs.size(); ++i) {
                if (IsHuman(blobs[i]->Color)
                    && (blobs[i]->Distance < fMinimum || fMinimum < 0)) {

                    fMinimum = blobs[i]->Distance;
                    unMinimumIndex = i;
                }
            }

            if (unMinimumIndex != -1) {
                Real fLennardJonesValue = LennardJones(blobs[unMinimumIndex]->Distance, m_fHumanPotentialGain, m_fHumanPotentialDistance);
                vector.FromPolarCoordinates(fLennardJonesValue, blobs[unMinimumIndex]->Angle);
            }
        }
    }

    return vector;
}

/****************************************/
/****************************************/

const CVector2 CEpuckFrontalBarrierOC::GravityPotential() const {
    CVector2 cVector;

    if (m_pcOmnidirectionalCameraSensor != NULL) {
        const CCI_EPuckOmnidirectionalCameraSensor::SReadings& readings = m_pcOmnidirectionalCameraSensor->GetReadings();
        const CCI_EPuckOmnidirectionalCameraSensor::TBlobList& blobs = readings.BlobList;

        if (blobs.size() > 0) {
            Real fMinimum = -100.0; // Arbitrary negative number
            SInt16 unMinimumIndex = -1;

            for (size_t i = 0; i < blobs.size(); ++i) {
                if (IsHuman(blobs[i]->Color)
                    && (blobs[i]->Distance < fMinimum || fMinimum < 0)) {

                    fMinimum = blobs[i]->Distance;
                    unMinimumIndex = i;
                }
            }

            if (unMinimumIndex != -1) {
                CRadians cRotationModification = 
                    (IsSameColor(blobs[unMinimumIndex]->Color, m_cHumanLeftColor)) ? 
                    CRadians::PI_OVER_TWO : 
                    -CRadians::PI_OVER_TWO;
                cVector.FromPolarCoordinates(m_fGravityPotentialGain, blobs[unMinimumIndex]->Angle + cRotationModification);
            }
        }
    }

    return cVector;
}

/****************************************/
/****************************************/

const CVector2 CEpuckFrontalBarrierOC::AgentRepulsionPotential() const {
    CVector2 cVector;

    if (m_pcOmnidirectionalCameraSensor != NULL) {
        const CCI_EPuckOmnidirectionalCameraSensor::SReadings& readings = m_pcOmnidirectionalCameraSensor->GetReadings();
        const CCI_EPuckOmnidirectionalCameraSensor::TBlobList& blobs = readings.BlobList;

        for (size_t i = 0; i < blobs.size(); ++i) {
            if (!IsHuman(blobs[i]->Color) && blobs[i]->Distance < m_fAgentPotentialDistance) {
                Real fLennardJonesValue = LennardJones(blobs[i]->Distance, m_fAgentPotentialGain, m_fAgentPotentialDistance);
                CVector2 cAgentLennardJones(fLennardJonesValue, blobs[i]->Angle);
                cVector += cAgentLennardJones;
            }
        }
    }

    return cVector;
}

/****************************************/
/****************************************/

const CVector2 CEpuckFrontalBarrierOC::DefaultPotential() const {
    CVector2 cVector;
    bool bHumanFound = false;

    if (m_pcOmnidirectionalCameraSensor != NULL) {
        const CCI_EPuckOmnidirectionalCameraSensor::SReadings& readings = m_pcOmnidirectionalCameraSensor->GetReadings();
        const CCI_EPuckOmnidirectionalCameraSensor::TBlobList& blobs = readings.BlobList;

        for (size_t i = 0; i < blobs.size(); ++i) {
            if (IsHuman(blobs[i]->Color)) {
                bHumanFound = true;
                break;
            }
        }

        if (!bHumanFound) {
            cVector.SetX(5.0);
        }
    }
    return cVector;
}

/****************************************/
/****************************************/

/* Called if the robot is in blocked mode. 
 * It happens if the robot is changing directions a lot between objects.
 */
void CEpuckFrontalBarrierOC::BlockedMode() {
    if (m_pcProximitySensor != NULL) {
        Real fLeftValue = m_pcProximitySensor->GetReading(0).Value;
        Real fRightValue = m_pcProximitySensor->GetReading(7).Value;
        Real fMax = (fLeftValue > fRightValue) ? fLeftValue : fRightValue;

        if (fMax > 0.2 && m_pcWheelsActuator != NULL) { // Something ahead
            // Turn right on itself
            m_pcWheelsActuator->SetLinearVelocity(m_fDefaultWheelsSpeed, -m_fDefaultWheelsSpeed);
        } else {                                        // Nothing ahead
            m_unBSCount = 0;
            if (m_pcWheelsActuator != NULL) {
                // One leap forward
                m_pcWheelsActuator->SetLinearVelocity(m_fDefaultWheelsSpeed, m_fDefaultWheelsSpeed);
            }
        }
    }
}

/****************************************/
/****************************************/

inline Real CEpuckFrontalBarrierOC::LennardJones(Real f_x, Real f_gain, Real f_distance) const {
    Real fRatio = f_distance / f_x;
    fRatio *= fRatio;
    return -4 * f_gain / f_x * ( fRatio * fRatio - fRatio );
}

/****************************************/
/****************************************/

inline bool CEpuckFrontalBarrierOC::IsHuman(const CColor& c_color) const {
    return (IsSameColor(c_color, m_cHumanLeftColor) || IsSameColor(c_color, m_cHumanRightColor));
}

/****************************************/
/****************************************/

inline bool CEpuckFrontalBarrierOC::IsSameColor(const CColor& c_color_1, const CColor& c_color_2) const {
    CVector3 cColor1(c_color_1.GetRed(), c_color_1.GetGreen(), c_color_1.GetBlue());
    CVector3 cColor2(c_color_2.GetRed(), c_color_2.GetGreen(), c_color_2.GetBlue());

    return Distance(cColor1, cColor2) < 32;
}

/****************************************/
/****************************************/

REGISTER_CONTROLLER(CEpuckFrontalBarrierOC, "e-puck_frontal_barrier_oc_controller");
