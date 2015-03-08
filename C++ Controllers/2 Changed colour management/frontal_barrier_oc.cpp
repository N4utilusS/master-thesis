#include "frontal_barrier_oc.h"
#include <argos3/core/utility/logging/argos_log.h>
#include <argos3/core/utility/math/angles.h>
#include <argos3/core/utility/math/vector3.h>

using namespace argos;

/****************************************/
/****************************************/

static const UInt8 BLOCKING_SYSTEM_MAX_COUNT = 20;
static const std::string DEFAULT_HUMAN_LEFT_COLOR = "cyan";
static const std::string DEFAULT_HUMAN_RIGHT_COLOR = "magenta";
static const std::string DEFAULT_AGENT_GOOD_COLOR = "green";
static const std::string DEFAULT_AGENT_BAD_COLOR = "red";

/****************************************/
/****************************************/

CEpuckFrontalBarrierOC::CEpuckFrontalBarrierOC() :
    m_fDefaultWheelsSpeed(0),
    m_fHumanPotentialGain(0),
    m_fHumanPotentialDistance(0),
    m_cHumanLeftColor(CColor::BLACK),
    m_cHumanRightColor(CColor::BLACK),
    m_fAgentPotentialGain(0),
    m_fAgentPotentialDistance(0),
    m_cAgentGoodColor(CColor::BLACK),
    m_cAgentBadColor(CColor::BLACK),
    m_fGravityPotentialGain(0),
    m_pcWheelsActuator(NULL),
    m_pcProximitySensor(NULL),
    m_pcRGBLED(NULL),
    m_pcGroundSensor(NULL),
    m_unBSDirection(0),
    m_unBSCount(0),
    m_fHumanPotentialModifiedDistance(0),
    m_unColorCountdownCounter(0) {
}

/****************************************/
/****************************************/

void CEpuckFrontalBarrierOC::ParseParams(TConfigurationNode& t_node) {
    std::string strHumanLeftColor, strHumanRightColor, strAgentGoodColor, strAgentBadColor;

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
        GetNodeAttributeOrDefault(t_node, "humanLeftColor", strHumanLeftColor, DEFAULT_HUMAN_LEFT_COLOR);
        m_cHumanLeftColor.Set(strHumanLeftColor);

        /* Human agent right color */
        GetNodeAttributeOrDefault(t_node, "humanRightColor", strHumanRightColor, DEFAULT_HUMAN_RIGHT_COLOR);
        m_cHumanRightColor.Set(strHumanRightColor);

        /* Agent good color */
        GetNodeAttributeOrDefault(t_node, "agentGoodColor", strAgentGoodColor, DEFAULT_AGENT_GOOD_COLOR);
        m_cAgentGoodColor.Set(strAgentGoodColor);

        /* Agent bad color */
        GetNodeAttributeOrDefault(t_node, "agentBadColor", strAgentBadColor, DEFAULT_AGENT_BAD_COLOR);
        m_cAgentBadColor.Set(strAgentBadColor);
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
    try {
        m_pcGroundSensor = GetSensor<CCI_EPuckGroundSensor>("epuck_ground");
        if (m_pcGroundSensor == NULL)
            LOG << "Ground sensor pointer is null in init." << std::endl;
    } catch (CARGoSException ex) {
        LOG << "Pb with ground sensor initialization." << std::endl;
    }
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
    /*
    if (m_pcOmnidirectionalCameraSensor != NULL) {
        const CCI_EPuckOmnidirectionalCameraSensor::SReadings& readings = m_pcOmnidirectionalCameraSensor->GetReadings();
        const CCI_EPuckOmnidirectionalCameraSensor::TBlobList& blobs = readings.BlobList;
        
        for (size_t i = 0; i < blobs.size(); ++i)
            if (blobs[i]->Distance > 9)
                LOG << "(Color = " << blobs[i]->Color << "," << "Angle = " << ToDegrees(blobs[i]->Angle) << ","
                           << "Distance = " << blobs[i]->Distance<< ", Area = " << blobs[i]->Area << ")" << std::endl;
    }
    LOG << std::endl;
    */

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

    // ------------ Different cases ---------------------------------------------------
    if (fMaxValue > 0.15) { // Obstacle to deal with
        cResultVector -= cObstacleVector;
        //LOG << "OBSTACLE " << fMaxValue << std::endl;

    } else if (HumanFound()) {                // No obstacle
        ComputeDirection(cResultVector);
        m_unBSCount = 0; // No obstacle detected, so the blocked mode counter is set to 0.
        if (m_pcRGBLED != NULL) {
            m_pcRGBLED->SetColors(GetAgentSituationColor());
        }
        //LOG << "AGENT" << std::endl;

    } else {
        cResultVector += DefaultPotential(); // If no human found, makes the agents move
        m_unBSCount = 0; // No obstacle detected, so the blocked mode counter is set to 0.
        if (m_pcRGBLED != NULL) {
            m_pcRGBLED->SetColors(CColor::BLACK);
        }
        //LOG << "NO HUMAN" << std::endl;

    }
    // --------------------------------------------------------------------------------

    // We have the direction and speed. Let's apply it on the wheels:
    CRadians cDirectionAngle = cResultVector.Angle();
    Real fSpeed = cResultVector.Length();
    fSpeed = (fSpeed > 10.0) ? 10 : fSpeed; // E-pucks cannot go faster than 10 cm/s.
    UInt8 unNewDirection(0);

    //LOG << cResultVector << std::endl;
    
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
/*
    if (m_pcWheelsActuator != NULL) {
        if (cDirectionAngle > (CRadians::PI / 36.0f) && cDirectionAngle < CRadians::PI) { // Left
            if (m_pcWheelsActuator != NULL) {
                m_pcWheelsActuator->SetLinearVelocity(-fSpeed, fSpeed);
                unNewDirection = 1;
            }
        } else if (cDirectionAngle < (-CRadians::PI / 36.0f)) {                           // Right
            if (m_pcWheelsActuator != NULL) {
                m_pcWheelsActuator->SetLinearVelocity(fSpeed, -fSpeed);
                unNewDirection = -1;
            }
        } else {
            if (m_pcWheelsActuator != NULL) {
                m_pcWheelsActuator->SetLinearVelocity(fSpeed, fSpeed);
            }
        }
    }*/

    if (m_unBSDirection != unNewDirection) {
        m_unBSCount++;
        m_unBSDirection = unNewDirection;
    }
}

/****************************************/
/****************************************/

/* Adds the interesting directions to the final direction vector. */
void CEpuckFrontalBarrierOC::ComputeDirection(CVector2& c_result_vector) {
    // Add the potentials:
    c_result_vector += HumanPotential(); // Certain distance to human
    c_result_vector += GravityPotential(); // Move in front of human
    c_result_vector += AgentRepulsionPotential(); // Repulsion among agents
}

/****************************************/
/****************************************/

const CVector2 CEpuckFrontalBarrierOC::HumanPotential() {
    CVector2 vector;

    if (m_pcOmnidirectionalCameraSensor != NULL) {
        const CCI_EPuckOmnidirectionalCameraSensor::SReadings& readings = m_pcOmnidirectionalCameraSensor->GetReadings();
        const CCI_EPuckOmnidirectionalCameraSensor::TBlobList& blobs = readings.BlobList;

        if (blobs.size() > 0) {
            Real fMinimum = -100.0; // Arbitrary negative number
            SInt16 unMinimumIndex = -1;

            for (size_t i = 0; i < blobs.size(); ++i) {
                if (blobs[i]->Distance > 9 && IsHuman(blobs[i]->Color)
                    && (blobs[i]->Distance < fMinimum || fMinimum < 0)) {

                    fMinimum = blobs[i]->Distance;
                    unMinimumIndex = i;
                }
            }

            if (unMinimumIndex != -1) {
                // 2 CASES: IN DANGER ZONE OR NOT --------------------------
                // 1 - IN DANGER ZONE:
                if (IsInDanger()) {
                    m_fHumanPotentialModifiedDistance = (m_fHumanPotentialModifiedDistance > 15) ? 
                    m_fHumanPotentialModifiedDistance-0.25 : 
                    m_fHumanPotentialModifiedDistance;
                // 2- NOT IN DANGER ZONE:
                } else {
                    m_fHumanPotentialModifiedDistance = (m_fHumanPotentialModifiedDistance < m_fHumanPotentialDistance) ? 
                    m_fHumanPotentialModifiedDistance+0.25 : 
                    m_fHumanPotentialModifiedDistance;
                }

                Real fLennardJonesValue = LennardJones(blobs[unMinimumIndex]->Distance, m_fHumanPotentialGain, m_fHumanPotentialModifiedDistance);
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
                if (blobs[i]->Distance > 9 && IsHuman(blobs[i]->Color)
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
            if (blobs[i]->Distance > 9 && !IsHuman(blobs[i]->Color) && blobs[i]->Distance < m_fAgentPotentialDistance) {
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

    if (m_pcOmnidirectionalCameraSensor != NULL) {
        const CCI_EPuckOmnidirectionalCameraSensor::SReadings& readings = m_pcOmnidirectionalCameraSensor->GetReadings();
        const CCI_EPuckOmnidirectionalCameraSensor::TBlobList& blobs = readings.BlobList;

        for (size_t i = 0; i < blobs.size(); ++i) {
            if (blobs[i]->Distance > 9) {
                Real fLennardJonesValue = LennardJones(blobs[i]->Distance, m_fAgentPotentialGain, m_fAgentPotentialDistance);
                CVector2 cAgentLennardJones(fLennardJonesValue, blobs[i]->Angle);
                cVector += cAgentLennardJones;
            }
        }
    }

    // If no other robot to go to, then go straight forward.
    if (cVector.SquareLength() == 0)
        cVector.SetX(5);
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
    
    return (c_color_1 == c_color_2);
}

/****************************************/
/****************************************/

bool CEpuckFrontalBarrierOC::HumanFound() const {
    bool bHumanFound = false;

    if (m_pcOmnidirectionalCameraSensor != NULL) {
        const CCI_EPuckOmnidirectionalCameraSensor::SReadings& readings = m_pcOmnidirectionalCameraSensor->GetReadings();
        const CCI_EPuckOmnidirectionalCameraSensor::TBlobList& blobs = readings.BlobList;

        for (size_t i = 0; i < blobs.size(); ++i) {
            if (blobs[i]->Distance > 9 && IsHuman(blobs[i]->Color)) {
                bHumanFound = true;
                break;
            }
        }
    }

    return bHumanFound;
}

/****************************************/
/****************************************/

inline const CColor CEpuckFrontalBarrierOC::GetAgentSituationColor(){

    if (IsInDanger())
        m_unColorCountdownCounter = 20;

    if (m_unColorCountdownCounter > 0) {
        m_unColorCountdownCounter--;
        return m_cAgentBadColor;
    } else {
        return m_cAgentGoodColor;
    }
}

/****************************************/
/****************************************/

inline const bool CEpuckFrontalBarrierOC::IsInDanger() const {
    const CCI_EPuckGroundSensor::SReadings& readings = m_pcGroundSensor->GetReadings();

    return (readings.Left < 0.5 || readings.Center < 0.5 || readings.Right < 0.5);
}

/****************************************/
/****************************************/

REGISTER_CONTROLLER(CEpuckFrontalBarrierOC, "e-puck_frontal_barrier_oc_controller");
