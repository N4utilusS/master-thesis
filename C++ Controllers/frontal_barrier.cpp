#include "frontal_barrier.h"
#include <argos3/core/utility/logging/argos_log.h>
#include <argos3/core/utility/math/angles.h>


using namespace argos;

/****************************************/
/****************************************/

static const UInt8 BLOCKING_SYSTEM_MAX_COUNT = 20;
static const UInt8 HUMAN_SIGNAL_MIN = 1;
static const UInt8 HUMAN_SIGNAL_MAX = 4;
static const UInt8 AGENT_SIGNAL = 0;

/****************************************/
/****************************************/

CEpuckFrontalBarrier::CEpuckFrontalBarrier() :
    m_fDefaultWheelsSpeed(0),
    m_fHumanPotentialGain(0),
    m_fHumanPotentialDistance(0),
    m_fAgentPotentialGain(0),
    m_fAgentPotentialDistance(0),
    m_fGravityPotentialGain(0),
    m_pcWheelsActuator(NULL),
    m_pcProximitySensor(NULL),
    m_pcRABActuator(NULL),
    m_pcRABSensor(NULL),
    m_unBSDirection(0),
    m_unBSCount(0) {
}

/****************************************/
/****************************************/

void CEpuckFrontalBarrier::ParseParams(TConfigurationNode& t_node) {
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
    } catch (CARGoSException& ex) {
        THROW_ARGOSEXCEPTION_NESTED("Error parsing <params>", ex);
    }
}

/****************************************/
/****************************************/

void CEpuckFrontalBarrier::Init(TConfigurationNode& t_node) {
	/* parse the xml tree <params> */
    ParseParams(t_node);

    try {
        m_pcWheelsActuator = GetActuator<CCI_EPuckWheelsActuator>("epuck_wheels");
    } catch (CARGoSException ex) {}
    try {
        m_pcProximitySensor = GetSensor<CCI_EPuckProximitySensor>("epuck_proximity");
    } catch (CARGoSException ex) {}
    try {
        m_pcRABActuator = GetActuator<CCI_EPuckRangeAndBearingActuator>("epuck_range_and_bearing");
    } catch (CARGoSException ex) {}
    try {
        m_pcRABSensor = GetSensor<CCI_EPuckRangeAndBearingSensor>("epuck_range_and_bearing");
    } catch (CARGoSException ex) {}

    if (m_pcRABActuator != NULL) {
        CCI_EPuckRangeAndBearingActuator::TData tSentData;
        tSentData[0] = AGENT_SIGNAL;
        m_pcRABActuator->SetData(tSentData);
    }
}

/****************************************/
/****************************************/

void CEpuckFrontalBarrier::Reset() {

    if (m_pcRABActuator != NULL) {
        CCI_EPuckRangeAndBearingActuator::TData tSentData;
        tSentData[0] = AGENT_SIGNAL;
        m_pcRABActuator->SetData(tSentData);
    }
}

/****************************************/
/****************************************/

void CEpuckFrontalBarrier::ControlStep() {
    CVector2 cResultVector;
    ComputeDirection(cResultVector);
    CRadians cDirectionAngle = cResultVector.Angle();

// Simple direction method:
/*    if (cDirectionAngle > (CRadians::PI / 36.0f) && cDirectionAngle < CRadians::PI) { // Left
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

    // Clear RAB:
    m_pcRABSensor->ClearPackets();
}

/****************************************/
/****************************************/

/* Called if the robot is in normal mode, not blocked. */
void CEpuckFrontalBarrier::NormalMode() {
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
void CEpuckFrontalBarrier::ComputeDirection(CVector2& c_result_vector) const {
    // Add the potentials:
    c_result_vector += HumanPotential(); // Certain distance to human
    c_result_vector += GravityPotential(); // Move in front of human
    c_result_vector += AgentRepulsionPotential(); // Repulsion among agents
    c_result_vector += DefaultPotential(); // If no human found, makes the agents move
}

/****************************************/
/****************************************/

const CVector2& CEpuckFrontalBarrier::HumanPotential() const {
    CVector2 vector;

    if (m_pcRABSensor != NULL) {
        const CCI_EPuckRangeAndBearingSensor::TPackets& packets = m_pcRABSensor->GetPackets();

        if (packets.size() > 0) {
            Real fMinimum = -100.0; // Arbitrary negative number
            SInt16 unMinimumIndex = -1;

            for (size_t i = 0; i < packets.size(); ++i) {
                if (packets[i]->Data[0] >= HUMAN_SIGNAL_MIN 
                    && packets[i]->Data[0] <= HUMAN_SIGNAL_MAX 
                    && (packets[i]->Range < fMinimum || fMinimum < 0)) {

                    fMinimum = packets[i]->Range;
                    unMinimumIndex = i;
                }
            }

            if (unMinimumIndex != -1) {
                Real fLennardJonesValue = LennardJones(packets[unMinimumIndex]->Range, m_fHumanPotentialGain, m_fHumanPotentialDistance);
                vector.FromPolarCoordinates(fLennardJonesValue, packets[unMinimumIndex]->Bearing);
            }
        }
    }

    return vector;
}

/****************************************/
/****************************************/

const CVector2& CEpuckFrontalBarrier::GravityPotential() const {
    CVector2 cVector;

    if (m_pcRABSensor != NULL) {
        const CCI_EPuckRangeAndBearingSensor::TPackets& packets = m_pcRABSensor->GetPackets();

        if (packets.size() > 0) {
            Real fMinimum = -100.0; // Arbitrary negative number
            SInt16 unMinimumIndex = -1;

            for (size_t i = 0; i < packets.size(); ++i) {
                if (packets[i]->Data[0] >= HUMAN_SIGNAL_MIN 
                    && packets[i]->Data[0] <= HUMAN_SIGNAL_MAX 
                    && (packets[i]->Range < fMinimum || fMinimum < 0)) {

                    fMinimum = packets[i]->Range;
                    unMinimumIndex = i;
                }
            }

            if (unMinimumIndex != -1) {
                CRadians cRotationModification = 
                    (packets[unMinimumIndex]->Data[0] <= (HUMAN_SIGNAL_MIN + HUMAN_SIGNAL_MAX)/2.0f) ? 
                    CRadians::PI_OVER_TWO : 
                    -CRadians::PI_OVER_TWO;
                cVector.FromPolarCoordinates(m_fGravityPotentialGain, packets[unMinimumIndex]->Bearing + cRotationModification);
            }
        }
    }

    return cVector;
}

/****************************************/
/****************************************/

const CVector2& CEpuckFrontalBarrier::AgentRepulsionPotential() const {
    CVector2 cVector;

    if (m_pcRABSensor != NULL) {
        const CCI_EPuckRangeAndBearingSensor::TPackets& packets = m_pcRABSensor->GetPackets();

        for (size_t i = 0; i < packets.size(); ++i) {
            if (packets[i]->Data[0] == AGENT_SIGNAL && packets[i]->Range < m_fAgentPotentialDistance) {
                Real fLennardJonesValue = LennardJones(packets[i]->Range, m_fAgentPotentialGain, m_fAgentPotentialDistance);
                CVector2 cAgentLennardJones(fLennardJonesValue, packets[i]->Bearing);
                cVector += cAgentLennardJones;
            }
        }
    }

    return cVector;
}

/****************************************/
/****************************************/

const CVector2& CEpuckFrontalBarrier::DefaultPotential() const {
    CVector2 cVector;
    bool bHumanFound = false;

    if (m_pcRABSensor != NULL) {
        const CCI_EPuckRangeAndBearingSensor::TPackets& packets = m_pcRABSensor->GetPackets();

        for (size_t i = 0; i < packets.size(); ++i) {
            if (packets[i]->Data[0] >= HUMAN_SIGNAL_MIN && packets[i]->Data[0] <= HUMAN_SIGNAL_MAX) {
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
void CEpuckFrontalBarrier::BlockedMode() {
    LOG << "Blocked Mode !" << std::endl;
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

inline Real CEpuckFrontalBarrier::LennardJones(Real f_x, Real f_gain, Real f_distance) const {
    Real fRatio = f_distance / f_x;
    fRatio *= fRatio;
    return -4 * f_gain / f_x * ( fRatio * fRatio - fRatio );
}

/****************************************/
/****************************************/

REGISTER_CONTROLLER(CEpuckFrontalBarrier, "e-puck_frontal_barrier_controller");
