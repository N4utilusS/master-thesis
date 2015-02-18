#include "frontal_barrier_human_oc.h"
#include <argos3/core/utility/logging/argos_log.h>


using namespace argos;

/****************************************/
/****************************************/

DEFAULT_HUMAN_LEFT_COLOR = CColor::CYAN;
DEFAULT_HUMAN_LEFT_COLOR = CColor::MAGENTA;

/****************************************/
/****************************************/

CEpuckFrontalBarrierStatic::CEpuckFrontalBarrierStatic() :
    m_fHumanAgentLeftSpeed(0),
    m_fHumanAgentRightSpeed(0),
    m_cHumanAgentLeftColor(DEFAULT_HUMAN_LEFT_COLOR),
    m_cHumanAgentRightColor(DEFAULT_HUMAN_RIGHT_COLOR),
    m_pcWheelsActuator(NULL),
    m_pcProximitySensor(NULL),
    m_pcOmnidirectionalCameraSensor(NULL) {
}

/****************************************/
/****************************************/

void CEpuckFrontalBarrierStatic::ParseParams(TConfigurationNode& t_node) {
    try {
        /* Human agent left wheel speed */
        GetNodeAttributeOrDefault(t_node, "humanAgentLeftSpeed", m_fHumanAgentLeftSpeed, m_fHumanAgentLeftSpeed);
        /* Human agent right wheel speed */
        GetNodeAttributeOrDefault(t_node, "humanAgentRightSpeed", m_fHumanAgentRightSpeed, m_fHumanAgentRightSpeed);
        /* Robot ID for signal 1 */
        GetNodeAttributeOrDefault(t_node, "humanAgentSignal", m_unHumanAgentSignal, m_unHumanAgentSignal);
    } catch (CARGoSException& ex) {
        THROW_ARGOSEXCEPTION_NESTED("Error parsing <params>", ex);
    }
}

/****************************************/
/****************************************/

void CEpuckFrontalBarrierStatic::Init(TConfigurationNode& t_node) {
    /* parse the xml tree <params> */
    ParseParams(t_node);
    LOG << GetId() << std::endl;

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
        tSentData[0] = m_unHumanAgentSignal;
        m_pcRABActuator->SetData(tSentData);
    }

}

/****************************************/
/****************************************/

void CEpuckFrontalBarrierStatic::Reset() {

    if (m_pcRABActuator != NULL) {
        CCI_EPuckRangeAndBearingActuator::TData tSentData;
        tSentData[0] = m_unHumanAgentSignal;
        m_pcRABActuator->SetData(tSentData);
    }

    if (m_pcWheelsActuator != NULL) {
        m_pcWheelsActuator->SetLinearVelocity(m_fHumanAgentLeftSpeed, m_fHumanAgentRightSpeed);
    }
}

/****************************************/
/****************************************/

void CEpuckFrontalBarrierStatic::ControlStep() {

    // Clear RAB:
    m_pcRABSensor->ClearPackets();
}

REGISTER_CONTROLLER(CEpuckFrontalBarrierStatic, "e-puck_frontal_barrier_human_controller");
