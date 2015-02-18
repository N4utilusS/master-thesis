#include "frontal_barrier_human_oc.h"
#include <argos3/core/utility/logging/argos_log.h>


using namespace argos;

/****************************************/
/****************************************/

DEFAULT_HUMAN_COLOR = CColor::CYAN;

/****************************************/
/****************************************/

CEpuckFrontalBarrierStatic::CEpuckFrontalBarrierStatic() :
    m_fHumanAgentLeftSpeed(0),
    m_fHumanAgentRightSpeed(0),
    m_cHumanAgentColor(DEFAULT_HUMAN_COLOR),
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
        /* Human agent color */
        GetNodeAttributeOrDefault(t_node, "humanAgentColor", m_cHumanAgentColor, m_cHumanAgentColor);
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
        m_pcOmnidirectionalCameraSensor = GetSensor<CCI_EPuckOmnidirectionalCameraSensor>("colored_blob_omnidirectional_camera");
    } catch (CARGoSException ex) {}
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
