#include "frontal_barrier_human_oc.h"
#include <argos3/core/utility/logging/argos_log.h>
#include <argos3/core/utility/math/vector3.h>


using namespace argos;

/****************************************/
/****************************************/

static const CColor DEFAULT_COLOR = CColor::CYAN;

/****************************************/
/****************************************/

CEpuckFrontalBarrierHumanOC::CEpuckFrontalBarrierHumanOC() :
    m_fLeftSpeed(0),
    m_fRightSpeed(0),
    m_cColor(DEFAULT_COLOR),
    m_pcWheelsActuator(NULL),
    m_pcProximitySensor(NULL),
    m_pcRGBLED(NULL) {
}

/****************************************/
/****************************************/

void CEpuckFrontalBarrierHumanOC::ParseParams(TConfigurationNode& t_node) {
    CVector3 cColorVector(DEFAULT_COLOR.GetRed(), 
    DEFAULT_COLOR.GetGreen(), 
    DEFAULT_COLOR.GetBlue());

    try {
        /* Human agent left wheel speed */
        GetNodeAttributeOrDefault(t_node, "leftSpeed", m_fLeftSpeed, m_fLeftSpeed);
        /* Human agent right wheel speed */
        GetNodeAttributeOrDefault(t_node, "rightSpeed", m_fRightSpeed, m_fRightSpeed);
        /* Human agent color */
        GetNodeAttributeOrDefault(t_node, "color", cColorVector, cColorVector);
    } catch (CARGoSException& ex) {
        THROW_ARGOSEXCEPTION_NESTED("Error parsing <params>", ex);
    }
    m_cColor.Set((UInt8) cColorVector.GetX(), (UInt8) cColorVector.GetY(), (UInt8) cColorVector.GetZ());
}

/****************************************/
/****************************************/

void CEpuckFrontalBarrierHumanOC::Init(TConfigurationNode& t_node) {
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
        m_pcRGBLED = GetActuator<CCI_EPuckRGBLEDsActuator>("epuck_rgb_leds");

        if (m_pcRGBLED != NULL) {
            m_pcRGBLED->SetColors(m_cColor);
        }
    } catch (CARGoSException ex) {}

    if (m_pcWheelsActuator != NULL) {
        m_pcWheelsActuator->SetLinearVelocity(m_fLeftSpeed, m_fRightSpeed);
    }
}

/****************************************/
/****************************************/

void CEpuckFrontalBarrierHumanOC::Reset() {
    if (m_pcWheelsActuator != NULL) {
        m_pcWheelsActuator->SetLinearVelocity(m_fLeftSpeed, m_fRightSpeed);
    }
    if (m_pcRGBLED != NULL) {
        m_pcRGBLED->SetColors(m_cColor);
    }
}

/****************************************/
/****************************************/

void CEpuckFrontalBarrierHumanOC::ControlStep() {

}

REGISTER_CONTROLLER(CEpuckFrontalBarrierHumanOC, "e-puck_frontal_barrier_human_oc_controller");
