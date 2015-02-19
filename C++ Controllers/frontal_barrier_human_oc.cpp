#include "frontal_barrier_human_oc.h"
#include <argos3/core/utility/logging/argos_log.h>


using namespace argos;

/****************************************/
/****************************************/

static const CColor DEFAULT_COLOR = CColor::CYAN;

/****************************************/
/****************************************/

CEpuckFrontalBarrierStaticOC::CEpuckFrontalBarrierStaticOC() :
    m_fLeftSpeed(0),
    m_fRightSpeed(0),
    m_cColor(DEFAULT_COLOR),
    m_pcWheelsActuator(NULL),
    m_pcProximitySensor(NULL),
    m_pcOmnidirectionalCameraSensor(NULL),
    m_pcRGBLED(NULL) {
}

/****************************************/
/****************************************/

void CEpuckFrontalBarrierStaticOC::ParseParams(TConfigurationNode& t_node) {
    UInt8 unRed = DEFAULT_COLOR.GetRed(), 
    unGreen = DEFAULT_COLOR.GetGreen(), 
    unBlue = DEFAULT_COLOR.GetBlue();

    try {
        /* Human agent left wheel speed */
        GetNodeAttributeOrDefault(t_node, "leftSpeed", m_fLeftSpeed, m_fLeftSpeed);
        /* Human agent right wheel speed */
        GetNodeAttributeOrDefault(t_node, "rightSpeed", m_fRightSpeed, m_fRightSpeed);
        /* Human agent color red component */
        GetNodeAttributeOrDefault(t_node, "red", unRed, unRed);
        /* Human agent color green component */
        GetNodeAttributeOrDefault(t_node, "green", unGreen, unGreen);
        /* Human agent color blue component */
        GetNodeAttributeOrDefault(t_node, "blue", unBlue, unBlue);
    } catch (CARGoSException& ex) {
        THROW_ARGOSEXCEPTION_NESTED("Error parsing <params>", ex);
    }

    m_cColor.Set(unRed, unGreen, unBlue);
}

/****************************************/
/****************************************/

void CEpuckFrontalBarrierStaticOC::Init(TConfigurationNode& t_node) {
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
    try {
        m_pcRGBLED = GetActuator<CCI_EPuckRGBLEDsActuator>("epuck_rgb_leds");

        if (m_pcRGBLED != NULL) {
            m_pcRGBLED->SetColors(m_cColor);
        }
    } catch (CARGoSException ex) {}
}

/****************************************/
/****************************************/

void CEpuckFrontalBarrierStaticOC::Reset() {
    if (m_pcWheelsActuator != NULL) {
        m_pcWheelsActuator->SetLinearVelocity(m_fLeftSpeed, m_fRightSpeed);
    }
    if (m_pcRGBLED != NULL) {
        m_pcRGBLED->SetColors(m_cColor);
    }
}

/****************************************/
/****************************************/

void CEpuckFrontalBarrierStaticOC::ControlStep() {

}

REGISTER_CONTROLLER(CEpuckFrontalBarrierStaticOC, "e-puck_frontal_barrier_human_controller");
