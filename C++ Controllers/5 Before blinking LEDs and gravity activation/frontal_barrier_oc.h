/*
 * AUTHOR: Anthony Debruyn <antdebru@ulb.ac.be>
 * 
 *
 * This controller is meant to be used with the XML files:
 *    frontal_barrier_omnidirectional_camera.argos
 */

#ifndef FRONTAL_BARRIER_OC_H
#define FRONTAL_BARRIER_OC_H

/*
 * Include some necessary headers.
 */
#include <stdlib.h>

/* Definition of the CCI_Controller class. */
#include <argos3/core/control_interface/ci_controller.h>
/* Definition of the differential steering actuator */
#include <argos3/plugins/robots/e-puck/control_interface/ci_epuck_wheels_actuator.h>
/* Definition of the e-puck proximity sensor */
#include <argos3/plugins/robots/e-puck/control_interface/ci_epuck_proximity_sensor.h>
/* Definition of the e-puck range and bearing */
#include <argos3/plugins/robots/e-puck/control_interface/ci_epuck_omnidirectional_camera_sensor.h>
/* Definition of the e-puck RGB LED actuator */
#include <argos3/plugins/robots/e-puck/control_interface/ci_epuck_rgb_leds_actuator.h>
#include <argos3/plugins/robots/generic/control_interface/ci_virtual_rgb_ground_sensor.h>
#include <argos3/core/utility/math/vector2.h>
#include <argos3/core/utility/datatypes/color.h>
#include <vector>

/*
 * All the ARGoS stuff in the 'argos' namespace.
 * With this statement, you save typing argos:: every time.
 */
using namespace argos;

/*
 * A controller is simply an implementation of the CCI_Controller class.
 */
class CEpuckFrontalBarrierOC : public CCI_Controller {

public:

   /* Class constructor. */
    CEpuckFrontalBarrierOC();

   /* Class destructor. */
    virtual ~CEpuckFrontalBarrierOC() {}

   /*
    * This function initializes the controller.
    * The 't_node' variable points to the <parameters> section in the XML
    * file in the <controllers><footbot_diffusion_controller> section.
    */
    virtual void Init(TConfigurationNode& t_node);

   /*
    * This function is called once every time step.
    * The length of the time step is set in the XML file.
    */
    virtual void ControlStep();

   /*
    * This function resets the controller to its state right after the
    * Init().
    * It is called when you press the reset button in the GUI.
    * In this example controller there is no need for resetting anything,
    * so the function could have been omitted. It's here just for
    * completeness.
    */
    virtual void Reset();

   /*
    * Called to cleanup what was done by Init() when the experiment finishes.
    * In this example controller there is no need for clean anything up,
    * so the function could have been omitted. It's here just for
    * completeness.
    */
    virtual void Destroy() {}

private:

   /* parse the <params> xml tree from the config file */
    void ParseParams(TConfigurationNode& t_node);
   // Step method for the agents.
    void NormalMode();
    void BlockedMode();
    void ComputeDirection(CVector2& cResultVector);
    const CVector2 HumanPotential();
    const CVector2 GravityPotential() const;
    const CVector2 AgentRepulsionPotential() const;
    const CVector2 DefaultPotential() const;
    inline Real LennardJones(Real f_x, Real f_gain, Real f_distance) const;
    inline Real LennardJonesStrongAttraction(Real f_x, Real f_gain, Real f_distance) const;
    inline bool IsHuman(const CColor& c_color) const;
    inline bool IsSameColor(const CColor& c_color_1, const CColor& c_color_2) const;
    bool HumanFound() const;
    inline const CColor GetAgentSituationColor();
    inline const bool IsInDanger() const;

  /*
  * The following variables are used as parameters for the
  * algorithm. You can set their value in the <parameters> section
  * of the XML configuration file, under the
  * <controllers><footbot_diffusion_controller> section.
  */
    Real m_fDefaultWheelsSpeed;

    Real m_fHumanPotentialGain;
    Real m_fHumanPotentialDistance;
    CColor m_cHumanLeftColorRef;
    CColor m_cHumanRightColorRef;

    Real m_fAgentPotentialGain;
    Real m_fAgentPotentialDistance;
    CColor m_cAgentGoodColor;
    CColor m_cAgentBadColor;
    CColor m_cAgentGoodColorRef;
    CColor m_cAgentBadColorRef;

    Real m_fGravityPotentialGain;

    /* Pointer to the differential steering actuator */
    CCI_EPuckWheelsActuator* m_pcWheelsActuator;
    /* Pointer to the e-puck proximity sensor */
    CCI_EPuckProximitySensor* m_pcProximitySensor;
    /* Pointer to the e-puck omnidirectional camera */
    CCI_EPuckOmnidirectionalCameraSensor* m_pcOmnidirectionalCameraSensor;
    /* Pointer to the e-puck RGB LED actuator */
    CCI_EPuckRGBLEDsActuator* m_pcRGBLED;
    /* Pointer to the e-puck virtual RGB ground sensor */
    CCI_VirtualRGBGroundSensor* m_pcVirtualRGBGroundSensor;

    /* Blocking System Variables */
    UInt8 m_unBSDirection;
    UInt8 m_unBSCount;

    /* Human potential distance variation variable */
    Real m_fHumanPotentialModifiedDistance;
    /* Human potential distance variation delta */
    Real m_fHumanPotentialDistanceVariationDelta;

    /* Agent color countdown counter */
    UInt8 m_unColorCountdownCounter;

    std::vector<CVector2> m_vecDirectionVectorsWindow;
    UInt8 m_unDirectionVectorsWindowSize;

};

#endif
