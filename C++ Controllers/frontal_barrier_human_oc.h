/*
 * AUTHOR: Anthony Debruyn <antdebru@ulb.ac.be>
 *
 *
 * This controller is meant to be used with the XML files:
 *    frontal_barrier.argos
 */

#ifndef FRONTAL_BARRIER_STATIC_OC_H
#define FRONTAL_BARRIER_STATIC_OC_H

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
/* Definition of the e-puck omnidirectional camera */
#include <argos3/plugins/robots/e-puck/control_interface/ci_epuck_omnidirectional_camera_sensor.h>
/* Definition of the e-puck RGB LED actuator */
#include <argos3/plugins/robots/e-puck/control_interface/ci_epuck_RGB_leds_actuator.h>
/* Definition of the colors*/
#include <argos3/core/utility/datatypes/color.h>

/*
 * All the ARGoS stuff in the 'argos' namespace.
 * With this statement, you save typing argos:: every time.
 */
using namespace argos;

/*
 * A controller is simply an implementation of the CCI_Controller class.
 */
class CEpuckFrontalBarrierStaticOC : public CCI_Controller {

public:

   /* Class constructor. */
    CEpuckFrontalBarrierStaticOC();

   /* Class destructor. */
    virtual ~CEpuckFrontalBarrierStaticOC() {}

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

    /*
    * The following variables are used as parameters for the
    * algorithm. You can set their value in the <parameters> section
    * of the XML configuration file, under the
    * <controllers><footbot_diffusion_controller> section.
    */

    Real m_fLeftSpeed;
    Real m_fRightSpeed;

    CColor m_cColor;

   /* Pointer to the differential steering actuator */
    CCI_EPuckWheelsActuator* m_pcWheelsActuator;
   /* Pointer to the e-puck proximity sensor */
    CCI_EPuckProximitySensor* m_pcProximitySensor;
   /* Pointer to the e-puck omnidirectional camera sensor */
    CCI_EPuckOmnidirectionalCameraSensor* m_pcOmnidirectionalCameraSensor;
   /* Pointer to the e-puck RGB LED actuator */
    CCI_EPuckRGBLEDsActuator* m_pcRGBLED;

};

#endif
