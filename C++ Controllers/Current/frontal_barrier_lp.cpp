
#include "frontal_barrier_lp.h"

namespace argos {

   /****************************************/
   /****************************************/

   CFrontalBarrierLP::CFrontalBarrierLP() :
      m_fDangerZoneRadiusSquared(1.0) {

   }

   void CFrontalBarrierLP::Init(TConfigurationNode& t_tree) {

      TConfigurationNode& tFrontalBarrierLFNode = GetNode(t_tree, "frontal_barrier_lp");
      /* Get danger area centre */
      GetNodeAttributeOrDefault(tFrontalBarrierLFNode, "dangerAreaCentre", m_cDangerZoneCentre, m_cDangerZoneCentre);
      
      /* Radius of the danger area */
      GetNodeAttributeOrDefault(tFrontalBarrierLFNode, "dangerAreaRadius", m_fDangerZoneRadiusSquared, m_fDangerZoneRadiusSquared);
      m_fDangerZoneRadiusSquared *= m_fDangerZoneRadiusSquared;

   }
   
   CColor CFrontalBarrierLP::GetFloorColor(const CVector2& c_pos_on_floor) {
      if ((c_pos_on_floor - m_cDangerZoneCentre).SquareLength() <= m_fDangerZoneRadiusSquared)
         return CColor::RED;
      return CColor::WHITE;
      /*
   	Real l = 4.5;
   	Real rX = c_pos_on_floor.GetX() - l*floor(c_pos_on_floor.GetX()/l);
   	Real rY = c_pos_on_floor.GetY() - l*floor(c_pos_on_floor.GetY()/l);

   	if(rY>=0 && rY<l/2) {
   		if(rX>=0 && rX<l/2)
   			return CColor::BLACK;
   		else 
   			return CColor::WHITE;
   	} else {
   		if(rX>=0 && rX<l/2)
   			return CColor::WHITE;
   		else 
   			return CColor::BLACK;
   	}*/
         //return CColor::WHITE;

   }

/* Register this loop functions into the ARGoS plugin system */
REGISTER_LOOP_FUNCTIONS(CFrontalBarrierLP, "frontal_barrier_lp");

}
