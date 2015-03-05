
#include "frontal_barrier_lp.h"

namespace argos {

   static const CVector2 CIRCLE_CENTER1(-3.0, -3.0);
   static const CVector2 CIRCLE_CENTER2(3.0, -3.0);
   static const CVector2 CIRCLE_CENTER3(3.0, 3.0);
   static const CVector2 CIRCLE_CENTER4(-3.0, 3.0);

   /****************************************/
   /****************************************/
   
   CColor CFrontalBarrierLP::GetFloorColor(const CVector2& c_pos_on_floor) {
      /*if ((c_pos_on_floor - CIRCLE_CENTER1).SquareLength() < 9 || (c_pos_on_floor - CIRCLE_CENTER2).SquareLength() < 9 || (c_pos_on_floor - CIRCLE_CENTER3).SquareLength() < 9 || (c_pos_on_floor - CIRCLE_CENTER4).SquareLength() < 9)
         return CColor::BLACK;
      else
         return CColor::WHITE;*/

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
      	}

   }

/* Register this loop functions into the ARGoS plugin system */
REGISTER_LOOP_FUNCTIONS(CFrontalBarrierLP, "frontal_barrier_lp");

}
