#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include "Multiservo.h"

float RAD = 60;
float a = 150, b = 150;
float max_angle = 360;
float step = 20 ;
bool toggle = false;

struct Leg{
	byte numLeg;

	float sx,sy;
	float angle = 0;
	float ex,ey,hx,hy;

	Multiservo s[3];

	void IK(){
			if(angle > max_angle)
			  {
				  angle = 0;
			  }
			  else{
				  angle += step;
			  }

			float dx = RAD * sin( radians( angle )) + a;
			float dy = RAD * cos( radians( angle )) + b;

			float distance = sqrt(dx*dx+dy*dy);
			float c = min(distance, a + b);

			float B = acos((b*b-a*a-c*c)/(-2*a*c));
			float C = acos((c*c-a*a-b*b)/(-2*a*b));

			float D = atan2(dy,dx);
			float E = D + B + PI + C;

			ex = cos(E) * a + sx;
			ey = sin(E) * a + sy;

			hx = cos(D+B) * b + ex;
			hy = sin(D+B) * b + ey;

			if(numLeg < 3){
				s[0].write(degrees(D+B));
				s[1].write(180 - constrain(map(degrees(E), 250, 480, 0, 180), 50, 170));
				s[2].write(degrees(D+B)+40);
			}else{
				s[0].write(180 - degrees(D+B));
				s[1].write(constrain(map(degrees(E), 250, 480, 0, 180), 50, 170));
				s[2].write(180 - (degrees(D+B)+40));
			}
			Serial.println(String(s[0].read())+String("\t")+String(s[1].read()));
	}
};

Leg leg[6];

void setup()
{
	Serial.begin(9600);
	Wire.begin();
	int count = 0;
	for(byte i = 0; i < 6; i++){
		leg[i].numLeg = i;
		for(byte j = 0; j < 3; j++){
			leg[i].s[j].attach(count++);
		}
	}
}


void loop()
{
	  	 leg[0].IK();
	     leg[2].IK();
	     leg[4].IK();

	   if(leg[0].angle >= 360/(5/2))
	     toggle = true;

	   if(toggle){
	     leg[1].IK();
	     leg[3].IK();
	     leg[5].IK();
	   }

}
