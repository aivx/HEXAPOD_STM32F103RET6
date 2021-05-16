import processing.net.*; 
import controlP5.*;

Toggle t;
Slider s;
Slider2D s2D;
CheckBox checkbox;
ControlP5 cp5;
Client myClient;

public float RAD = 40;
public int a = 140, b = 160;
public float max_angle = 360;
public float min_angle = 0;
public float step = 5;
public boolean pause = false;
public boolean back = false;
public boolean inverce_servo = true;

Boolean[] toggle = {false, false, false};
byte inverse_count = 3;

@ControlElement (x=1200, y=100, label="speed", properties = {"type=numberbox","value=100","min=0", "max=200","height=220", "width=25"})
public void speed(float val){
  this.step = val/10;
}
@ControlElement (x=975, y=100, label="radius", properties = {"type=numberbox","value=50","min=0", "max=70","height=240", "width=25"})
public void radius(float val){
  this.RAD = val;
}

class Leg{
  
  Toggle t1, t2;
  Slider s1, s2;
  Slider2D s2D1, s2D2;
  
  CheckBox checkbox;
  
  public int sx,sy; 
  
  public float angle = 0.0;
  public float armLength = width/5;
  public float ex,ey,hx,hy;
  public float dx, dy;
  public int x = 90, y = 100 ,z = 135;
  
  Leg(String name,CheckBox checkbox, Toggle t1, Slider s1, Slider2D s2D1, int sx, int sy){
    this.checkbox = checkbox;
    this.sx = sx;
    this.sy = sy;
    this.t1 = t1;
    this.t2 = cp5.addToggle("t"+name)
           .setPosition(sx,sy+220)
           .setSize(50,20)
           .setState(true) 
           ;

    this.s1 = s1;
    this.s2 = cp5.addSlider("S"+name)
           .setPosition(sx,sy+200)
           .setSize(200,20)
           .setRange(0,180)
           .setValue(90)
           .setSliderMode(Slider.FLEXIBLE)
           ;
           
    this.s2D1 = s2D1;
    this.s2D2 = cp5.addSlider2D("\n\n 2D"+name)
           .setPosition(sx,sy)
           .setSize(200,200)
           .setMinMax(sx+25, sy+25, sx+400, sy+400)
           .setValue(sx+213, sy+220)
           //.disableCrosshair()
           ;
  }
  public void reset(){
    angle = 0;
  }
  
  public void IK(float dx, float dy){
    
     s2D2.setVisible(!checkbox.getState(0)); 
     
     this.dx = dx;
     this.dy = dy;
     
     float distance = sqrt(dx*dx+dy*dy);
     float c = min(distance, a + b);
     
     float B = acos((b*b-a*a-c*c)/(-2*a*c));
     float C = acos((c*c-a*a-b*b)/(-2*a*b));
     
     float D = atan2(dy,dx);
     float E = D + B + PI + C;
     
     ex = cos(E) * a + sx;
     ey = sin(E) * a + sy;
     int y = (int)degrees(E);
     this.y = (int)constrain(map(y == y ? y:90, 210, 470,0,180),0,180);
     //val2 = (int)constrain(map(degrees(E),354,396,0,180),0,180);
     //val2 = (int)degrees(E);
    // print("\tUpperArm Angle=\t"+y);
     
     hx = cos(D+B) * b + ex;
     hy = sin(D+B) * b + ey;
     //val1 = 180 - (int)constrain(map(degrees(D+B),-90,90,0,180),0,180);
     int x = (int)degrees(D+B);
     this.x = int(180 - (x == x ? x:90));
    // print("\tLowerArm Angle=\t"+x);
     //println(this.x+"\t"+x);
     //if(!mousetongled)
     ;//val3 = 180 -(int)constrain(map(angle,0,360,40,80),40,80);
    // println("\tlengtht=\t"+z);
     stroke(255,0,0,100);
     fill(240,0,0,200);
     ellipse(sx,sy,20,20);
     ellipse(ex,ey,8,8);
     ellipse(hx,hy,6,6);
     stroke(0);
     line(sx,sy,ex,ey);
     line(ex,ey,hx,hy); 
  }
  
  public void upperArm(){
    if(pause){
        if(back ? angle > max_angle : angle < max_angle)
          {
            angle += step;
          }
          else{
            angle = min_angle;
          }
    }
   // println(angle);
  if(s2D1.isMousePressed()){
    IK(map(s2D1.getArrayValue()[0],0,200,sx+25,sx+400) - sx, map(s2D1.getArrayValue()[1],0,200,sy+25,sy+400) - sy);
  }else 
  if(s2D2.isMousePressed()){
    IK(s2D2.getArrayValue()[0] - sx,s2D2.getArrayValue()[1] - sy);
  }else
  if(t2.getState() && t1.getState()){
    IK((RAD * sin( radians( angle )) + a),(RAD * cos( radians( angle )) + b-50));
  }else{
    IK(dx,dy);
  }
  
 if(s1.isMousePressed()){
   z = (int)s1.getValue();
 }else
 if(s2.isMousePressed()){
   z = (int)s2.getValue();
 }
 
 }
}

Leg[] leg = new Leg[6];

int[] values = {255, 0, 90, 100, 135};
Boolean[] state = {false, false, false, false, false};

void state_toggle(int j){
  for(byte i = 0; i< state.length;i++){
    if(i == j){
      state[i] = true;
    }else{
      state[i] = false;
    }
  }
}

byte[] convert(int[] arr){
  byte[] out = new byte[arr.length];
  for(int i = 0; i< arr.length; i++)
      out[i] = byte(arr[i] - 128);
  return out;
}

int[] convert(byte[] arr){
  int[] out = new int[arr.length];
  for(int i = 0; i< arr.length; i++)
      out[i] = arr[i] + 127;
  return out;
}

void clientEvent(Client someClient) {
  for(byte i = 0; i<6; i++){
    values[1] = i;
    int x, y, z;
    if(inverce_servo){
      x = leg[i].x;
      y = leg[i].y;
      z = leg[i].z;
    }else{
      x = 90;
      y = leg[i].x+20;
      z = leg[i].y+45;
    }
    
    values[2] = x;
    
    if(i<inverse_count){
      values[3] = y;
    }else{
      values[3] = 180 - y;
    } 
    if(i<3){
      values[4] = z;
    }else{
      values[4] = 180 - z;
    } 
   // myClient.write(convert(values));
  }
  
  myClient.write(convert(values));
  //p.write(convert(values));
   //printArray(vals);
   //printArray(m);
   //printArray(myClient.readBytes(4));
    
}


void setup(){
 size(1300,700);
 myClient = new Client(this, "192.168.1.182", 80);
 //myClient.write("connect");
 
 cp5 = new ControlP5(this);
 cp5.addControllersFor(this);
 cp5.enableShortcuts();
    t = cp5.addToggle("tGeneral")
         .setPosition(1000,320)
         .setSize(50,20)
         .setState(true)
         ;
    s = cp5.addSlider("S")
         .setPosition(1000,300)
         .setSize(200,20)
         .setRange(0,180)
         .setValue(90)
         .setSliderMode(Slider.FLEXIBLE)
         ;
         
    s2D = cp5.addSlider2D("\n\nS2D")
         .setPosition(1000,100)
         .setSize(200,200)
         .setMinMax(0, 0, 200, 200)
         .setValue(100, 100)
         //.disableCrosshair()
         ;
    checkbox = cp5.addCheckBox("checkBox")
         .setPosition(1, 1)
         .setColorForeground(color(120))
         .setColorActive(color(255))
         .setColorLabel(color(255))
         .setSize(40, 40)
         .addItem("0", 0)
         .toggle(0);
         ;
  /*   n = cp5.addNumberbox("speed")
         .setSize(25, 220)
         .setPosition(1150, 100)
         .setRange(0,10)
         .setValue(5)
         ;*/
         
 leg[5] = new Leg("1",checkbox,t,s,s2D,100,100);
 leg[4] = new Leg("2",checkbox,t,s,s2D,400,100);
 leg[3] = new Leg("3",checkbox,t,s,s2D,700,100);
 leg[0] = new Leg("4",checkbox,t,s,s2D,100,400);
 leg[1] = new Leg("5",checkbox,t,s,s2D,400,400);
 leg[2] = new Leg("6",checkbox,t,s,s2D,700,400);
  started();
  
}

void resetall(){
  inverse_count = 3;
  
  for(byte i = 0; i< toggle.length;i++)
  toggle[i] = false;
  
  for(byte i = 0; i < 6; i++)
  leg[i].reset();
}

void started(){
      resetall();
      state_toggle(0);
      pause = false;
}

void walking0(){
  for(byte i = 0; i < 6; i++){
   leg[i].upperArm();
   //println("leg="+i+"\t"+leg[i].x+"\t"+leg[i].y+"\t"+leg[i].z);
   }  
}

void walking1Up(boolean b){
  
  if(!toggle[0]){
     inverce_servo = b;
     max_angle = 360;
     min_angle = 0;
     step = abs(step);
     back = false;
     toggle[0] = true;
  }
  
     leg[0].upperArm();
     leg[2].upperArm();
     leg[4].upperArm();
     
   if(leg[0].angle >= 360/(5/2))
     toggle[1] = true;
     
   if(toggle[1]){
     leg[1].upperArm();
     leg[3].upperArm();
     leg[5].upperArm();
   }
}
void walking1Down(boolean b){
  if(!toggle[0]){
     inverce_servo = b;
     max_angle = 0;
     min_angle = 360;
     step = -abs(step);
     back = true;
     toggle[0] = true;
  }
  
     leg[0].upperArm();
     leg[2].upperArm();
     leg[4].upperArm();
     
   if(leg[0].angle <= 360/(5/2))
     toggle[1] = true;
     
   if(toggle[1]){
     leg[1].upperArm();
     leg[3].upperArm();
     leg[5].upperArm();
   }
}
void walking2(){
   leg[0].upperArm();
   leg[5].upperArm(); 
   
   if(leg[0].angle >= 360/(5/2))
     toggle[0] = true;
     
   if(toggle[0]){
   leg[1].upperArm();
   leg[4].upperArm();
   }
   if(leg[1].angle >= 360/(5/2))
     toggle[1] = true;
   if(toggle[1]){
   leg[2].upperArm();
   leg[3].upperArm();
   }
}

void draw(){
 background(255, 224, 150);
 //translate(160,140);
 fill(255);
 rect(0,0,width,height);
 if(keyPressed){
     switch(keyCode) {
      case(UP):  
        state_toggle(1);
      break;
      case(DOWN):
        state_toggle(2);
      break;
      case(LEFT):
        state_toggle(3);
      break;
      case(RIGHT):
        state_toggle(4);
      break;
     }
 } 
   if(state[0]){
     walking0();
   }else if(state[1]){
     walking1Up(true);
   }else if(state[2]){
     walking1Down(true);
   }else if(state[3]){
     walking1Up(false);
   }else if(state[4]){
     walking1Down(false);
   }else if(state[5]){
     walking2();
   }
 
 }


void keyReleased(){
  switch(keyCode) {
    //***AWSD******
    case(UP):
      resetall();
      state_toggle(0);
      pause = false;
    break;
    case(DOWN):
      resetall();
      state_toggle(0);
      pause = false;
    break;
    case(LEFT):
      resetall();
      state_toggle(0);
      pause = false;
    break;
    case(RIGHT):
      resetall();
      state_toggle(0);
      pause = false;
    break;
    //***AWSD******
  }
}
void keyPressed() {
  pause = true;
  switch(keyCode) {
    case('1'):
      started();
    break;
    case('2'):
      resetall();
      state_toggle(1);
    break;
    case('3'):
      resetall();
      state_toggle(2);
    break;
    case('4'):
      resetall();
      state_toggle(4);
      inverse_count = 6;
    break;
    case('5'):
      resetall();
      state_toggle(4);
      inverse_count = 6;
    break;
  }
}
