import processing.serial.*;
import controlP5.*;

ControlP5 cp5;

Slider2D s;
Serial myPort;
byte[] data = {1,2,3,4};

void setup() {
  size(300, 300);
    cp5 = new ControlP5(this);
    s = cp5.addSlider2D("wave")
         .setPosition(30,40)
         .setSize(100,100)
         .setMinMax(-90,-90,90,90)
         .setValue(0,0)
         //.disableCrosshair()
         ;
  printArray(Serial.list());
  delay(1000);
  String portName = Serial.list()[2];
  myPort = new Serial(this, portName, 9600);
  myPort.write(data);
  printArray(data);
}

void draw() {
    background(0);
    if(myPort.available() >= 4){
       byte[] val = myPort.readBytes(4);
       print(val[2]);
       print("\t");
       print(val[3]);
       print("\t");
       data[2] = (byte)s.getArrayValue()[0];
       data[3] = (byte)s.getArrayValue()[1];
       print(data[2]);
       print("\t");
       println(data[3]);
       myPort.write(data);
    }
}
