// Program to simulate stick slip motion of a block on a surface
import grafica.*;
GPlot plot1, plot2;

float g = 9.81;      //gravitational constant
float Kp = 4000.0;   //Pusher spring
float k = 2000.0;         // springs constant hookes law
float m=3.0;        // Mass of bock kg
int N_Blocks = 7;    // Number of blocks
float TL_X = 0;      //Force
Block [] b;          //array of blocks
float length_of_fault= 10;    
float ms = 1.0;       //constant of static friction
float mk = 0.5;      //constant of kinetic friction
float dt = 0.005;      // Time step - Euler's method
float t;              //time
float Fls = 0.0;      //Force Leaf Spring
float vdriver = 0.1;  //Velocity
  

GPointsArray points1;
GPointsArray points2;
int count=0;

void setup() {     //Setup for the blocks
  b=new Block[N_Blocks+1];
  for (int i=1; i<=N_Blocks; i++) {
    b[i]=new Block(i);
  }
  //size of display window
  size(1200, 800);
  
  points1 = new GPointsArray(10000000);
  points2 = new GPointsArray(10000000);
}

void draw() {
  background(255);
  textSize(26);
  strokeWeight(5);
  
  //calculation of the force through time
  TL_X+=vdriver*dt;

  //movement of blocks
  for(int i=1; i<=N_Blocks; i++) {
    b[i].advance(i);       // Advance block through time step dt
  }
  
  draw_blocks();
  draw_axes();
     


 //plotting graphs
 if(b[7].stuck==0){
   if((count%7)==0) 
   
        points1.add(t, b[4].v);
       
        
        plot1 = new GPlot(this);
        plot1.setPos(25, 450);
        plot1.setDim(400, 200);
        plot1.setMar(60, 70, 40, 70);
        plot1.setTitleText("Velocity vs Time");
        plot1.getXAxis().setAxisLabelText("Time (s)");
        plot1.getYAxis().setAxisLabelText("Velocity (m/s^-1)");
        plot1.setPointSize(0.5);
        plot1.setPoints(points1);
        plot1.defaultDraw();
        plot1.drawGridLines(GPlot.VERTICAL);
          
        plot1.beginDraw();
        plot1.drawRightAxis();
        plot1.drawLines();
        plot1.drawPoints();
        plot1.endDraw();
        
        
        points2.add(b[4].x-TL_X, b[4].v);
        plot2 = new GPlot(this);
        plot2.setPos(600, 450);
        plot2.setDim(400, 200);
        plot2.setMar(60, 70, 40, 70);
        plot2.setTitleText("Velocity vs Position");
        plot2.getXAxis().setAxisLabelText("Position (m)");
        plot2.getYAxis().setAxisLabelText("Velocity (m/s^-1)");
        plot2.setPointSize(0.5);
        plot2.setPoints(points2);
        plot2.defaultDraw();
        plot2.drawGridLines(GPlot.VERTICAL);
          
        plot2.beginDraw();
        plot2.drawRightAxis();
        plot2.drawLines();
        plot2.drawPoints();
        plot2.endDraw();
 }
}

//drawing the table
void draw_blocks(){
    fill(255, 20, 147);
    text("                              Block number", 50, 20);
    text("                              Force", 250, 20);
    text("                              X Position", 375, 20);
    text("                              Velocity", 550, 20);
  for(int i=1; i<=N_Blocks; i++) {
    
    fill(255, 20, 147);
    text("                                      "+ (i) +"               " + nf((b[i].ForceTot), 0 , 2) +"         " + nf(b[i].x, 0 ,2) + "                " + nf(b[i].v, 0 ,2), 50, 20+(26*i));
    b[i].draw_block(i);    // Redraw the block
  }
}

//drawing layout for display window
 void draw_axes(){
    
    //x and y positions of axes
    float sx=map(TL_X, 0, length_of_fault, 0, width);   
    float sy=map(2.8, 0, 5, height-1, 0);
    
    rect(sx%width, sy, 1, 80);
    
    line(0, sy-50, width-10, sy-50);
    line(0, sy+80, width-10, sy+80);
    
  }


class Block {
  float x=0;  // Initial position of the block
  float v;  // Initial velocity of block
  int block_num;
  float block_width;
  float block_sep;
  float weight;
  float ForceTot;
  int stuck;
  

  //dimensions physics surrounding block
  Block(int i){
    block_num = i;
    x=0;
    v=0;
    block_width=0.1;
    block_sep=0.5;
    weight =m*g;
    ForceTot = 0;
    stuck=1;
   
  }
  
  //function to move the blocks through time
  void advance(int i) {

      float fi;
      float dvx=0;
      float dx=0;
      float nforce=0;
   
     //friction = weight * kinetic friction
      if(b[i].v>0){
        fi = -weight*mk;
      }
      else{
        fi = weight*mk;
      }

    //physics applied for the motion of each block in system
     Fls = Kp*(TL_X-b[1].x);
      if(i == 1){ 
        //Fls = Kp*(TL_X-b[1].x);
        nforce = (k*(b[2].x-b[1].x) + Fls + fi);
      }
      else if(i == N_Blocks){
        nforce = (k*((b[i-1].x) - (b[i].x)) + fi );
      }
      else{
        nforce = (k*((b[i+1].x) - (2*b[i].x) + (b[i-1].x)) + fi);
      }
      
      //Newtons third Law F = ma 
      if(nforce>(m*g*ms)){
        stuck=0;
      }
      if(stuck==1){
        nforce=0;
      }
      
      ForceTot=nforce;
      
      //differential equation Euler's Method
      dvx = nforce*(dt/m);
      v = v + dvx;
      dx = v * dt;
      x = x + dx;
      t = t + dt; 
  }
 

  //function to draw the blocks
  void draw_block(int i) {
    //setting up x, y, w positions of blocks 
    float sx=map(x+(float)((block_sep+block_width)*(block_num+1)), 0, length_of_fault, 0, width);   
    float sy=map(2.8, 0, 5, height-1, 0);
    float wx=map(block_width, 0, length_of_fault, 0, width);
    
      //if block is stuck, it will go from black to pink
      if (b[i].stuck!=1.0) 
      {
        fill(255, 200, 200);
      } 
      else
      {
        fill(0, 0, 0);
      }
      text(i, sx, sy-10);
      rect(sx, sy, wx, 80);
  }
}
