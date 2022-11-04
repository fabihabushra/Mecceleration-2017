#define sensorNum 8          //always even

#define maxSpeed 180//180  //180 //200
#define sSpeed 150

int blackLimit[sensorNum];
int digitalReading[sensorNum];

const int motorPin1=10,motorPin2=9;        //right motor
const int motorPin3=6,motorPin4=5;       //left motor

const int startIR=0;
const int initIR=1;         //first ir of pid

float error, prevError=0;
float mappedValue, targetValue = 8;      //as pid is for 7 ir AND IR STARTS AT 1

float safety=0.65;               //changed for white on black

float kp=26.7;   //26.7G //26        //38 //37  //39                //45 IF DOESN'T WORK
float kd=54;  //57.3  //50G //50    //67             //70  //42 //87

int rotationSpeed=80;

int motorResponse;
float correction;

int leftSpeed,rightSpeed;

float lastAct;
int time=5;

int allBlack=0;
int leftBlack=0;
int rightBlack=0;

int insideLoop=0;
//int caveSpeed=;
//int turnSpeed=;

const int topIR=0;                 //topmost IR 
int leftTrans=0;
int rightTrans=0;
int node;
int currentNode=0;

int buzzer=12;

int left=0,right=0;                   //Left and Right most Irs
int contiBlack = 0;
int stopTrigger = 1 ; //2
int startTrigger = 3;


void setup()
{
  //initialize IR pins
  for(int i = 0; i < sensorNum; i++)          //A0 is leftmost Ir
  {
    pinMode(A0 + i, INPUT);
  }

  //pinMode(frontIR,INPUT);
  //pinMode(rightIR,INPUT);
  //pinMode(leftIR,INPUT);

  //initialize motor pins
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT); 
  pinMode(motorPin3, OUTPUT);
  pinMode(motorPin4, OUTPUT);
  delay(1000);
  Serial.begin(9600);
  
  Serial.println("Calibrating");
  calibration();
  sensorMapping();

  while(allBlack)                   //prevent creating node at start point
    {
      motor(sSpeed,sSpeed);
      sensorMapping();   
    }
   contiBlack=0; 
}





void loop()
{
 
if(stopTrigger < contiBlack) 
{ 
  while(allBlack == 1) {brake(); startTrigger = 0; sensorMapping();}
   //if(mappedValue == 100) {delay(100); sensorMapping(); if(mappedValue == 100) brake();}
} 

//sensorRead();

/*
	while(1)
	{
sensorMapping();
Serial.println(nodeDetection());
}
Serial.print(" ");

if(stopTrigger < contiBlack) 
{ 
  while(allBlack == 1) {brake(); startTrigger = 0; sensorMapping();}
   //if(mappedValue == 100) {delay(100); sensorMapping(); if(mappedValue == 100) brake();}
}

for(int i = startIR; i < sensorNum; i++)
{
  //Serial.print(analogRead(A0+i));
  if(analogRead(A0+i)<blackLimit[i])
     Serial.print("1");
  else
     Serial.print("0");   
  Serial.print(" ");
}
  Serial.print("left: ");
  Serial.print(left);
  Serial.print(" right: ");
  Serial.print(right);
  Serial.println();
  */

 /*
  while(!digitalRead(frontIRpin))    // if reading 0 object is close
  {
  	brake();
  }
 */ 

  sensorMapping();

  if(nodeDetection())
  {
    //assign Nodes
    motor(-100,-100);
    brake();
    soundBuzzer();
    move();

  }
  else if(mappedValue!=100 || (contiBlack > stopTrigger && allBlack == 1))
  {
    pid();      
    motor(leftSpeed,rightSpeed);

  }
  else if(mappedValue==100)
  {
      if(abs(prevError)<1)
        {
          ACRotate(rotationSpeed);
          do{
          sensorMapping();
          }
          while((prevError>=0)||(mappedValue==100));
          pid();
          motor(leftSpeed,rightSpeed);                 // false Road
          left=0;
          right=0;
        }
      else
      { 
        if((left==1)&&(right==0))
            {
              ACRotate(rotationSpeed);   
              while(mappedValue==100) sensorMapping();
              while(digitalReading[initIR+2]!=1)            
              {
                sensorMapping();
                pid();
                motor(leftSpeed,rightSpeed);
              }      
              left=0;                                     //changed
            }
        else if((right==1)&&(left==0))
            {
              CRotate(rotationSpeed);   
              while(mappedValue==100) sensorMapping();
              while(digitalReading[sensorNum-4]!=1)        //-3 before -4 as rightmost ir not used in PID
                {
                  sensorMapping();
                  pid();
                  motor(leftSpeed,rightSpeed);
                }
              right=0;                                     //changed
        }
        else if((!right)&&(!left))
            {
              ACRotate(rotationSpeed);
              do{
                 sensorMapping();
                }
              while((prevError>=0)||(mappedValue==100));
              pid();
              motor(leftSpeed,rightSpeed);                 // false Road
              left=0;
              right=0;
            }
        else 
            {
                    //node=1; 
            }   
       } 
  }  


  if(mappedValue!=100 || (contiBlack > stopTrigger && allBlack == 1))     // add conditions                    
    {
      pid();      
      motor(leftSpeed,rightSpeed);
      
    }
  else                                 // all white
    {
        
        if((left==1)&&(right==0))
        {
            ACRotate(rotationSpeed);   
            while(mappedValue==100) sensorMapping();
            while(digitalReading[initIR+2]!=1)            
            {
              sensorMapping();
              pid();
              motor(leftSpeed,rightSpeed);
            }      
              left=0;                                     //changed
        }
        else if((right==1)&&(left==0))
        {
            CRotate(rotationSpeed);   
            while(mappedValue==100) sensorMapping();
            while(digitalReading[sensorNum-4]!=1)        //-3 before -4 as rightmost ir not used in PID
            {
             sensorMapping();
             pid();
             motor(leftSpeed,rightSpeed);
            }
             right=0;                                     //changed
        }
        else
        {
            motor(150,150);
            while(mappedValue==100) sensorMapping();
            pid();
            motor(leftSpeed,rightSpeed);
        }

      
    }
     

Serial.println();
}


void sensorMapping()
{

int sum=0,coun=0;
int leftDetect=0,rightDetect=0;
leftTrans=0;
rightTrans=0;

 for (int i = initIR; i <sensorNum; i++)    // 7 ir pid
  { 
    if (analogRead(A0+i) > blackLimit[i])              //A0 is leftmost IR  //> due to white line
     { 
      
      sum += i*2;
      coun++;
      digitalReading[i]=1;
    }
    else digitalReading[i]=0;
  }

   if(digitalReading[0]||digitalReading[sensorNum-1])  //if left or right gives black saves their value
   {
     left=digitalReading[0];
     right=digitalReading[sensorNum-1];
   }

   if(left)leftDetect=1;
   else if(right) rightDetect=1;
   if(leftDetect)
   {
    if(!left)
    {
      leftTrans=1;
      leftDetect=0;
    }
   }
   
   if(rightDetect)
   {
    if(!right)
    {
      rightTrans=1;
      rightDetect=0;
    }
   }
    
   
   if(coun!=0){                   
  mappedValue = sum / coun;
   }
   else mappedValue=100;         //all White detection 

   if(coun==sensorNum) 
   {
    contiBlack++;
   
    allBlack=1;
   
   //allblack detection
      }
   else if(startTrigger == 0 && mappedValue != 7 && coun != sensorNum)
   {
    allBlack = 1;
    //Serial.println("All black");

   }

    else
    {
    allBlack=0; 
    contiBlack = 0;  
    }
  /* 
  int  leftAND=1;
  int  rightAND=1;
  int  leftOR=0;
  int  rightOR=0;

   int i=0;
   while(i<(sensorNum/2)-1)                      // (sensorNum/2)-1=3   and < before
   {
    
    leftAND= leftAND && digitalReading[i];
    rightAND= rightAND && digitalReading[sensorNum-i-1];
    
    leftOR= leftOR || digitalReading[i];
    rightOR= rightOR || digitalReading[sensorNum-i-1];

    i++;
   }
   
    if((leftOR==0)&&(rightAND==1)) 
    {  
   // Serial.print("All Right black");
    rightBlack=1;               //all right
    }
    else rightBlack=0;

    if((leftAND==1)&&(rightOR==0)) 
    {
    // Serial.print("All Left black");  
     leftBlack=1;                //all Left
    }
    else leftBlack=0;
   */


  //Serial.print("\tmappedValue: ");
  //Serial.print(mappedValue);
 
}


void pid()
{
  
  error=targetValue-mappedValue;
  correction=(kp*error)+(kd*(error-prevError));
  prevError=error;
  motorResponse=(int)correction;
 
 if(motorResponse>maxSpeed) motorResponse=maxSpeed;
 
if(motorResponse<-maxSpeed) motorResponse=-maxSpeed;

   if(motorResponse>0)
  {
    rightSpeed=maxSpeed;
    leftSpeed=maxSpeed-motorResponse;
  }
  else if(motorResponse<0)
  {
    rightSpeed=maxSpeed+ motorResponse;
    leftSpeed=maxSpeed;
  }
  //non aggressive straight
  else
  {
    rightSpeed= sSpeed;
    leftSpeed= sSpeed;
  }

}


void motor(int left, int right)
{
  
  if(right>0)
  {
  analogWrite(motorPin1,right);
  analogWrite(motorPin2,0);
  }
  else
  {
    analogWrite(motorPin1,0);
    analogWrite(motorPin2,-right);
  }

  if(left>0)
  {
  analogWrite(motorPin3,left);
  analogWrite(motorPin4,0);
  }
  else
  {
   analogWrite(motorPin3,0);
   analogWrite(motorPin4,-left); 
  }

 }

void CRotate(int rotateSpeed)
{
  analogWrite(motorPin1,0);
  analogWrite(motorPin2, rotateSpeed);
  analogWrite(motorPin3, rotateSpeed);
  analogWrite(motorPin4,0);

}

void ACRotate(int rotateSpeed)
{
  analogWrite(motorPin1,rotateSpeed);
  analogWrite(motorPin2, 0);
  analogWrite(motorPin3, 0);
  analogWrite(motorPin4,rotateSpeed);

}

void brake(void)
{
  analogWrite(motorPin1, 0);
  analogWrite(motorPin2, 0);
  analogWrite(motorPin3, 0);
  analogWrite(motorPin4, 0);
}


int nodeDetection()
{
	if((leftTrans)&&(rightTrans))
	{
		if(digitalReading[topIR])
		{
			node=4;   //forward left right
		}
		else
		{
			node=1;   //left and right
		}
	}
	else if(leftTrans)
	{
		if(digitalReading[topIR])
		{
			node=2;   //forward left 
		}
		else
    {
      node=0;   //not a node
    }
	}
	else if(rightTrans)
	{
		if(digitalReading[topIR])
		{
			node=3;   //forward left 
		}
		else
		{
			node=0;   //not a node
		}
	}
  else
  {
    node=0;  // not a node
  }
 return node;
}

void move()
{
  int mode=0;                            //0 for left 1 for right

  if(mode=0)                             //for left based
  {  
    if(node==3)
      {
        pid();
        motor(leftSpeed,rightSpeed);
      }
     else if(node)
     {
        ACRotate(rotationSpeed);
        do{
        sensorMapping();
        }
        while((prevError>=0)||(mappedValue==100));
        pid();
        motor(leftSpeed,rightSpeed);
     }
  }
  else                                    //for right based
  {
     if(node==2)
      {
        pid();
        motor(leftSpeed,rightSpeed);
      }
     else if(node)
     {
        CRotate(rotationSpeed);
        do{
        sensorMapping();
        }
        while((prevError<=0)||(mappedValue==100));
        pid();
        motor(leftSpeed,rightSpeed);
     }
  }

  left=0;
  right=0;      

}

void soundBuzzer()
{
  digitalWrite(buzzer,HIGH);
  delay(5);
  digitalWrite(buzzer,LOW);
}

 //auto calibration
void calibration()
{
  CRotate(50);
  float upSum = 0,lowSum = 0;
  int sensorArray[sensorNum][2];

  for(int i = startIR; i < sensorNum; i++)
    {
      sensorArray[i][0] = analogRead(A0+i);
      sensorArray[i][1] = analogRead(A0+i);
    }
 

  int loopCounter = (int)(time * 1000 / 2.5);  
  while(loopCounter)
  {
    for(int i =startIR; i < sensorNum; i++)
    {
      if(analogRead(A0+i)<sensorArray[i][0]) sensorArray[i][0]=analogRead(A0+i);
      if(analogRead(A0+i)>sensorArray[i][1]) sensorArray[i][1]=analogRead(A0+i);
    }
  loopCounter--;

  }

 for(int i=startIR; i < sensorNum; i++)
  blackLimit[i] = (int)(sensorArray[i][0] + safety * (sensorArray[i][1] - sensorArray[i][0]));

  brake();
  delay(1000);

}
