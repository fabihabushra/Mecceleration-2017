//things to change each bot: motorpins, kp, kd, maxSpeed, rotationSpeed,contiWhite;
#define sensorNum 8
#define maxSpeed 255
/*
بسم الله الرحمن الرحيم
بسم الله توكلت علي الله ولا حول ولا قوة الا بالله العلي العظيم
*/

#define rotationSpeed 120

int blackLimit[sensorNum];

const int motorPin1 = 9,motorPin2 = 10;        //right motor
const int motorPin3 = 6,motorPin4 = 5;       //left motor

float error, prevError=0;
float mappedValue, targetValue = 8;     

float safety=0.55;

float kp=45;                         //40
float kd=50;                       //10
                              

int motorResponse;
float correction;

int incomingByte;

int digitalReading[sensorNum];
int leftSpeed,rightSpeed;
int pidAllWhite=0 ;
int leftIR,rightIR,frontIR;
int buzzer=12,buzzing=0;

int contiWhite=10,stopCounter=0;


//sonar shit

const int lSonarTrig = 7;
const int lSonarEcho = 8;
int lDistance;
int caveMin=140,caveMax=180;
int lWallDistance=25;

int prev,curr,diff;

int time=4;


void setup()
{
  //initialize IR pins
  for(int i = 0; i < sensorNum; i++)
  {
    pinMode(A0 + i, INPUT);
  }

  //initialize motor pins
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT); 
  pinMode(motorPin3, OUTPUT);
  pinMode(motorPin4, OUTPUT);
  pinMode(buzzer,    OUTPUT);
  
  delay(1000);
    Serial.begin(38400);
calibration();
  

}



void loop()
{
sensorMapping();

// send data only when you receive data:
if (Serial.available() > 0) 
{
        // read the incoming byte:
        incomingByte = Serial.read();

        // say what you got:
        Serial.print("I received: ");
        Serial.println(incomingByte, DEC);
}  
 
//sensorRead();


Serial.println(mappedValue);
Serial.println();
Serial.print(leftSpeed);
Serial.print(" ");
Serial.print(rightSpeed);
Serial.println(stopCounter);

/*// For IR Reading
for(int i = 0; i < sensorNum; i++)


{
  //Serial.print(analogRead(A0+i));
  if(analogRead(A0+i)<blackLimit[i])
 Serial.print("B");
 else Serial.print("W");
 Serial.print(" ");
 Serial.print(analogRead(A0+i));
 Serial.print(" && ");
 Serial.print(blackLimit[i]);
 Serial.print(" ||| ");
}
Serial.println();
Serial.print(leftIR);
Serial.print(" & ");
Serial.println(rightIR);
*/


  while(stopCounter>contiWhite)
  {
    if(buzzing<3500)
  {
    digitalWrite(buzzer, HIGH);
  }  else digitalWrite(buzzer, LOW);
  
  brake();
  buzzing++;

  sensorMapping();
  }
  buzzing=0;

digitalWrite(buzzer,LOW);
  
 if(mappedValue!=111)
   {  
      pid();
      motor(leftSpeed,rightSpeed);
   }
   
 else 
  { 
    
    motor(-10,-10);
    delay(1);
    sonarReading();
 
    if(lDistance<30)
    {
      brake();
      delay(50);
      for(int i=0;i<3;i++)
        {
        
        sonarReading();

        if(lDistance>=30)
        break;
        }
      if(lDistance<30)
      {
        while(mappedValue==111)
        {
        curveWallRun();
        sensorMapping();
        }
      }
      
    }
    
    else
    {
      if (leftIR==1 && rightIR==0) 
      {{while(digitalReading[3]!=0) 
      {plannedCRotate();  
      sensorMapping();
      } 
      pid(); 
      motor(leftSpeed,rightSpeed);
      } rightIR=1;}
      
      else if (leftIR==0 && rightIR==1) 
      {{while(digitalReading[5]!=0) 
      {plannedACRotate();  
      sensorMapping();
      } 
      pid(); 
      motor(leftSpeed,rightSpeed);
      } leftIR=1;}
      
      else {
      while(mappedValue==111){
      motor(maxSpeed,maxSpeed);
      sensorMapping();
      }
      }
    }
 }
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
  else 
  {
    rightSpeed=maxSpeed+ motorResponse;
    leftSpeed=maxSpeed;
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


void brake()
{
  analogWrite(motorPin1, 0);
  analogWrite(motorPin2, 0);
  analogWrite(motorPin3, 0);
  analogWrite(motorPin4, 0);

}

void plannedACRotate()
{
  analogWrite(motorPin1,rotationSpeed);
  analogWrite(motorPin2, 0);
  analogWrite(motorPin3, 0);
  analogWrite(motorPin4,rotationSpeed);

}

void plannedCRotate()
{
  analogWrite(motorPin1,0);
  analogWrite(motorPin2, rotationSpeed);
  analogWrite(motorPin3, rotationSpeed);
  analogWrite(motorPin4,0);

}

//sonar functions
long mstocm(long microseconds)
{
 
  return (microseconds*346.3)/2/10000;
}

int trigger(int trigPin,int echopin)
{
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  int distance =  mstocm(pulseIn(echopin, HIGH));
  return distance;
}
void sonarReading()
{
  lDistance=trigger(lSonarTrig,lSonarEcho);
 if(lDistance==0)
  {
    sonarReading();
  }
}
void curveWallRun()
{  
    if(lDistance>lWallDistance)
    {
      motor(caveMax,caveMin);
     
      
    } 
    else 
    {
      plannedCRotate();
    }

     sonarReading();
     sensorMapping();
    
}


void sensorMapping()
{
int sum=0,pidDetect=0,stopDetect = 0; 
 
 for (int i = 0; i <sensorNum; i++)
  { 
    
    if (analogRead(A0+i) > blackLimit[i])           
     { 

      if(i>1 && i<7) {sum += i*2; pidDetect++;}
      stopDetect++;
      digitalReading[i]= 0;
    } else digitalReading[i]= 1;
    

  }
   if(pidDetect!=0){  
  mappedValue = sum / pidDetect;
  
   }
   else mappedValue=111;



if (digitalReading[1]==0 || digitalReading[7]==0 ) 
{leftIR=digitalReading[1]; rightIR=digitalReading[7]; }


if(stopDetect>=sensorNum-1)
{
stopCounter++;

} else 
{
stopCounter=0;
}

}


//auto calibration 
void calibration()
{
  plannedCRotate();
  float upSum = 0,lowSum = 0;
  int sensorArray[sensorNum][2];

  for(int i = 0; i < sensorNum; i++)
    {
      sensorArray[i][0] = analogRead(A0+i);
      sensorArray[i][1] = analogRead(A0+i);
    }
 

  int loopCounter = (int)(time * 1000 / 2.5);  
  while(loopCounter)
  {
    for(int i = 0; i < sensorNum; i++)
    {
      if(analogRead(A0+i)<sensorArray[i][0]) sensorArray[i][0]=analogRead(A0+i);
      if(analogRead(A0+i)>sensorArray[i][1]) sensorArray[i][1]=analogRead(A0+i);
    }
  loopCounter--;

  }

 for(int i=0; i < sensorNum; i++)
  blackLimit[i] = (int)(sensorArray[i][0] + safety * (sensorArray[i][1] - sensorArray[i][0]));
prev = millis();
sensorMapping ();
curr= millis();
diff = curr - prev;

  brake();
  delay(1000);

}

