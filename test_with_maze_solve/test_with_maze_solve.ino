//things to change each bot: motorpins, kp, kd, maxSpeed, rotationSpeed,turnDelay,buzzer,contiBlack,contiWhite;
#define sensorNum 8
#define maxSpeed 150
#define rotationSpeed 120

int blackLimit[sensorNum];

const int motorPin1 = 3, motorPin2 = 4;        //right motor
const int motorPin3 = 7,motorPin4 = 6;       //left motor

float error, prevError=0;
float mappedValue, targetValue = 7;     

float safety=0.35;

float kp=45;                         //40
float kd=50;                       //10
                              

int motorResponse;
float correction;

int flag = 0;
int mazeSolved = 0;

int nodeDirection[500]; //(where bot went while mapping, will be opposite while coming back) 1 for right; 0 for front; -1 for left;
int nodeReturn[500];

int currentNode=-1;

int forwardDelay=150, returnDelay=250;
int buzzer=12;

int digitalReading[sensorNum];
int leftSpeed,rightSpeed;
int pidAllWhite=0,turnAllWhite=0,allWhite=0,fSpeed=150 , brakeDelay=200;
int leftIR,rightIR,frontIR;
int contiBlack=140,resetIRCounter=0;
int contiWhite=1000,mappingEndCounter=0,stopCounter=0;

int incomingByte;

float time=5;

int prev, curr, diff;


void setup()
{


  //initialize motor pins
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT); 
  pinMode(motorPin3, OUTPUT);
  pinMode(motorPin4, OUTPUT);

  pinMode(buzzer,OUTPUT);

  
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


if(mazeSolved == 1)
{
  currentNode = -1;
  mazeSolve();
}




//return from wrong path

if(mappedValue== 111)
{

   currentNode++;
   nodeDirection[currentNode]= 2;
   Serial.println(nodeDirection[currentNode]);
   
   motor(fSpeed,fSpeed);
   delay(returnDelay);
   
   while(digitalReading[3]!=1 && digitalReading[4]!=1)
   {
      plannedACRotate();
      sensorMapping();
   }      
}
    //nodeDetection

if(digitalReading[0]==1 && digitalReading[7]==1)
{
  Serial.print("AllBlackFound");
  
  Serial.println();
  motor(fSpeed,fSpeed);
  delay(forwardDelay);
  brake();
  delay(brakeDelay);
  sensorMapping();
  if (mappedValue==111)
  {
    
    Serial.print("LR ");
    
    currentNode++;
    nodeDirection[currentNode]= -1;
    
    Serial.println(nodeDirection[currentNode]);
    
    printValues();
    Serial.println();
    while(mappedValue!=targetValue)
    {
    plannedACRotate();
    sensorMapping();
    } 

    
  } 
  else
  {
    if(digitalReading[0]==0 && digitalReading[7]==0 )
    {
      Serial.print("LFR ");
      currentNode++;
      nodeDirection[currentNode]=-1;
      
      Serial.println(nodeDirection[currentNode]);
    
      printValues();
      Serial.println();
      while(digitalReading[3]==1 || digitalReading[4]==1 )
      {
      plannedACRotate();
      sensorMapping();
      } 
      while(digitalReading[3]!=1 || digitalReading[4]!=1)
      {
      plannedACRotate();
      sensorMapping();
      }
       
    }
    else 
    {
      Serial.println("End");
      brake();
      currentNode++;
      nodeDirection[currentNode]=100;
      Serial.println(nodeDirection[currentNode]);

      for(int z=0;z<=currentNode;z++)
      {
        Serial.print(nodeDirection[z]);
        Serial.print(" ");
      }
      Serial.println("Calculation start");
      Serial.println();
      mazeCalculate();
      Serial.println("Calculation done");
      
      while(digitalReading[4]!=0)
      {
        plannedACRotate();
        sensorMapping();
      }
      while(digitalReading[2]!=1 || digitalReading[3]!=1)
      {
        plannedACRotate();
        sensorMapping();
      }

      brake();
      delay(500);    
      mazeSolved = 1;  
    }

  }
} 
else if(digitalReading[0]==1 && digitalReading[1]==1 &&  digitalReading[7]==0 )
{
  motor(fSpeed,fSpeed);
  delay(forwardDelay);
  brake();
  delay(brakeDelay);
  sensorMapping();
  if (mappedValue==111)
  {
    Serial.print("LeftTurn ");
    printValues();
    Serial.println();
    while(digitalReading[3]!=1 || digitalReading[4]!=1)
    {
    plannedACRotate();
    sensorMapping();
    } 

    
  } 
  else
  {
    Serial.print("LF ");
    currentNode++;
    nodeDirection[currentNode]=-1;
    
    Serial.println(nodeDirection[currentNode]);
    
    printValues();
    Serial.println();
    leftIR=1;
    while(digitalReading[3]==1 || digitalReading[4]==1)
    {
    plannedACRotate();
    sensorMapping();
    } 
    while(digitalReading[3]!=1 || digitalReading[4]!=1)
    {
    plannedACRotate();
    sensorMapping();
    }
  }
} 
else if(digitalReading[0]==0 && digitalReading[6]==1 &&  digitalReading[7]==1 )
{
  motor(fSpeed,fSpeed);
  delay(forwardDelay);
  brake();
  delay(brakeDelay);
  sensorMapping();
  if (mappedValue==111)
  {
    Serial.print("RightTurn ");
    printValues();
    Serial.println();
    while(digitalReading[3]!=1 || digitalReading[4]!=1)
    {
    plannedCRotate();
    sensorMapping();
    } 

    
  } 
  else
  {
    
    Serial.print("FR ");
    currentNode++;
    nodeDirection[currentNode]=0;
  
    Serial.println(nodeDirection[currentNode]);
    
    
    printValues();
    Serial.println();
    rightIR=0;
    motor(maxSpeed,maxSpeed);
    sensorMapping();
  }
}

else
{  
  pid();
  motor(leftSpeed,rightSpeed);
}
   
}
  

  
void mazeCalculate()
{
  int nodes = currentNode;
  do
  {
    for(int i = 0; i<nodes; i++)
    {
        if(nodeDirection[i] == 2)
        {
              nodeDirection[i-1]= nodeDirection[i-1] + nodeDirection[i] +nodeDirection[i+1];

              for(int j=i; j<nodes; j++)
              {
                  nodeDirection[j] = nodeDirection[j+2];
              }

              nodes-=2;
              i-=2;
        }

    }

    flag = 0;

    for(int i = 0; i<nodes; i++)
    {
       if(nodeDirection[i] == 2)
          flag = 1;
    }
  } while(flag == 1);


    for(int i = 0; i <= nodes-1; i++)
   {
    nodeReturn[i] = -nodeDirection[nodes-i-1];
   }
   nodeReturn[nodes] = 100;
   
   int k=0;
    Serial.print("Initial: ");
    do{
        Serial.print(nodeReturn[k]);
        Serial.print("  ");
        k++;
    }while(nodeReturn[k-1] != 100);
    

}



void mazeSolve()
{
  sensorMapping();
  if(digitalReading[0]==1 && digitalReading[7]==1)
{
  Serial.print("AllBlackFound");
  
  Serial.println();
  motor(fSpeed,fSpeed);
  delay(forwardDelay);
  brake();
  delay(brakeDelay);
  sensorMapping();
  if (mappedValue==111)
  {
    
    Serial.print("LR ");
    
    currentNode++;
    mazeSolvingDecision(nodeReturn[currentNode]);
    Serial.println(nodeReturn[currentNode]);
      
  } 
  else
  {
    if(digitalReading[0]==0 && digitalReading[7]==0 )
    {
      Serial.print("LFR ");

    currentNode++;
    mazeSolvingDecision(nodeReturn[currentNode]);
    Serial.println(nodeReturn[currentNode]);
    }
    else 
    {
      Serial.println("End");
      brake();
      delay(5000);
    }

  }
} 
else if(digitalReading[0]==1 && digitalReading[1]==1 &&  digitalReading[7]==0 )
{
  motor(fSpeed,fSpeed);
  delay(forwardDelay);
  brake();
  delay(brakeDelay);
  sensorMapping();
  if (mappedValue==111)
  {
    Serial.print("LeftTurn ");
    printValues();
    Serial.println();
    while(digitalReading[3]!=1 || digitalReading[4]!=1)
    {
    plannedACRotate();
    sensorMapping();
    } 

    
  } 
  else
  {
    Serial.print("LF ");
        
    currentNode++;
    mazeSolvingDecision(nodeReturn[currentNode]);
    Serial.println(nodeReturn[currentNode]);
  }  
} 
else if(digitalReading[0]==0 && digitalReading[6]==1 &&  digitalReading[7]==1 )
{
  motor(fSpeed,fSpeed);
  delay(forwardDelay);
  brake();
  delay(brakeDelay);
  sensorMapping();
  if (mappedValue==111)
  {
    Serial.print("RightTurn ");
    printValues();
    Serial.println();
    while(digitalReading[3]!=1 || digitalReading[4]!=1)
    {
    plannedCRotate();
    sensorMapping();
    } 

    
  } 
  else
  {
    
    Serial.print("FR ");
        
    currentNode++;
    mazeSolvingDecision(nodeReturn[currentNode]);
    Serial.println(nodeReturn[currentNode]);
    
  }
}

else
{  
  pid();
  motor(leftSpeed,rightSpeed);
}




mazeSolve();
   
}


void mazeSolvingDecision (int path)
{
 sensorMapping();
 if(path == 1)
 {
    Serial.println("Will go right");

    if(mappedValue != 111)
    {
    
      rightIR = 1;
      while(digitalReading[3]==1 || digitalReading[4]==1)
      {
      plannedCRotate();
      sensorMapping();
      } 
      while(digitalReading[3]!=1 || digitalReading[4]!=1)
      {
      plannedCRotate();
      sensorMapping();
      }

    }

    else
    {
      while(mappedValue!=targetValue)
      {
      plannedCRotate();
      sensorMapping();
      } 
    }
 }

 else if(path == -1)
 { 
    if(mappedValue != 111)
    {
      Serial.println("Will go left");
      leftIR=1;
      while(digitalReading[3]==1 || digitalReading[4]==1)
      {
      plannedACRotate();
      sensorMapping();
      } 
      while(digitalReading[3]!=1 || digitalReading[4]!=1)
      {
      plannedACRotate();
      sensorMapping();
      }
    }

     else
    {
      while(mappedValue!=targetValue)
      {
      plannedACRotate();
      sensorMapping();
      } 
    }
 }

 else
 {
  Serial.println("Will go forward");
  motor(maxSpeed,maxSpeed);
  sensorMapping();
 }


int k=currentNode+1;
Serial.println("Remaining: ");
do{
    Serial.print(nodeReturn[k]);
    Serial.print("  ");
    k++;
}while(nodeReturn[k-1] != 100);
    
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



void brake(void)
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

void sensorMapping()
{
int sum=0, pidDetect=0, mappingEndDetect=0; 
 
 for (int i = 0; i <sensorNum; i++)
  { 
    
    if (analogRead(A0+i) < blackLimit[i])           
     { 

      if(i>0 && i<sensorNum-1){sum += i*2; pidDetect++;}
      mappingEndDetect++;
      digitalReading[i]= 1;
    } else digitalReading[i]= 0;
    

  }
   if(pidDetect!=0 ){  
  mappedValue = sum / pidDetect;
   }

   else mappedValue=111;

if(digitalReading[0]==1 || digitalReading[7]==1)
{
  leftIR=digitalReading[0];
  rightIR=digitalReading[7];
  resetIRCounter=0;
} 

if(digitalReading[0]==0 && digitalReading[sensorNum-1]==0)
{
 resetIRCounter++;
 if(resetIRCounter>contiBlack)
 {
 leftIR=0;
 rightIR=0;
 }
}

if(mappingEndDetect==sensorNum)
{
mappingEndCounter++;

} else {mappingEndCounter=0;}

}

void printValues()
{
  Serial.print(mappedValue);
  Serial.println();
  
  
  
  for(int i = 0; i < sensorNum; i++)
  {
  //Serial.print(analogRead(A0+i));
  if(analogRead(A0+i)<blackLimit[i])
   Serial.print("1");
   else Serial.print("0");
   Serial.print(" ");
  } 
Serial.println();

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
