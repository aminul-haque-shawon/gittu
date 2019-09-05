#define sensorNum 8
#define maxSpeed 250
#define rotationSpeed 180

int blackLimit[sensorNum];
//int blackLimit[sensorNum] = {500,500,500,500,500,500,500,500}; //set blackLimit here
int digitalReading[sensorNum];

const int motorPin1=6,motorPin2=5;        //right motor
const int motorPin3=11,motorPin4=10;       //left motor


float error, prevError=0;

float mappedValue, targetValue =7;     //changed ferom 4.5 to 9
float safety=0.35;

float kp=40;                             //45 IF DOESN'T WORK
float kd=50;

int leftIR = 0, rightIR = 0;

int frontTrig = 4, frontEcho = 3;
int rightTrig = 9, rightEcho = 8;
int frontDistance = -1, rightDistance = -1;

int sonarTimer=0;
int sonarActive=0;
int obstacleKey=0;
int fDistance=0;

int linegapCount;
int motorResponse;
float correction;

int leftSpeed,rightSpeed;


int time=5;
int lastBlack;
int i;



void setup()
{

    pinMode(motorPin1, OUTPUT);
    pinMode(motorPin2, OUTPUT);
    pinMode(motorPin3, OUTPUT);
    pinMode(motorPin4, OUTPUT);
    pinMode(frontTrig, OUTPUT);
    pinMode(frontEcho, INPUT);
    pinMode(rightTrig, OUTPUT);
    pinMode(rightEcho, INPUT);

    Serial.begin(9600);
    calibration();
    
}







void loop()
{

     

    sensorMapping();


    for(int i=0; i<8; i++ )
     {
         Serial.print(digitalReading[i]);
         Serial.print(" ");


     }
     Serial.println();
    
    //Reset for acute
    if(digitalReading[0]==0 && digitalReading[7]==0){
          linegapCount++;
          if(linegapCount>15){
              
              rightIR = 0;
              leftIR = 0;
            }
          }
     else
      linegapCount=0;


    
//    if(obstacleKey==0){
//    frontDistance= triggerSonars(frontTrig, frontEcho);
//    //1Serial.println(fDistance);
//       if(frontDistance<15 && frontDistance>5){
//        brake();
//        delay(50);
//        frontDistance= trigger(frontTrig, frontEcho);
//        if(frontDistance<15 && frontDistance>5){
//            obstacleKey=1;
//            Run1();
//            Run2();
//          }
//       }
//    }
    if(mappedValue!=100)  //if bot is not fully on white
    {
        pid();
        motor(leftSpeed,rightSpeed);
    }
    
    
    
    //Acute turn
    else
    {

        if(leftIR == 1 && rightIR ==0)
        {

            plannedACRotate();

            while(digitalReading[3] == 0 && digitalReading[4] == 0 )
            {
                sensorMapping();

            }

            leftIR = 0;
        }
        else if(leftIR == 0 && rightIR == 1 )
        {
            plannedCRotate();

            while(digitalReading[3] == 0&&digitalReading[4] == 0)
            {
                sensorMapping();

            }

            rightIR = 0;

        }
        else 
        {
            goForward();
            
if(obstacleKey==0)
{
    frontDistance= triggerSonars(frontTrig, frontEcho);
    //Serial.println(fDistance);
    if(frontDistance<15 && frontDistance>5)
    {
        brake();
        delay(50);
        frontDistance= trigger(frontTrig, frontEcho);
        if(frontDistance<15 && frontDistance>5)
        {
            obstacleKey=1;
            goForward();
            delay(100);
            brake();
            Run1();
            wallFollow();

        }
    }
    sensorMapping();


    rightIR = 0;
    leftIR = 0;
}        

        }



    Serial.println();

}
}

void sensorMapping()
{
    int sum=0,count=0;

    for (int i = 0; i <sensorNum; i++)
    {

        if (analogRead(i) < blackLimit[i])
        {
            digitalReading[i] = 1;

            if(i > 0 && i < 7)
            {
            sum += i*2;
            count++;
            }
        }
        else
            digitalReading[i] = 0;

    }
    if(count!=0)
    {
        mappedValue = sum / count;
    }
    else
        mappedValue=100;

    if(digitalReading[1] || digitalReading[6])
    {
        leftIR = digitalReading[1];
        rightIR = digitalReading[6];
    }

}


void pid()
{

    error=targetValue-mappedValue;
    correction=(kp*error)+(kd*(error-prevError));
    prevError=error;
    motorResponse=(int)correction;

    if(motorResponse>maxSpeed)
        motorResponse=maxSpeed;

    if(motorResponse<-maxSpeed)
        motorResponse=-maxSpeed;

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

void plannedCRotate()
{
    analogWrite(motorPin1,0);
    analogWrite(motorPin2, rotationSpeed);
    analogWrite(motorPin3, rotationSpeed);
    analogWrite(motorPin4,0);

}

void plannedACRotate()
{
    analogWrite(motorPin1,rotationSpeed);
    analogWrite(motorPin2, 0);
    analogWrite(motorPin3, 0);
    analogWrite(motorPin4,rotationSpeed);

}

void brake(void)
{
    analogWrite(motorPin1, 0);
    analogWrite(motorPin2, 0);
    analogWrite(motorPin3, 0);
    analogWrite(motorPin4, 0);
}

void goForward(void)
{
    analogWrite(motorPin1,rotationSpeed );
    analogWrite(motorPin2, 0);
    analogWrite(motorPin3, rotationSpeed);
    analogWrite(motorPin4, 0);


}
void goBack(void)
{
    analogWrite(motorPin1,0 );
    analogWrite(motorPin2, rotationSpeed);
    analogWrite(motorPin3, 0);
    analogWrite(motorPin4, rotationSpeed);


}
void calibration()
{
    plannedCRotate();
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
            if(analogRead(A0+i)<sensorArray[i][0])
                sensorArray[i][0]=analogRead(A0+i);
            if(analogRead(A0+i)>sensorArray[i][1])
                sensorArray[i][1]=analogRead(A0+i);
        }
        loopCounter--;
    }

    for(int i=0; i < sensorNum; i++)
        blackLimit[i] = (int)(sensorArray[i][0] + safety * (sensorArray[i][1] - sensorArray[i][0]));

    brake();
    delay(2000);
}

long mstocm(long microseconds)
{
 return (microseconds*346.3)/2/10000;
}

long trigger(int trigPin,int echoPin)
{
  unsigned long oldTime;
  unsigned long tiMe;                //changed time to tiMe
  unsigned long timeLimit=2000;

  digitalWrite(echoPin,LOW);     
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  
  while(digitalRead(echoPin)==LOW); 
  
  oldTime=micros();
  while(digitalRead(echoPin)==HIGH&&(micros()-oldTime<timeLimit));
  tiMe=micros()-oldTime;

  
  long distance =  mstocm(tiMe);

//  Serial.print(" trigger Time: ");
//  Serial.println(distance);
  
  return distance;
}

int triggerSonars(int trigg,int echo)
{
//  Serial.println("trigger start");
  int sonarDelay=12;  //use 50 if 50 works fine
  


  
  if(millis()-sonarTimer < sonarDelay)
   delay(sonarDelay-millis()+sonarTimer);

  int sonarDistance=trigger(trigg,echo);

  
  sonarTimer=millis();
  return sonarDistance;
//  Serial.println("trigger end");  
}

void Run1(){
   
      plannedACRotateSlow();
      delay(410);
      brake();
      delay(400);
}

void wallFollow(){
      int distance2;
      distance2 = triggerSonars(rightTrig, rightEcho);
    
    while(distance2 <32){
    if(distance2 > 12){
    
        plannedCRotateSlow();
    }
    else if(distance2 < 10){
        plannedACRotateSlow();
    }
    else
        goForward();

    delay(10);
    distance2 = triggerSonars(rightTrig, rightEcho);
}
  goForward();
  while(mappedValue==100){
      sensorMapping();
  }
}
void plannedACRotateSlow(){
    analogWrite(motorPin1,120);
    analogWrite(motorPin2, 0);
    analogWrite(motorPin3, 0);
    analogWrite(motorPin4,120);
}
void plannedCRotateSlow(){
    analogWrite(motorPin1,0);
    analogWrite(motorPin2, 120);
    analogWrite(motorPin3, 120);
    analogWrite(motorPin4,0);
}
