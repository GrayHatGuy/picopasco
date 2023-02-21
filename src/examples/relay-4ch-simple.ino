const int n = 4; //number of relays
const int ctl[n] = {6,7,20,21}; // define n pins 
const int trigger = LOW;// init trigger state
int loopDelay = 1000;// delay in loop
int state[n] = {0,0,0,0};
int dwell[n] = {500,1200,2000,3000}; // dwell at trigger HIGH state per relay
int tmpStat =1;
const int ledPin =  LED_BUILTIN; //LED settings
int ledState = LOW;          
unsigned long previousMillis = 0;        
const long LED_interval = 80; 

void setup() {
  pinMode(ledPin, OUTPUT);
  for(int i=0; i < n; i++)
  {
    pinMode(ctl[i], OUTPUT);// set pin as output
    if(trigger ==LOW){ //set trigger init state
      digitalWrite(ctl[i], HIGH); 
    }else{
       digitalWrite(ctl[i], LOW);   
    }
  }
  Serial.begin(9600);
  
}

void loop() {
  for(int i=0; i < n; i++)
  {
   digitalWrite(ctl[i], HIGH);
   state[i] = 1;
   ledState = HIGH;
   digitalWrite(ledPin, ledState);
   for (int j=0; j < n; j++)
   {
   Serial.print ("S"); Serial.print (j); Serial.print (","); Serial.print(state[j]); Serial.print(",");Serial.print (dwell[j]);
   if (j < n )
   {
    Serial.print (",");
   }
   }
   Serial.println ();
   delay(dwell[i]);
   digitalWrite(ctl[i], LOW);
   state[i] = 0;
   ledState = LOW;
   digitalWrite(ledPin, ledState);
  }
         
}
