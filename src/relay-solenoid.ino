//user inputs
const int nrelays = 4; //user inputs number of relays for mix including H20 tank 
//long fill = 10000; //user inputs fill volume of tank switch 4 
//set relay inputs from pico gpio
//int p_sw1 = 21; 
//int p_sw2 = 20;
//int p_sw3 = 19;
//int p_sw4 = 18;

//static array for relays gpio pico relay trigger pins pull from ui
int sw[ 4 ] = {21, 20, 19, 18} ;
//min and max dwell times static [ms]
static long maxhigh = 60000;
static int maxlow = 250;

void setup() 
{
  Serial.begin(9600);
  //init pins
  for (int i = 0;(nrelays -1); i++ ) 
  {  
    pinMode(i+18, OUTPUT); //SW pins 18 to 21
  }
  Serial.println("relay init");
}
void loop() 
{     
//trigger pumps on and off using dwell    
  for (int j = 0;(nrelays -1); j++ ) 
  {
// *dev use only demo random on and off time add to j loop init*
 // dwell_rnd = random(maxlow, maxhigh/50);// limit max to 50% of high for demo
 // maxlow = dwell_rnd;
 // maxhigh = dwell_rnd;  
    Serial.print("S");
    Serial.print(j+1);
    Serial.println(" 0ff...");
    digitalWrite(sw[j], LOW);
    delay(maxlow);
    Serial.print("S");
    Serial.print(j+1);
    Serial.println(" ON...");
    digitalWrite(sw[j], HIGH);
    delay(maxhigh);
  }
}
