#define TdsSensorPin A1
#define VREF 5.0 // analog reference voltage(Volt) of the ADC
#define SCOUNT 30 // sum of sample point
int analogBuffer[SCOUNT]; // store the analog value in the array, read from ADC
int analogBufferTemp[SCOUNT];
int analogBufferIndex = 0,copyIndex = 0;
float averageVoltage = 0,tdsValue = 0,temperaturetds = 25;
void setup()
{
Serial.begin(115200);
pinMode(TdsSensorPin,INPUT);
}
void loop()
{
  
static unsigned long analogSampleTimepoint = millis();
if(millis()-analogSampleTimepoint > 40U) //every 40 milliseconds,read the analog value from the ADC
  {
  analogSampleTimepoint = millis();
  analogBuffer[analogBufferIndex] = analogRead(TdsSensorPin); //read the analog value and store into the buffer
  analogBufferIndex++;
  if(analogBufferIndex == SCOUNT)
  analogBufferIndex = 0;
  }

static unsigned long printTimepoint = millis();
if(millis()-printTimepoint > 800U)
  {
  printTimepoint = millis();
  for(copyIndex=0;copyIndex<SCOUNT;copyIndex++)
  analogBufferTemp[copyIndex]= analogBuffer[copyIndex];
//begin
{
int bTab[SCOUNT]; 
for (byte i = 0; i<SCOUNT; i++)
bTab[i] = analogBufferTemp[SCOUNT];
int i, j, bTemp;
for (j = 0; j < SCOUNT - 1; j++)
{
for (i = 0; i < SCOUNT - j - 1; i++)
{
if (bTab[i] > bTab[i + 1])
{
bTemp = bTab[i];
bTab[i] = bTab[i + 1];
bTab[i + 1] = bTemp;
}
}
}
if ((SCOUNT & 1) > 0)
bTemp = bTab[(SCOUNT - 1) / 2];
else
bTemp = (bTab[SCOUNT / 2] + bTab[SCOUNT / 2 - 1]) / 2;
}
// end 
  averageVoltage =  bTemp * (float)VREF/ 1024.0; // read the analog value more stable by the median filtering algorithm, and convert to voltage value
  float compensationCoefficient=1.0+0.02*(temperaturetds-25.0); //temperature compensation formula: fFinalResult(25^C) = fFinalResult(current)/(1.0+0.02*(fTP-25.0));
  float compensationVolatge=averageVoltage/compensationCoefficient; //temperature compensation
  tdsValue=(133.42*compensationVolatge*compensationVolatge*compensationVolatge - 255.86*compensationVolatge*compensationVolatge + 857.39*compensationVolatge)*0.5; //convert voltage value to tds value
  Serial.print("voltage:");
  Serial.print(averageVoltage,2);
  //Serial.print("V ");
  Serial.print("TDS Value:");
  Serial.print(tdsValue,0);
  //Serial.println("ppm");
  }

}