/*
IDP
David Paterson
LED Module Test Code V1
*/
int led = 2;
void setup()
{
 pinMode(led, OUTPUT); //Set Pin3 as output
}
void loop()
{
 digitalWrite(led, HIGH); //Turn off led
 delay(2000);
 digitalWrite(led, LOW); //Turn on led
 delay(2000);
}