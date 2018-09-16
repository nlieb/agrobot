


void setup() {
  // put your setup code here, to run once:

  Serial.begin(9600);

  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(8, OUTPUT);
  pinMode(9, OUTPUT);

  Serial.println("Start");

  
}

void readString(char* buf, int len) {
  int i = 0;
  while(i < len - 1) {
    if (Serial.available() > 0) {
      char c = Serial.read();
      if (c == '\r') break;

      buf[i] = c;
      i++;
    }
  }

  // Adding the null terminator
  buf[i] = '\0';
}

void loop() {
  
  char pin[128];
  readString(pin, 128);
  
  char value[128];
  readString(value, 128);

  Serial.println(pin);
  Serial.println(value);

  int  p, v;
  p = atoi(pin);
  v = atoi(value);
  

  if(p == 3 || p == 5 || p == 6 || p ==9 )
  {
    analogWrite(p, v);
  } 
  else 
  {
   digitalWrite(p, v);  
  }

}
