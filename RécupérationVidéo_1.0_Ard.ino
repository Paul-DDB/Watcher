#define DIR_PIN 8
#define STEP_PIN 9
#define ENABLE_PIN 10

#define DATA_BUFF_SIZE 128
#define BUFFER_SIZE 7

int buff[BUFFER_SIZE];

const float anglePerStep = 1.8;

int32_t angle = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.println("Serial Start");
  
  pinMode(DIR_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(ENABLE_PIN, OUTPUT);

  int myTimeout = 100;
  Serial.setTimeout(myTimeout);
}

void loop() {
  // put your main code here, to run repeatedly:
  receiveSerial();
}

void receiveSerial()
{
  if (Serial.available())
  {
      String a;
      a = Serial.readString();
      int b = a.toInt();
      processData(b);
  }
}


void processData(int data)
{

  int32_t _command = data;
  int32_t _delta = angle - _command;
  int32_t _steps = abs(float(_delta) / anglePerStep);

  Serial.print("Command = ");
  Serial.print(_command);
  Serial.print(" Delta = ");
  Serial.print(_delta);
  Serial.print("  Steps:");
  Serial.println(_steps);
  
  digitalWrite(ENABLE_PIN, LOW);
  
  if (_delta < 0) {
    digitalWrite(DIR_PIN, HIGH);
    for (int i = 0; i <= _steps; i++) {
      digitalWrite(STEP_PIN, HIGH);
      delay(1);
      digitalWrite(STEP_PIN, LOW);
      delay(1);
    }
  }
  else if (_delta > 0) {
    digitalWrite(DIR_PIN, LOW);
    for (int i = 0; i <= _steps; i++) {
      digitalWrite(STEP_PIN, HIGH);
      delay(1);
      digitalWrite(STEP_PIN, LOW);
      delay(1);
    }
  }
  angle = _command;

}

int charToInt(char data[]) {
  int values[9];
  long returnValue = 0;
  int i = 0;
  int u = 0;
  bool negative = false;
  if (data[0] == 45) {
    i = 1;
    negative = true;
  }
  while (isDigit(data[i])) {
    values[u] = data[i] - 48;
    i++;
    u++;
  }
  for (int v = 0; v < u; v++) {
    returnValue += long(values[v]) * long(pow(10, u - v - 1));
  }
  if (negative) {
    returnValue = -returnValue;
  }
  return returnValue;
}



