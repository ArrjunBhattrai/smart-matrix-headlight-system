// IC1 Address Pins
const int addrPins1[] = {3, 4, 5, 6};
const int strobePin1 = 7;

// IC2 Address Pins
const int addrPins2[] = {8, 9, 10, 11};
const int strobePin2 = 12;

char lastCommand = ' ';

  int ledsIC1[16] = {0};
  int ledsIC2[16] = {0};
  int countIC1 = 0, countIC2 = 0;

void setup() {
  Serial.begin(9600);

  for (int i = 0; i < 4; i++) {
    pinMode(addrPins1[i], OUTPUT);
    pinMode(addrPins2[i], OUTPUT);
  }

  pinMode(strobePin1, OUTPUT);
  pinMode(strobePin2, OUTPUT);

  digitalWrite(strobePin1, LOW);
  digitalWrite(strobePin2, LOW);

  for (int i = 0; i < 16; i++) {
        ledsIC1[i] = i;
        ledsIC2[i] = i;
      }
      countIC1 = countIC2 = 16;
      
      glowBothICs(ledsIC1, countIC1, ledsIC2, countIC2);

}

void loop() {
  if (Serial.available() > 0) {
    char ch = Serial.read();
    if (ch >= 'A' && ch <= 'J') {
      lastCommand = ch;
    } else {
      lastCommand = 'Z'; // Light up all LEDs if invalid command
    }
    Serial.print("Received: ");
    Serial.println(lastCommand);
  }

  getLEDsForChar(lastCommand, ledsIC1, countIC1, ledsIC2, countIC2);

  // Always use fast glow
  glowBothICs(ledsIC1, countIC1, ledsIC2, countIC2);
}

// Get LED patterns for characters
void getLEDsForChar(char ch, int ledsIC1[], int &countIC1, int ledsIC2[], int &countIC2) {
  switch (ch) {
    case 'A': {
      int ic1[] = {4, 5, 6, 7, 8, 9, 10, 11};
      int ic2[] = {0, 1, 2, 3, 12, 13, 14};
      setArrays(ledsIC1, countIC1, ic1, sizeof(ic1)/sizeof(int));
      setArrays(ledsIC2, countIC2, ic2, sizeof(ic2)/sizeof(int));
      break;
    }
    case 'B': {
      int ic1[] = {6, 7, 8, 9, 10, 11};
      int ic2[] = {0, 1, 2, 3, 4, 5, 8, 9, 12, 13, 14, 15};
      setArrays(ledsIC1, countIC1, ic1, sizeof(ic1)/sizeof(int));
      setArrays(ledsIC2, countIC2, ic2, sizeof(ic2)/sizeof(int));
      break;
    }
    case 'C': {
      int ic2[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15};
      countIC1 = 0;
      setArrays(ledsIC2, countIC2, ic2, 16);
      break;
    }
    case 'D': {
      int ic1[] = {1, 3, 4, 5, 6, 7, 8, 9, 10, 11, 14, 15};
      int ic2[] = {0, 2, 12, 13};
      setArrays(ledsIC1, countIC1, ic1, sizeof(ic1)/sizeof(int));
      setArrays(ledsIC2, countIC2, ic2, sizeof(ic2)/sizeof(int));
      break;
    }
    case 'E': {
      for (int i = 0; i < 16; i++) ledsIC1[i] = i;
      countIC1 = 16;
      countIC2 = 0;
      break;
    }
    case 'F': {
      int ic1[] = {3, 4, 5, 6, 7, 8, 9, 10, 11, 14};
      int ic2[] = {0, 1, 2, 3, 4, 9, 12, 13, 14, 15};
      setArrays(ledsIC1, countIC1, ic1, sizeof(ic1)/sizeof(int));
      setArrays(ledsIC2, countIC2, ic2, sizeof(ic2)/sizeof(int));
      break;
    }
    case 'G': {
      int ic1[] = {4, 5, 6, 7, 8, 9, 10, 11};
      int ic2[] = {0, 1, 2, 3, 4, 5, 8, 9, 12, 13, 14, 15};
      setArrays(ledsIC1, countIC1, ic1, sizeof(ic1)/sizeof(int));
      setArrays(ledsIC2, countIC2, ic2, sizeof(ic2)/sizeof(int));
      break;
    }
    case 'H': {
      int ic1[] = {6, 7, 8, 9, 10, 11};
      for (int i = 0; i < 16; i++) ledsIC2[i] = i;
      countIC2 = 16;
      setArrays(ledsIC1, countIC1, ic1, sizeof(ic1)/sizeof(int));
      break;
    }
    case 'I': {
      int ic1[] = {1, 3, 4, 5, 6, 7, 8, 9, 10, 11, 14, 15};
      int ic2[] = {0, 1, 2, 3, 12, 13, 14, 15};
      setArrays(ledsIC1, countIC1, ic1, sizeof(ic1)/sizeof(int));
      setArrays(ledsIC2, countIC2, ic2, sizeof(ic2)/sizeof(int));
      break;
    }
    case 'J': {
      int ic2[] = {0, 2, 8, 12, 13};
      for (int i = 0; i < 16; i++) ledsIC1[i] = i;
      countIC1 = 16;
      setArrays(ledsIC2, countIC2, ic2, sizeof(ic2)/sizeof(int));
      break;
    }
    case 'K': {
      int ic1[] = {1, 3, 4, 5, 6, 7, 8, 9, 10, 11, 14, 15};
      int ic2[] = {0, 1, 2, 3, 4, 5, 8, 9, 12, 13, 14, 15};
      setArrays(ledsIC1, countIC1, ic1, sizeof(ic1)/sizeof(int));
      setArrays(ledsIC2, countIC2, ic2, sizeof(ic2)/sizeof(int));
      break;
    }
    case 'L': {
      int ic1[] = {3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 14};
      int ic2[] = {0, 1, 2, 3, 4, 5, 6, 8, 9, 11, 12, 13, 14, 15};
      setArrays(ledsIC1, countIC1, ic1, sizeof(ic1)/sizeof(int));
      setArrays(ledsIC2, countIC2, ic2, sizeof(ic2)/sizeof(int));
      break;
    }
    case 'M': {
      int ic1[] = {4, 5, 6, 7, 8, 9, 10, 11};
      for (int i = 0; i < 16; i++) ledsIC2[i] = i;
      countIC2 = 16;
      setArrays(ledsIC1, countIC1, ic1, sizeof(ic1)/sizeof(int));
      break;
    }
    case 'N': {
      int ic1[] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 14};
      int ic2[] = {0, 1, 2, 3, 4, 9, 12, 13, 14, 15};
      setArrays(ledsIC1, countIC1, ic1, sizeof(ic1)/sizeof(int));
      setArrays(ledsIC2, countIC2, ic2, sizeof(ic2)/sizeof(int));
      break;
    }
    case 'O': {
      for (int i = 0; i < 16; i++) ledsIC1[i] = i;
      countIC1 = 16;
      int ic2[] = {0, 1, 2, 3, 12, 13, 14, 15};
      setArrays(ledsIC2, countIC2, ic2, sizeof(ic2)/sizeof(int));
      break;
    }

    case 'Z': { // For invalid characters
      for (int i = 0; i < 16; i++) {
        ledsIC1[i] = i;
        ledsIC2[i] = i;
      }
      countIC1 = countIC2 = 16;
      break;
    }
  }
}

// Copy LED pattern array
void setArrays(int target[], int &count, const int source[], int len) {
  for (int i = 0; i < len; i++) {
    target[i] = source[i];
  }
  count = len;
}

// Light up selected LEDs for both decoders
void glowBothICs(int ic1Leds[], int ic1Count, int ic2Leds[], int ic2Count) {
  for (int i = 0; i < max(ic1Count, ic2Count); i++) {
    if (i < ic1Count) {
      setAddress(addrPins1, ic1Leds[i]);
      digitalWrite(strobePin1, HIGH);
      delayMicroseconds(400);
      digitalWrite(strobePin1, LOW);
    }

    if (i < ic2Count) {
      setAddress(addrPins2, ic2Leds[i]);
      digitalWrite(strobePin2, HIGH);
      delayMicroseconds(400);
      digitalWrite(strobePin2, LOW);
    }
  }
}

// Set address lines for decoder
void setAddress(const int addrPins[], int value) {
  for (int i = 0; i < 4; i++) {
    digitalWrite(addrPins[i], (value >> i) & 1);
  }
}
