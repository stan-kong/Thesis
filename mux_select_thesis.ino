/******** Pin Definitions ********/

// MUX 1 (right side of the board)
const uint8_t MUX1_A0 = 10;
const uint8_t MUX1_A1 = 11;
const uint8_t MUX1_A2 = 12;
const uint8_t MUX1_A3 = 13;
const uint8_t MUX1_EN = 8;

// MUX 2 (left side of the board)
const uint8_t MUX2_A0 = 4;
const uint8_t MUX2_A1 = 5;
const uint8_t MUX2_A2 = 6;
const uint8_t MUX2_A3 = 7;
const uint8_t MUX2_EN = 2;

/******** Channel Codes ********/

// MUX 1 electrodes
const uint8_t E1_M1  = 0b0000;
const uint8_t E5_M1  = 0b0001;
const uint8_t E3_M1  = 0b0010;
const uint8_t E7_M1  = 0b0011;
const uint8_t E9_M1  = 0b0100;
const uint8_t E13_M1 = 0b0101;
const uint8_t E15_M1 = 0b0110;
const uint8_t E21_M1 = 0b0111;
const uint8_t E23_M1 = 0b1100;
const uint8_t E14_M1 = 0b1101;
const uint8_t E22_M1 = 0b1110;
const uint8_t E20_M1 = 0b1111;

// MUX 2 electrodes
const uint8_t E0_M2  = 0b0000;
const uint8_t E4_M2  = 0b0001;
const uint8_t E2_M2  = 0b0010;
const uint8_t E6_M2  = 0b0011;
const uint8_t E8_M2  = 0b0100;
const uint8_t E12_M2 = 0b0101;
const uint8_t E10_M2 = 0b0110;
const uint8_t E18_M2 = 0b0111;
const uint8_t E19_M2 = 0b1100;
const uint8_t E16_M2 = 0b1101;
const uint8_t E17_M2 = 0b1110;
const uint8_t E11_M2 = 0b1111;

// Reference electrodes
// Assuming Ref1 and Ref2 are tied together on six-well plate, only one ref is chosen
const uint8_t REF_M1 = 0b1010;
const uint8_t REF_M2 = 0b1010;

struct PairStep {
  const char* leftName;
  uint8_t leftCode;
  const char* rightName;
  uint8_t rightCode;
};

/******** Step Sequence ********/

PairStep sequence[] = {
  {"Reference", REF_M1, "E0",         E0_M2},
  {"E1",        E1_M1,  "Reference",  REF_M2},
  {"Reference", REF_M1, "E2",         E2_M2},
  {"E3",        E3_M1,  "Reference",  REF_M2},
  {"Reference", REF_M1, "E4",         E4_M2},
  {"E5",        E5_M1,  "Reference",  REF_M2},
  {"Reference", REF_M1, "E6",         E6_M2},
  {"E7",        E7_M1,  "Reference",  REF_M2},
  {"Reference", REF_M1, "E8",         E8_M2},
  {"E9",        E9_M1,  "Reference",  REF_M2},
  {"Reference", REF_M1, "E10",        E10_M2},
  {"Reference", REF_M1, "E11",        E11_M2},
  {"Reference", REF_M1, "E12",        E12_M2},
  {"E13",       E13_M1, "Reference",  REF_M2},
  {"E14",       E14_M1, "Reference",  REF_M2},
  {"E15",       E15_M1, "Reference",  REF_M2},
  {"Reference", REF_M1, "E16",        E16_M2},
  {"Reference", REF_M1, "E17",        E17_M2},
  {"Reference", REF_M1, "E18",        E18_M2},
  {"Reference", REF_M1, "E19",        E19_M2},
  {"E20",       E20_M1, "Reference",  REF_M2},
  {"E21",       E21_M1, "Reference",  REF_M2},
  {"E22",       E22_M1, "Reference",  REF_M2},
  {"E23",       E23_M1, "Reference",  REF_M2}
};

const size_t NUM_STEPS = sizeof(sequence) / sizeof(sequence[0]);
size_t currentStep = 0;

void writeMuxCode(uint8_t code, uint8_t a0, uint8_t a1, uint8_t a2, uint8_t a3) {
  digitalWrite(a0, (code >> 0) & 0x01);
  digitalWrite(a1, (code >> 1) & 0x01);
  digitalWrite(a2, (code >> 2) & 0x01);
  digitalWrite(a3, (code >> 3) & 0x01);
}

void selectPair(const PairStep& step) {
  // Disable both first
  digitalWrite(MUX1_EN, LOW);
  digitalWrite(MUX2_EN, LOW);

  delayMicroseconds(2);

  // Update addresses
  writeMuxCode(step.leftCode,  MUX1_A0, MUX1_A1, MUX1_A2, MUX1_A3);
  writeMuxCode(step.rightCode, MUX2_A0, MUX2_A1, MUX2_A2, MUX2_A3);

  delayMicroseconds(2);

  // Enable both
  digitalWrite(MUX1_EN, HIGH);
  digitalWrite(MUX2_EN, HIGH);
}

void printCurrentStep(const PairStep& step, size_t idx) {
  Serial.print("Step ");
  Serial.print(idx + 1);
  Serial.print("/");
  Serial.print(NUM_STEPS);
  Serial.print(": ");
  Serial.print(step.leftName);
  Serial.print(" <-> ");
  Serial.println(step.rightName);
}

void doReset() {
  currentStep = 0;
  selectPair(sequence[0]);
  printCurrentStep(sequence[0], 0);
  currentStep = 1;
}

void doNext() {
  selectPair(sequence[currentStep]);
  printCurrentStep(sequence[currentStep], currentStep);
  currentStep = (currentStep + 1) % NUM_STEPS;
}

void setup() {
  Serial.begin(115200);

  pinMode(MUX1_A0, OUTPUT);
  pinMode(MUX1_A1, OUTPUT);
  pinMode(MUX1_A2, OUTPUT);
  pinMode(MUX1_A3, OUTPUT);

  pinMode(MUX2_A0, OUTPUT);
  pinMode(MUX2_A1, OUTPUT);
  pinMode(MUX2_A2, OUTPUT);
  pinMode(MUX2_A3, OUTPUT);

  pinMode(MUX1_EN, OUTPUT);
  pinMode(MUX2_EN, OUTPUT);

  digitalWrite(MUX1_EN, LOW);
  digitalWrite(MUX2_EN, LOW);

  doReset();
}

void loop() {
  while (Serial.available() > 0) {
    char cmd = Serial.read();

    if (cmd == 'r') {
      doReset();
    } else if (cmd == 'n') {
      doNext();
    }
  }
}