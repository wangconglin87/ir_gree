const byte first[35] = { 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 1, 0 };
const byte second[32] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 1 };
const byte third[35] = { 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 1, 0 };
const byte four[32] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 1 };

void setup() {
  Serial.begin(115200);

  TCCR1A = _BV(WGM11);              // PWM, Phase Correct, Top is ICR1
  TCCR1B = _BV(WGM13) | _BV(CS10);  // CS10 -> no prescaling

  // // Clear OC1A on Compare Match / Set OC1A at Bottom; Wave Form Generator: Fast PWM 14, Top = ICR1
  // TCCR1A = (1 << WGM11);
  // TCCR1B = (1 << WGM13) + (1 << WGM12) + (1 << CS10);  // prescaler = none;
  // ICR1 = 420;
  // OCR1A = 138;
  TCNT1 = 0;
  ICR1 = 210;
  OCR1A = 69;
  // DDRB |= (1 << PB1);
  DDRB |= _BV(PB1);
  //TCCR1A |= _BV(COM1A1);
}
void loop() {

  enableSend();
  delayMicroseconds(9000);
  disableSend();
  delayMicroseconds(4500);

  for (int i = 0; i < sizeof(first); i++) {
    if (0 == first[i]) {
      sendZero();
    } else {
      sendOne();
    }
  }

  sendConnect();

  for (int i = 0; i < sizeof(second); i++) {
    if (0 == second[i]) {
      sendZero();
    } else {
      sendOne();
    }
  }

  sendConnectLong();

  enableSend();
  delayMicroseconds(8980);
  disableSend();
  delayMicroseconds(4480);

  for (int i = 0; i < sizeof(first); i++) {
    if (0 == third[i]) {
      sendZero();
    } else {
      sendOne();
    }
  }

  sendConnect();

  for (int i = 0; i < sizeof(second); i++) {
    if (0 == four[i]) {
      sendZero();
    } else {
      sendOne();
    }
  }

  sendEnd();
  delay(5000);
}

void sendGree() {
  delay(50);

  // 引导码
  sendLeader();


  // 结束码
  sendEnd();
}

void enableSend() {
  TCNT1 = 0;
  TCCR1A |= _BV(COM1A1);
}

void disableSend() {
  TCCR1A &= ~(_BV(COM1A1));
}

void sendLeader() {
  enableSend();
  delayMicroseconds(9000);
  disableSend();
  delayMicroseconds(4500);
}

void sendZero() {
  enableSend();
  delayMicroseconds(650);
  disableSend();
  delayMicroseconds(550);
}

void sendOne() {
  enableSend();
  delayMicroseconds(650);
  disableSend();
  delayMicroseconds(1650);
}

void sendConnect() {
  enableSend();
  delayMicroseconds(650);
  disableSend();
  delayMicroseconds(10000);
  delayMicroseconds(10000);
}

void sendConnectLong() {
  enableSend();
  delayMicroseconds(650);
  disableSend();
  delayMicroseconds(10000);
  delayMicroseconds(10000);
  delayMicroseconds(10000);
  delayMicroseconds(10000);
}

void sendEnd() {
  enableSend();
  delayMicroseconds(650);
  disableSend();
}
