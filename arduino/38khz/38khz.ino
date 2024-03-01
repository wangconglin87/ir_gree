/*
 *  格力空调遥控器，型号：YAPOF或者是YAP0F
 *  编码格式为：引导码 + 35位（bit）数据码 + 短连接码 + 32位数据码 + 长连接码 + 引导码 + 35位数据码 + 短连接码 + 32位数据码 + 结束码
 *  引导码：9000us脉冲 + 4500us空闲
 *  短连接码：660us脉冲 + 20000us空闲
 *  长连接码：660us脉冲 + 40000us空闲
 *  结束码：660us脉冲
 *  数据0：660us脉冲 + 540us空闲
 *  数据1：660us脉冲 + 1640us空闲
 *  us为微秒
 */

const byte first[35] = { 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 1, 0 };
const byte second[32] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0 };
const byte third[35] = { 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 1, 0 };
const byte four[32] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0 };

// 引导码
#define AGC_MARK 9000
#define AGC_SPACE 4500

// 数字0
#define ZERO_MARK 660
#define ZERO_SPACE 540

// 数字1
#define ONE_MARK 660
#define ONE_SPACE 1640

// 短连接码 
#define CONN_MARK 660
// 没有使用，因为delayMicroseconds 最大值为16383，需要延时两次
#define CONN_SPACE 20000

// 长连接码
#define CONN_LONG_MARK 660
// 没有使用，因为delayMicroseconds 最大值为16383，需要延时四次
#define CONN_LONG_SPACE 40000

// 结束码
#define END_MARK 660

void setup() {
  TCCR1A = _BV(WGM11);              // PWM, Phase Correct, Top is ICR1
  TCCR1B = _BV(WGM13) | _BV(CS10);  // CS10 -> no prescaling

  TCNT1 = 0;
  // 16,000,000 / 38,000 / 2 = 210
  // Arduino UNO R3主频为16MHz，除以红外发送频率38KHz
  // 因为在PWM, Phase Correct模式下，频率减半，所以除2
  // 使用1/3的占空比，所以210 / 3 = 69
  ICR1 = 210;
  OCR1A = 69;
  // PB1在Arduino UNO R3中为9号针脚
  DDRB |= _BV(PB1);
}
void loop() {

  delay(100);

  sendLeader();

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

  sendLeader();

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

void enableSend() {
  TCNT1 = 0;
  TCCR1A |= _BV(COM1A1);
}

void disableSend() {
  TCCR1A &= ~(_BV(COM1A1));
}

void sendLeader() {
  enableSend();
  delayMicroseconds(AGC_MARK);
  disableSend();
  delayMicroseconds(AGC_SPACE);
}

void sendZero() {
  enableSend();
  delayMicroseconds(ZERO_MARK);
  disableSend();
  delayMicroseconds(ZERO_SPACE);
}

void sendOne() {
  enableSend();
  delayMicroseconds(ONE_MARK);
  disableSend();
  delayMicroseconds(ONE_SPACE);
}

void sendConnect() {
  enableSend();
  delayMicroseconds(CONN_MARK);
  disableSend();
  // delayMicroseconds 最大值为16383，所以延时两次
  delayMicroseconds(10000);
  delayMicroseconds(9800);
}

void sendConnectLong() {
  enableSend();
  delayMicroseconds(CONN_LONG_MARK);
  disableSend();
  delayMicroseconds(10000);
  delayMicroseconds(10000);
  delayMicroseconds(10000);
  delayMicroseconds(9500);
}

void sendEnd() {
  enableSend();
  delayMicroseconds(END_MARK);
  disableSend();
}
