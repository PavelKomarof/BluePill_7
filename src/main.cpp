#include <Arduino.h>
// #include <SPI.h>
// #include <Wire.h> // Библиотека для работы с шиной I2C
// #include <Adafruit_ADS1X15.h> // Библиотека для работы с модулями ADS1115 и ADS1015

HardwareSerial Serial2(USART2); // PA3  (RX)  PA2  (TX)

// Adafruit_ADS1115 ads;  /* Use this for the 16-bit version */
// Adafruit_ADS1015 ads;     /* Use this for the 12-bit version */

// Pin connected to the ALERT/RDY signal for new sample notification.
// constexpr int READY_PIN = 3;

// This is required on ESP32 to put the ISR in IRAM. Define as
// empty for other platforms. Be careful - other platforms may have
// other requirements.
// #ifndef IRAM_ATTR
// #define IRAM_ATTR
// #endif

// volatile bool new_data = false;
// void IRAM_ATTR NewDataReadyISR() {
//   new_data = true;
// }

// TwoWire I2Cone = TwoWire (PB7,PB6);
uint8_t cnt = 0;

void setup()
{
  Serial.begin(115200);

  Serial2.begin(115200); // PA3  (RX)  PA2  (TX)

  // adsGain_t kGain=GAIN_ONE;
  // ads.setGain(kGain);

  // ads.setDataRate(RATE_ADS1115_860SPS);

  // if (!ads.begin(0x4A,&I2Cone))  Serial.println("Failed to initialize ADS.");

  /*

  pinMode(LED_BUILTIN,OUTPUT);
  Serial.begin(9600);

  Serial.println("Hello!");

  Serial.println("Getting differential reading from AIN0 (P) and AIN1 (N)");
  Serial.println("ADC Range: +/- 6.144V (1 bit = 3mV/ADS1015, 0.1875mV/ADS1115)");

  // The ADC input range (or gain) can be changed via the following
  // functions, but be careful never to exceed VDD +0.3V max, or to
  // exceed the upper and lower limits if you adjust the input range!
  // Setting these values incorrectly may destroy your ADC!
  //                                                                ADS1015  ADS1115
  //                                                                -------  -------
  // ads.setGain(GAIN_TWOTHIRDS);  // 2/3x gain +/- 6.144V  1 bit = 3mV      0.1875mV (default)
  // ads.setGain(GAIN_ONE);        // 1x gain   +/- 4.096V  1 bit = 2mV      0.125mV
  // ads.setGain(GAIN_TWO);        // 2x gain   +/- 2.048V  1 bit = 1mV      0.0625mV
  // ads.setGain(GAIN_FOUR);       // 4x gain   +/- 1.024V  1 bit = 0.5mV    0.03125mV
  // ads.setGain(GAIN_EIGHT);      // 8x gain   +/- 0.512V  1 bit = 0.25mV   0.015625mV
   ads.setGain(GAIN_SIXTEEN);    // 16x gain  +/- 0.256V  1 bit = 0.125mV  0.0078125mV

  if (!ads.begin()) {
    Serial.println("Failed to initialize ADS.");
    while (1);
  }

  //pinMode(READY_PIN, INPUT);
  // We get a falling edge every time a new sample is ready.
  //attachInterrupt(digitalPinToInterrupt(READY_PIN), NewDataReadyISR, FALLING);

  // Start continuous conversions.
  //ads.startADCReading(ADS1X15_REG_CONFIG_MUX_DIFF_0_1, continuous=true);
  ads.startADCReading(ADS1X15_REG_CONFIG_MUX_DIFF_0_1, true);


*/
}

// https://3d-diy.ru/wiki/arduino-moduli/modul-16-bitnogo-atsp-ads1115/
// http://langster1980.blogspot.com/2020/05/stm32-blue-pill-with-mpxv7002dp.html

char str[50];
char str1[50];
char str2[50];
float Fres1, Fres2;
// int16_t cnt=0;
/// Длина входного буфера для поиска начала посылки
#define BUF_LEN 20
/// Длина буфера для фильтра низких частот
#define SIG_BUF_LEN 30 // 15
uint8_t InBuf[21];
uint8_t BufWritePoint = 0;
uint8_t curB = 0, prevB = 0;
uint32_t Sample;
float X_Buf[SIG_BUF_LEN], Y_Buf[SIG_BUF_LEN]; //, M_Buf[SIG_BUF_LEN];
// float a0 = 1, a1 = 2, a2 = 1, k1 = 0.06746, b1 = 1.14298, b2 = -0.4128;  //100hz
// float a0 = 1, a1 = 2, a2 = 1, k1 = 0.04613, b1 = 1.30729, b2 = -0.49181;  //Hz
float a0 = 1, a1 = 2, a2 = 1, k1 = 0.00782, b1 = 1.73473, b2 = -0.76601; // 3
uint8_t HeartBeatCounter=0;
/// на вход новый Sample
uint8_t HR_detector(uint32_t Sample, uint8_t HeartBeatCounter)
{
  // Коэфициенты фильтра низких частот
  // https://xn--80atbdbsooh2gqb.xn--80aehb1bzc.xn--p1ai/digit-filters/df-lpf-2order.html
  // https://xn--80atbdbsooh2gqb.xn--80aehb1bzc.xn--p1ai/calc/calculator.html
  static float X_Buf[SIG_BUF_LEN], Y_Buf[SIG_BUF_LEN];
  /// предыдущий и текущий признак наличия макимума
  static uint8_t prevMaxFlag, curMaxFlag;
  float a0 = 1, a1 = 2, a2 = 1, k1 = 0.00782, b1 = 1.73473, b2 = -0.76601; // 3
  /// Проверка на выбросы и засветку.
  /// TODO проверить в реальных физических условиях и донастроить
  if (Sample < 50000)
  {
    // Продвинем буферы фильтра
    for (uint8_t i = SIG_BUF_LEN - 1; i > 0; i--)
    {
      X_Buf[i] = X_Buf[i - 1];
      Y_Buf[i] = Y_Buf[i - 1];
    }
    X_Buf[0] = Sample;
    Y_Buf[0] = a0 * X_Buf[0] + a1 * X_Buf[1] + a2 * X_Buf[2] + b1 * Y_Buf[1] + b2 * Y_Buf[2];
    // TODO попробовать  домножать на k1 сразу   попробовал  фигня получилась
    // TODO надо расширить  скользящее окно . А то округлые вершины пропускает.  Расширил стало лучше

    // 1 stage
    // Serial.println(Sample);

    float max = 0.0;
    /// Это для нового длинного буффера на 30 мест
    /// проба слева
    float sr1 = (Y_Buf[0] + Y_Buf[1] + Y_Buf[2] + Y_Buf[3] + Y_Buf[4]) / 5.;
    /// проба справа
    float sr2 = (Y_Buf[25] + Y_Buf[26] + Y_Buf[27] + Y_Buf[28] + Y_Buf[29]) / 5.;

    prevMaxFlag = curMaxFlag;
    /// максимальный посередение
    float max_element = 0;
    for (size_t i = 5; i < 25; i++)
    {
      if (max_element < Y_Buf[i])
        max_element = Y_Buf[i];
    }

    if (((max_element - sr1) > (5.0 / k1)) && ((Y_Buf[5] - sr2) > (5.0 / k1)))
      max = 1000.0;
    if (max == 0)
      curMaxFlag = 0;
    else
      curMaxFlag = 1;
    if (prevMaxFlag < curMaxFlag)
      HeartBeatCounter++;
    // 2 stage
    sprintf(str, "%u %u %u %u", Sample, (uint32_t)(k1 * Y_Buf[0]), (uint32_t)max,(uint8_t)HeartBeatCounter);
    // sprintf(str,"%u %u %u",Sample,(uint32_t) (k1*Y_Buf[0]),(uint32_t) sr1);
    // sprintf(str,"%u %u %u",Sample,(uint32_t) (Y_Buf[0]),(uint32_t) sr1);
    // sprintf(str,"%u %u %u %u %u %u",Sample,(uint32_t) (Y_Buf[0]),(uint32_t) sr1,(uint32_t) sr2,(uint32_t) Y_Buf[5], (uint32_t) max);
    Serial.println(str);
  }
  return HeartBeatCounter;
}

void loop()
{
  // Serial.print("Differential: ");
  // delay(1000);

  // int16_t results1 = ads.readADC_SingleEnded(0);
  // delay(10);
  // int16_t results2 = ads.readADC_SingleEnded(1);

  // Serial.print("Differential: "); Serial.print(results); Serial.print("("); Serial.print(ads.computeVolts(results)*1000,4); Serial.println("mV)");
  // Serial.print("Differential: "); Serial.print(results); Serial.print("("); Serial.print(ads.computeVolts(results),6); Serial.println("mV)");

  // Fres1=ads.computeVolts(results1);
  // Fres2=ads.computeVolts(results2);
  // //Serial.println(Fres,6);

  // //sprintf(str,"f=%f",Fres);
  // dtostrf(Fres1, 2, 6, str1);
  // dtostrf(Fres2, 2, 6, str2);
  /*
    for (int i=0; i<8; i++) {
      str[i]=str1[i];
      str[i+10]='a';
    }
    */

  // for (int i=0; i<20; i++) str[i]=' ';
  // for (int i=0; i<8; i++) {
  //   if(str1[i]=='.') str[i]=',';
  //   else str[i]=str1[i];
  //   if(str2[i]=='.') str[i+11]=',';
  //   else str[i+11]=str2[i];
  //   }
  // str[9]=';';

  // cnt=cnt+1;
  // Serial.print(cnt); Serial.print(" ; ");
  // Serial.println(str);
  // Serial.print(str);
  cnt++;
  // dtostrf(cnt, 2, 6, str1);
  // sprintf(str,"f=%u",cnt);
  // Serial.println(str1);
  // Serial.println(str);
  uint8_t NewByte;
  int res;
  do
  {
    res = Serial2.read();
    // Если приняли байт
    if (res != -1)
    {
      // Serial.print((char )res);
      prevB = curB;
      curB = res;
      // типа круговой входной буфер
      if (BufWritePoint == BUF_LEN)
        BufWritePoint = 0;
      // Запишем байт во входной буфер
      InBuf[BufWritePoint] = res;
      BufWritePoint++;
      // Если метка начала посылки 0x550x55
      if ((curB == 0x55) && (prevB == 0x55))
      {
        Sample = ((uint32_t)InBuf[0] << 16) | /* get raw red data */
                 ((uint32_t)InBuf[1] << 8) |  /* get raw red data */
                 ((uint32_t)InBuf[2] << 0);

        HeartBeatCounter= HR_detector( Sample,  HeartBeatCounter);
        // if (Sample < 50000)
        // {
        //   // SignalBuf
        //   for (uint8_t i = SIG_BUF_LEN - 1; i > 0; i--)
        //   {
        //     X_Buf[i] = X_Buf[i - 1];
        //     Y_Buf[i] = Y_Buf[i - 1];
        //   }
        //   X_Buf[0] = Sample;
        //   Y_Buf[0] = a0 * X_Buf[0] + a1 * X_Buf[1] + a2 * X_Buf[2] + b1 * Y_Buf[1] + b2 * Y_Buf[2];
        //   // попробовать  домножать на k1 сразу   попробовал  фигня получилась
        //   // надо расширить  скользящее окно . А то округлые вершины пропускает

        //   // 1
        //   // Serial.println(Sample);

        //   float max = 0.0;
        //   /// Это для старого коротококг буфера на 15 мест
        //   // float sr1=(Y_Buf[0]+Y_Buf[1]+Y_Buf[2]+Y_Buf[3]+Y_Buf[4])/5.;
        //   // float sr2=(Y_Buf[7]+Y_Buf[8]+Y_Buf[9]+Y_Buf[10]+Y_Buf[11])/5.;
        //   // if (((Y_Buf[5]-sr1)>(2.0/k1))&&((Y_Buf[5]-sr2)>(2.0/k1))) max=1000.0;
        //   /// Это для нового длинного буффера на 30 мест
        //   /// проба слева
        //   float sr1 = (Y_Buf[0] + Y_Buf[1] + Y_Buf[2] + Y_Buf[3] + Y_Buf[4]) / 5.;
        //   /// проба справа
        //   float sr2 = (Y_Buf[25] + Y_Buf[26] + Y_Buf[27] + Y_Buf[28] + Y_Buf[29]) / 5.;
        //   /// максимальный посередение
        //   float max_element = 0;
        //   for (size_t i = 5; i < 25; i++)
        //   {
        //     if (max_element < Y_Buf[i])
        //       max_element = Y_Buf[i];
        //   }

        //   if (((max_element - sr1) > (5.0 / k1)) && ((Y_Buf[5] - sr2) > (5.0 / k1)))
        //     max = 1000.0;

        //   // 2
        //   sprintf(str, "%u %u %u", Sample, (uint32_t)(k1 * Y_Buf[0]), (uint32_t)max);
        //   // sprintf(str,"%u %u %u",Sample,(uint32_t) (k1*Y_Buf[0]),(uint32_t) sr1);
        //   // sprintf(str,"%u %u %u",Sample,(uint32_t) (Y_Buf[0]),(uint32_t) sr1);
        //   // sprintf(str,"%u %u %u %u %u %u",Sample,(uint32_t) (Y_Buf[0]),(uint32_t) sr1,(uint32_t) sr2,(uint32_t) Y_Buf[5], (uint32_t) max);
        //   Serial.println(str);
        // }
        sprintf(str, "%X %X %X", InBuf[0], InBuf[1], InBuf[2]);
        // Serial.println(str);
        //  Поскольку обработали посылку теперь будем писать с начала входного буфера
        BufWritePoint = 0;
      }
    }

  } while (res > 0);

  /*
  digitalWrite(LED_BUILTIN, HIGH);
  delay(500);
  digitalWrite(LED_BUILTIN, LOW);
  delay(500);

  int16_t results = ads.getLastConversionResults();

  Serial.print("Differential: "); Serial.print(results); Serial.print("("); Serial.print(ads.computeVolts(results)*1000,4); Serial.println("mV)");
*/
}

// uart2_rcv_isr() {
//   receive_from_uart2();
// }

// timer_isr() {
//   mark_flag_for_uart1_transfer();
// }

// main() {
//    if(flag_for_uart2) {
//       reply_with_uart2();
//       unflag_uart2_task();
//    }

//    if(flag_for_timing_task) {
//       transfer_with_uart1();
//       clear_timing_task_flag();
//    }
// }
