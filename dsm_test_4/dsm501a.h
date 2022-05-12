#include "esphome.h"
#define DSM501a_pin_vermelho 17 //DSM501A Vout2 sensibilidade em 1 micrometro
#define DSM501a_pin_amarelo 36  //DSM501A Vout1 sensibilidade em 2.5 micrometro, se pino Controle (preto) aberto

namespace esphome {

  unsigned int tempo_analise = 60000; //ms
  float ratioV2 = 0;
  float ratioV1 = 0;
  float concentrationV2 = 0;
  float concentrationV1 = 0;

  volatile unsigned long lowpulseoccupancyV2 = 0;
  volatile unsigned long lowpulseoccupancyV1 = 0;

  volatile unsigned int estadoV2 = HIGH;
  volatile unsigned int estadoV1 = HIGH;

  volatile unsigned long marca_fallingV2 = 0;
  volatile unsigned long marca_risingV2 = 0;

  volatile unsigned long marca_fallingV1 = 0;
  volatile unsigned long marca_risingV1 = 0;

  class DSM501a : public PollingComponent, public Sensor {
  public:
    DSM501a() : PollingComponent (tempo_analise) {}

    static void change_interrupcaoV2();
    static void change_interrupcaoV1();

    float get_setup_priority() const override { return esphome::setup_priority::DATA; }

    void setup () override {
      pinMode(DSM501a_pin_vermelho, INPUT);
      pinMode(DSM501a_pin_amarelo, INPUT);

      attachInterrupt(digitalPinToInterrupt(DSM501a_pin_vermelho), DSM501a::change_interrupcaoV2, CHANGE);
      attachInterrupt(digitalPinToInterrupt(DSM501a_pin_amarelo), DSM501a::change_interrupcaoV1, CHANGE);
    }

    void update () override {
      detachInterrupt(digitalPinToInterrupt(DSM501a_pin_vermelho));
      detachInterrupt(digitalPinToInterrupt(DSM501a_pin_amarelo));
      ratioV2 = lowpulseoccupancyV2 / (tempo_analise * 10); //otimizado, pois e o mesmo que (lowpulseoccupancyV2 * 10 ) / ((endtime - starttime) * 100); // o *10 para estar em ms
      ratioV1 = lowpulseoccupancyV1 / (tempo_analise * 10);
      concentrationV2 = 1.1 * pow(ratioV2, 3) - 3.8 * pow(ratioV2, 2) + 520 * ratioV2 + 0.62; // using spec sheet curve
      concentrationV1 = 1.1 * pow(ratioV1, 3) - 3.8 * pow(ratioV1, 2) + 520 * ratioV1 + 0.62; // using spec sheet curve

      publish_state(concentrationV2);
      publish_state(concentrationV1);

      attachInterrupt(digitalPinToInterrupt(DSM501a_pin_vermelho), DSM501a::change_interrupcaoV2, CHANGE);
      attachInterrupt(digitalPinToInterrupt(DSM501a_pin_amarelo), DSM501a::change_interrupcaoV1, CHANGE);
      lowpulseoccupancyV2 = 0;
      lowpulseoccupancyV1 = 0;
    }
  };

  void ICACHE_RAM_ATTR HOT DSM501a::change_interrupcaoV2()
  {
    if (digitalRead(DSM501a_pin_vermelho)==LOW && estadoV2==HIGH)
    {
      marca_fallingV2 = micros();
      estadoV2=LOW;
    }
    else if(digitalRead(DSM501a_pin_vermelho)!=LOW && estadoV2==LOW)
    {
      marca_risingV2 = micros();
      estadoV2=HIGH;
      lowpulseoccupancyV2 += (marca_risingV2 - marca_fallingV2);
    }
  }

  void ICACHE_RAM_ATTR HOT DSM501a::change_interrupcaoV1()
  {
    if(digitalRead(DSM501a_pin_amarelo)==LOW && estadoV1==HIGH)
    {
      marca_fallingV1 = micros();
      estadoV1=LOW;
    }
    else if(digitalRead(DSM501a_pin_amarelo)==HIGH && estadoV1==LOW)
    {
      marca_risingV1 = micros();
      estadoV1=HIGH;
      lowpulseoccupancyV1 += (marca_risingV1 - marca_fallingV1);
    }
  }

}
