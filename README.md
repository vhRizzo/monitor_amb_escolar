# Monitoramento de Ambiente Escolar

Neste repositório estão todos os códigos-fonte desenvolvidos no decorrer do projeto de monitoramento de ambiente escolar.

O projeto consiste em utilizar diversos sensores de baixo custo para monitorar as condições
do ambiente de certas escolas municipais de Apucarana - PR.

Sensores utilizados:
  - BME280 - Temperatura, umidade e pressão.
  - CCS811 - CO2 e TVOC (descartado por funcinamento insatisfatório).
  - DSM501A - Material Particulado com sensibilidade de 1 μm e 2,5 μm.
  - NEO-6M - Latitude e longitude.
  - INMP441 - Intensidade de ruído.

Para o controle dos sensores, foi-se utilizado o microcontrolador ESP32, versões WiFi LoRa (V2) inicialmente, e ESP32S posteriormente.
