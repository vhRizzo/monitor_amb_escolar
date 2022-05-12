# Códigos desenvolvidos através da plataforma ESPHome.

Este diretório possui todos os códigos desenvolvidos através da plataforma ESPHome para a geração do
firmware do projeto de monitoramento de ambiente escolar.

No diretório schoolMonitor está o código final utilizado, enquanto nos demais diretórios estão todos os códigos desenvolvidos
na tentativa de fazer o sensor DSM501A funcionar com a plataforma.

Testes realizados:
  - dsm_test_1 - Utilizado o código-fonte do protocolo [duty_cycle](https://github.com/esphome/esphome/tree/dev/esphome/components/duty_cycle) da ESPHome como base, com pequenas alterações para servir para o sensor DSM501A.
  - dsm_test_2 - Utilizado o código-fonte do protocolo [pulse_width](https://github.com/esphome/esphome/tree/dev/esphome/components/pulse_width) da ESPHome como base, com pequenas alterações para servir para o sensor DSM501A.
  - dsm_test_3 - Utilizado diretamente o protocolo [duty_cyle](https://esphome.io/components/sensor/duty_cycle.html) sem alterações no código-fonte, apenas algumas manipulações no arquivo de configuração para transformar o Duty Cycle em partículas por milhão.
  - dsm_test_4 - Tentativa de adicionar um novo sensor personalizado, utilizando [este código](https://create.arduino.cc/projecthub/mircemk/arduino-air-quality-monitor-with-dsm501a-sensor-b4f8fc) de Arduino como base.

Como nenhum teste funcionou corretamente, o sensor DSM501A foi removido do produto final.
