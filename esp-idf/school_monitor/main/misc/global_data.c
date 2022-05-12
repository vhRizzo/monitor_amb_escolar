#include "global_data.h"

dados_t dados;

#ifdef INMP441_SENSOR
inmp441_t inmp_receive;
#endif
#ifdef DSM501A_SENSOR
dsm501a_t dsm_receive;
#endif
#ifdef BME280_SENSOR
bme280_t bme_receive;
#endif
#ifdef NEO6M_SENSOR
neo6m_t neo_receive;
#endif

esp_mqtt_client_handle_t client;

void rcv_data_task ( void *pvParameters )
{
    mqtt_app_start();

    dados.poeira_pm_10 = -1;
    dados.poeira_pm_25 = -1;
    dados.temperatura = -1;
    dados.umidade = -1;
    dados.pressao = -1;
    dados.ruido = -1;
    dados.coord[0] = -1;
    dados.coord[1] = -1;

    esp_timer_create_args_t data_timer_args = {
        .callback = &send_data_timer,
        .name = "data_timer"
    };

    esp_timer_handle_t data_timer;
    ESP_ERROR_CHECK(esp_timer_create(&data_timer_args, &data_timer));

    esp_timer_create_args_t startup_timer_args = {
        .callback = &startup_timer_func,
        .arg = (void *) data_timer,
        .name = "startup_timer"
    };

    esp_timer_handle_t startup_timer;
    ESP_ERROR_CHECK(esp_timer_create(&startup_timer_args, &startup_timer));
    ESP_ERROR_CHECK(esp_timer_start_once(startup_timer, TEMPO_STARTUP * 1000000));

    while (true) {
        #ifdef INMP441_SENSOR
        if (xQueueReceive(inmp_queue, &inmp_receive, portMAX_DELAY)) 
        {
            dados.ruido = inmp_receive.ruido;
        }
        #endif

        #ifdef DSM501A_SENSOR
        if (xQueueReceive(dsm_queue, &dsm_receive, portMAX_DELAY))
        {
            dados.poeira_pm_10 = dsm_receive.poeira_pm_10;
            dados.poeira_pm_25 = dsm_receive.poeira_pm_25;
        }
        #endif

        #ifdef BME280_SENSOR
        if (xQueueReceive(bme_queue, &bme_receive, portMAX_DELAY))
        {
            dados.temperatura = bme_receive.temperatura;
            dados.umidade = bme_receive.umidade;
            dados.pressao = bme_receive.pressao;
        }
        #endif

        #ifdef NEO6M_SENSOR
        if (xQueueReceive(neo_queue, &neo_receive, portMAX_DELAY))
        {
            dados.coord[0] = neo_receive.coord[0];
            dados.coord[1] = neo_receive.coord[1];
        }
        #endif
    }
}

void startup_timer_func (void *arg)
{
    ESP_ERROR_CHECK(esp_timer_start_periodic((esp_timer_handle_t)arg, TEMPO_ANALISE * 1000000));
}

void send_data_timer (void *arg)
{
    ESP_LOGI(__func__, "BME280(%.2f*C, %.2f%%, %dPa), DSM501a(%.2fpcs/0.01cf, %.2fpcs/0.01cf), INMP441(%.2fdB), NEO-6M(%.6f, %.6f).", 
                dados.temperatura, dados.umidade, dados.pressao, 
                dados.poeira_pm_10, dados.poeira_pm_25, 
                dados.ruido, 
                dados.coord[0], dados.coord[1]);
            
    char tmp[21];
    sprintf(tmp, "%.2f", dados.temperatura);
    esp_mqtt_client_publish(client, BME_TEMP_TOPIC, tmp, 0, 0, 0);
    sprintf(tmp, "%.2f", dados.umidade);
    esp_mqtt_client_publish(client, BME_UMID_TOPIC, tmp, 0, 0, 0);
    sprintf(tmp, "%d", dados.pressao);
    esp_mqtt_client_publish(client, BME_PRES_TOPIC, tmp, 0, 0, 0);
    sprintf(tmp, "%.2f", dados.poeira_pm_10);
    esp_mqtt_client_publish(client, DSM_PM10_TOPIC, tmp, 0, 0, 0);
    sprintf(tmp, "%.2f", dados.poeira_pm_25);
    esp_mqtt_client_publish(client, DSM_PM25_TOPIC, tmp, 0, 0, 0);
    sprintf(tmp, "%.2f", dados.ruido);
    esp_mqtt_client_publish(client, _INMP_DB_TOPIC, tmp, 0, 0, 0);
    sprintf(tmp, "%.6f", dados.coord[0]);
    esp_mqtt_client_publish(client, _NEO_LAT_TOPIC, tmp, 0, 0, 0);
    sprintf(tmp, "%.6f", dados.coord[1]);
    esp_mqtt_client_publish(client, _NEO_LNG_TOPIC, tmp, 0, 0, 0);
}

void mqtt_app_start(void)
{
    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set("MQTT_CLIENT", ESP_LOG_VERBOSE);
    esp_log_level_set("MQTT_EXAMPLE", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT_BASE", ESP_LOG_VERBOSE);
    esp_log_level_set("esp-tls", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT", ESP_LOG_VERBOSE);
    esp_log_level_set("OUTBOX", ESP_LOG_VERBOSE);

    esp_mqtt_client_config_t mqtt_cfg = {
        .uri = BROKER_URL,
        .task_prio = configMAX_PRIORITIES - 3,
        .task_stack = 4096,
    };

    client = esp_mqtt_client_init(&mqtt_cfg);
    /* The last argument may be used to pass data to the event handler, in this example mqtt_event_handler */
    esp_mqtt_client_register_event(client, (esp_mqtt_event_id_t)ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(client);
}
