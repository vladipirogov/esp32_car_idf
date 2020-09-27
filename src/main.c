#include "freertos/FreeRTOS.h"
#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_event_loop.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "mqtt_client.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "servo_control.h"
#include "common.h"
#include "camera_controller.h"
#include "car_controller.h"
#include "ultrasonic.h"
#include "speed_sensor.h"
#include "driver/pcnt.h"
#include "esp_sleep.h"

static esp_mqtt_client_handle_t client = NULL;
static char *TAG = "MQTTS_ESP";
static EventGroupHandle_t mqtt_event_group = NULL;
static EventGroupHandle_t wifi_event_group = NULL;
static char topic[100];
static char data[4];
static uint32_t sensor_array[MAX_SENSORS];
//Difinition array of topicks to be send by mqtt
const char *topics[] = {ULTRASONIC_MQTT_TOPIC, OPTICAL_1_MQTT_TOPIC, OPTICAL_2_MQTT_TOPIC};

//Sensors queue
static QueueHandle_t sensorQueue;

/** */
void wifi_event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    switch (event_id) {
        case WIFI_EVENT_STA_START:
            esp_wifi_connect();
            break;
        case WIFI_EVENT_STA_DISCONNECTED:
            esp_wifi_connect();
            xEventGroupClearBits(wifi_event_group, BIT0);
            break;
        default:
            break;
    }
    return;
}

/**
 *
 */
void ip_event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    switch (event_id) {
        case IP_EVENT_STA_GOT_IP:
            xEventGroupSetBits(wifi_event_group, BIT0);

            break;
        default:
            break;
    }
    return;
}

/** */
void parseCommand(command_t *command) {
    int length = strlen(command->topic);
    char topic[length];
    strcpy(topic, command->topic);
    char *array[length + 2];
    int i = 0;
    array[i] = strtok(topic, "/");
    while (array[i] != NULL) {
        array[++i] = strtok(NULL, "/");
    }

    if (!strcmp(array[1], CAMERA_HANDLE)) handle_camera(command);
    else if (!strcmp(array[1], CAR_HANDLE)) handle_car(command);
}


/** */
esp_err_t mqtt_event_handler(esp_mqtt_event_handle_t event)
{
    esp_mqtt_client_handle_t client = event->client;
    int msg_id;
    // your_context_t *context = event.context;
    switch (event->event_id) {
        case MQTT_EVENT_CONNECTED:
        	 xEventGroupSetBits(mqtt_event_group, BIT1);
            ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
            msg_id = esp_mqtt_client_subscribe(client, "esp32/car/#", 0);
            msg_id = esp_mqtt_client_subscribe(client, "esp32/camera/#", 0);
            ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);
            break;

        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
            break;

        case MQTT_EVENT_SUBSCRIBED:
            ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
            msg_id = esp_mqtt_client_publish(client, "esp32/status/activ", "1", 0, 0, 1);
            ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);
            break;

        case MQTT_EVENT_UNSUBSCRIBED:
            ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
            break;

        case MQTT_EVENT_PUBLISHED:
            ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
            break;

        case MQTT_EVENT_DATA:
            ESP_LOGI(TAG, "MQTT_EVENT_DATA");
            printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
            printf("DATA=%.*s\r\n", event->data_len, event->data);
            memset(topic, 0, strlen(topic));
            memset(data, 0, strlen(data));
            strncpy(topic, event->topic, event->topic_len);
            strncpy(data, event->data, event->data_len);
            command_t command = {
                .topic = topic,
                .message = data,
            };
            parseCommand(&command);
            break;

        case MQTT_EVENT_ERROR:
            ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
            break;

        default:
            break;
    }
    return ESP_OK;
}

/** */
void mqtt_app_start(EventGroupHandle_t event_group)
{
	mqtt_event_group = event_group;
    const esp_mqtt_client_config_t mqtt_cfg = {
        .uri = "mqtt://192.168.1.107:1883",    //mqtt://mqtt.eclipse.org:1883
        .event_handle =  mqtt_event_handler,
		.keepalive = 10,
        .lwt_topic = "esp32/status/activ",
		.lwt_msg = "0",
        .lwt_retain = 1,
    };

    ESP_LOGI(TAG, "[APP] Free memory: %d bytes", esp_get_free_heap_size());
    client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_start(client);
}

/*
Method to print the reason by which ESP32
has been awaken from sleep
*/
void print_wakeup_reason(){
  esp_sleep_wakeup_cause_t wakeup_reason;

  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch(wakeup_reason)
  {
    case ESP_SLEEP_WAKEUP_EXT0 : printf("Wakeup caused by external signal using RTC_IO\n"); break;
    case ESP_SLEEP_WAKEUP_EXT1 : printf("Wakeup caused by external signal using RTC_CNTL\n"); break;
    case ESP_SLEEP_WAKEUP_TIMER : printf("Wakeup caused by timer\n"); break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD : printf("Wakeup caused by touchpad\n"); break;
    case ESP_SLEEP_WAKEUP_ULP : printf("Wakeup caused by ULP program\n"); break;
    default : printf("Wakeup was not caused by deep sleep: %d\n",wakeup_reason); break;
  }
}

/** */
void ultrasonicTask(void *parameters)
{
  sensor_data_t data;
  portBASE_TYPE xStatus;
  const uint32_t MAX_DISTANCE_CM = 500; // 5m max
  uint32_t distance = 0;
  ultrasonic_sensor_t sensor;
        sensor.trigger_pin = TRIGGER_PIN;
		sensor.echo_pin = ECHO_PIN;

    ultrasonic_init(&sensor);

  while (true) {
    esp_err_t res = ultrasonic_measure_cm(&sensor, MAX_DISTANCE_CM, &distance);
    if (res != ESP_OK)
        {
            printf("Error: ");
            switch (res)
            {
                case ESP_ERR_ULTRASONIC_PING:
                    printf("Cannot ping (device is in invalid state)\n");
                    break;
                case ESP_ERR_ULTRASONIC_PING_TIMEOUT:
                    printf("Ping timeout (no device found)\n");
                    break;
                case ESP_ERR_ULTRASONIC_ECHO_TIMEOUT:
                    printf("Echo timeout (i.e. distance too big)\n");
                    break;
                default:
                    printf("%d\n", res);
            }
        }
        else {
            data.sensor = ULTRASONIC_SENSOR;
            data.value = distance;
            sensor_array[ULTRASONIC_SENSOR] = distance;
            //Sending data to sensors queue
            xStatus = xQueueSend(sensorQueue, (void *)&data, 0);
            if( xStatus != pdPASS ) {
            printf("Could not send to ultrasonic to the queue.\r\n");
            }
            taskYIELD();
        }
    vTaskDelay(100 / portTICK_PERIOD_MS);
    }
  }

/** */
void sensorProcessTask(void *parameters)
{
  sensor_data_t data;

  while (true) {
    if (xQueueReceive(sensorQueue, &data, portMAX_DELAY) == pdPASS) handle_sensor(&data);
  }
  vTaskDelete(NULL);
}

/** */
void periodicTask(void *parameters)
{
  char topic_buff[MAX_SENSORS][4];
  for (int i = 0; i < NUMBER_OF_TOPICS_TO_SEND; i++) memset(topic_buff[i], 0, 4);

  while (true) {
    if(!gpio_get_level(WAKEUP_PIN)) {
         printf("Going to sleep now\n");
        esp_deep_sleep_start();
    }

    for (int i = 0; i < NUMBER_OF_TOPICS_TO_SEND; i++)
    {
      itoa(sensor_array[i], topic_buff[i], 10);
      esp_mqtt_client_publish(client, topics[i], topic_buff[i], 0,0,0);
    }
    vTaskDelay(2000 / portTICK_PERIOD_MS);
  }
}

/** */
void pulseTask(void *parameters) {
  sensor_data_t data_1;
  sensor_data_t data_2;
  data_1.sensor = OPTICAL_SENSOR_1;
  data_2.sensor = OPTICAL_SENSOR_2;
  portBASE_TYPE xStatus;

  speed_sensor_params_t params_1 = {
      .delay = 100,
      .pin = ENCODER_1_PIN,
      .ctrl_pin = GPIO_NUM_0,
      .channel = PCNT_CHANNEL_0,
      .unit = PCNT_UNIT_0,
      .count = 0,
  };
    ESP_ERROR_CHECK(init_speed_sensor(&params_1));

    speed_sensor_params_t params_2 = {
      .delay = 100,
      .pin = ENCODER_2_PIN,
      .ctrl_pin = GPIO_NUM_1,
      .channel = PCNT_CHANNEL_0,
      .unit = PCNT_UNIT_1,
      .count = 0,
  };
    ESP_ERROR_CHECK(init_speed_sensor(&params_2));

    while(true) {
        data_1.value = calculateRpm(&params_1);
        data_2.value = calculateRpm(&params_2);
        sensor_array[OPTICAL_SENSOR_1] = data_1.value;
        sensor_array[OPTICAL_SENSOR_2] = data_2.value;
        printf("speed 1 = %d\n", data_1.value);
        printf("speed 2 = %d\n", data_2.value);
        xStatus = xQueueSend(sensorQueue, (void *)&data_1, 0);
        xStatus = xQueueSend(sensorQueue, (void *)&data_2, 0);
        if( xStatus != pdPASS ) {
        printf("Could not send optical to the queue.\r\n");
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);
}
}

/** */
void app_main(void)
{
    nvs_flash_init();
    tcpip_adapter_init();
    wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &ip_event_handler, NULL));
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );
    wifi_config_t sta_config = {
        .sta = {
            .ssid = CONFIG_ESP_WIFI_SSID,
            .password = CONFIG_ESP_WIFI_PASSWORD,
            .bssid_set = false
        }
    };
    ESP_ERROR_CHECK( esp_wifi_set_config(WIFI_IF_STA, &sta_config) );
    ESP_LOGI(TAG, "start the WIFI SSID:[%s] password:[%s]", CONFIG_ESP_WIFI_SSID, "******");
    ESP_ERROR_CHECK( esp_wifi_start() );
    ESP_LOGI(TAG, "Waiting for wifi");
    xEventGroupWaitBits(wifi_event_group, BIT0, false, true, portMAX_DELAY);

    //MQTT init
    mqtt_event_group = xEventGroupCreate();
    mqtt_app_start(mqtt_event_group);

    //Camera control init
    init_camera();
    //Car control init
    init_car();

    gpio_set_direction(WAKEUP_PIN, GPIO_MODE_INPUT);
    esp_sleep_enable_ext0_wakeup(WAKEUP_PIN,1); //1 = High, 0 = Low

    //creation sensors queue
    sensorQueue = xQueueCreate(4, sizeof(sensor_data_t) );

    /* we create a new task here */
  xTaskCreate(
      ultrasonicTask,   /* Task function. */
      "ultrasonicTask", /* name of task. */
      10000,                /* Stack size of task */
      NULL,                 /* parameter of the task */
      1,                    /* priority of the task */
      NULL);                /* Task handle to keep track of created task */

/* we create a new task here */
  xTaskCreate(
      sensorProcessTask,   /* Task function. */
      "sensorProcessTask", /* name of task. */
      10000,                /* Stack size of task */
      NULL,                 /* parameter of the task */
      2,                    /* priority of the task */
      NULL);                /* Task handle to keep track of created task */

/* we create a new task here */
  xTaskCreate(
      periodicTask,   /* Task function. */
      "periodicTask", /* name of task. */
      10000,                /* Stack size of task */
      NULL,                 /* parameter of the task */
      1,                    /* priority of the task */
      NULL);                /* Task handle to keep track of created task */

/* we create a new task here */
  xTaskCreate(
      pulseTask,   /* Task function. */
      "pulseTask", /* name of task. */
      10000,                /* Stack size of task */
      NULL,                 /* parameter of the task */
      1,                    /* priority of the task */
      NULL);                /* Task handle to keep track of created task */

}

