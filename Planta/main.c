 //Bibliotecas utilizadas:
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <stdbool.h>
#include <unistd.h>
#include <string.h>
#include <time.h>
#include "esp_wifi.h"
#include "esp_wifi_types.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "mqtt_client.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_event.h"
#include "esp_timer.h"
#include "sdkconfig.h"
#include "hal/adc_types.h"
#include "esp_adc/adc_oneshot.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "sdkconfig.h"
#include "soc/clk_tree_defs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#define POT ADC_CHANNEL_0							//Canal do ADC do potenciômetro
#define COOLER GPIO_NUM_25							//GPIO do PWM

#define ts 48.3										//tempo de amostragem, em ms

//Constantes-padrão do controlador:
#define Kp 30
#define Ki 20
#define Kd 0

#define kf 1										//constante do filtro

#define BUFF_LEN 1									//tamanho do buffer de envio

#define CONFIG_ESP_INT_WDT_TIMEOUT_MS 10000		//Configuração do timeout do watchdog
#define CONFIG_ESP_TASK_WDT_TIMEOUT_S 2000		//Configuração do timeout de tasks
#define CONFIG_ESP_TIMER_INTERRUPT_LEVEL 2		//Nível de prioridade da interrupção

#define PARALLEL_DATAS 3							//Número de sinais a serem enviados paralelalemnte

#define MIN_ANGLE 0								//Ângulo mínimo de operação do plate (0 graus)
#define MAX_ANGLE 93*M_PI/180.0					//Ângulo máximo do plate (limite mecânico)

#define TAMA 5										//Tempo de Acomodação em Malha Aberta (superdimensionado, aproximado, em segundos)



//Declaração e inicialização de variáveis:
float ref=0, angle=0, u=0, ueq=0, kp=0, ki=0, kd=0, sg=1, offset=0, ea=0, pop=0, uia=0;
int mode=0, valpot=0;
short id=0;
char topic[15], data[15];
adc_oneshot_unit_handle_t handle = NULL;
esp_mqtt_client_handle_t client = NULL;
bool start=false;

//Estrutura de dados para buffer:
typedef struct
{
    float buff1[PARALLEL_DATAS][BUFF_LEN];
    float buff2[PARALLEL_DATAS][BUFF_LEN];
    bool b1f;
    bool b2f;
} Buffer;
Buffer buff;

//Função para envio de buffer via MQTT:
void send_data(float buffer[PARALLEL_DATAS][BUFF_LEN])
{
	char aux[12], send[12*PARALLEL_DATAS*BUFF_LEN];
	send[0] = '\0';
	for(int i=0; i<PARALLEL_DATAS; i++)
	{
		for(int j=0; j<BUFF_LEN; j++)
		{
			aux[0] = '\0';
			sprintf(aux, "%.3f", buffer[i][j]);
			strcat(send, aux);
			if(j<(BUFF_LEN-1))
			{
				strcat(send, ",");
			}
		}
		if(i<(PARALLEL_DATAS-1))
		{
			strcat(send, "*");
		}
	}
	esp_mqtt_client_publish(client, "plot", send, 0, 1, 0);
}

//Função de ajuste linear:
float calib(int analog)
{
	if(offset)
	{
		return ((float)((float)(analog-offset)*(((float)MAX_ANGLE-MIN_ANGLE)/(float)sg))+(float)MIN_ANGLE);
	}
	return ((float)analog);
}

//Função para limpar o buffer:
void clr_buff()
{
	for(int i=0; i<BUFF_LEN; i++)
	{
		for(int j=0; j<PARALLEL_DATAS; j++)
		{
			buff.buff1[j][i] = 0;
			buff.buff2[j][i] = 0;
		}
	}
	id = 0;
	buff.b1f = false;
	buff.b2f = false;
}

//Função para atualizar o filtro:
void update_sample()
{
	adc_oneshot_read(handle, POT, &valpot);
	angle += ((calib(valpot)-angle)/(float)kf);
}

//Função para configurar o dutty cycle:
void set_duty(float duty)
{
	ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, ((int)(8191*duty/100.0)));
	ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
}

//Implementação do controlador discreto:
void PID()
{
	//Calcular o sinal de controle:
	float e = (ref-angle);
	float duia = ki*e*(ts*1e-3);
	float ud = (e-ea)*kd/(float)(ts*1e-3);
	u = ((kp*e) + ud + uia + duia);
	ea = e;
	if(u > 16)
	{
		u = 16;
	}else
	{
		if(u <0)
		{
			u = 0;
		}else
		{
			uia += duia;
		}
	}
}

//Função para atualizar o buffer:
void att_buff()
{
	if(id<BUFF_LEN)
	{
		buff.buff1[0][id] = angle*180.0/(float)M_PI;
		buff.buff1[1][id] = ref*180.0/(float)M_PI;
		buff.buff1[2][id] = u*100/16.0;
		id++;
		if(id == BUFF_LEN)
		{
			buff.b1f = true;
		}
	}else
	{
		buff.buff2[0][id%BUFF_LEN] = angle*180.0/(float)M_PI;
		buff.buff2[1][id%BUFF_LEN] = ref*180.0/(float)M_PI;
		buff.buff2[2][id%BUFF_LEN] = u*100/16.0;
		id++;
		if(id == (2*BUFF_LEN))
		{
			buff.b2f = true;
			id = 0;
		}
	}
}

//Função de callback para o timer da malha de controle:
static void periodic_timer_callback(void* arg)
{
	if(mode)
	{
		update_sample();
		if(start)
		{
			if(mode==3)
			{
				//Atualiza ação de controle:
				PID();
				//Aplica PWM:
				set_duty(u*100/16.0);
			}
		}
		att_buff();
	}else
	{
		clr_buff();
	}
}

//Função de callback para o MQTT:
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
	//Associa o evento a um tópico e mensagem:
	esp_mqtt_event_handle_t event = event_data;
	sprintf(topic, "%.*s", event->topic_len, event->topic);
	sprintf(data, "%.*s", event->data_len, event->data);
	//Tópico para seleção do modo de operação:
	if(!strcmp(topic, "mode"))
	{
		mode = atoi(data);
	}
	//Tópico para alteração das constantes do controlador:
	if(!strcmp(topic, "controller"))
	{
		int i=0, pos=0, mark=0;
		float consts[3];
		char cut[12];
		for(i=0; i<event->data_len; i++)
		{
			if(data[i] != ',')
			{
				cut[i-mark] = data[i];
			}else
			{
				cut[i-mark] = '\0';
				consts[pos] = atof(cut);
				pos++;
				mark = i+1;
				cut[0] = '\0';
			}
		}
		cut[i-mark+1] = '\0';
		consts[pos] = atof(cut);
		kp = consts[0];
		ki = consts[1];
		kd = consts[2];
	}
	//Tópico para atualização da referência:
	if(!strcmp(topic, "reference"))
	{
		ref = atof(data)*M_PI/180.0;
	}
}

//Função para inicializar o wifi:
void wifi()
{
	//Inicializando wifi:
	nvs_flash_init();
	esp_netif_init();
	esp_event_loop_create_default();
	esp_netif_create_default_wifi_sta();
	wifi_init_config_t wifi_initiation = WIFI_INIT_CONFIG_DEFAULT();
	esp_wifi_init(&wifi_initiation);
	wifi_config_t wifi_configuration = {
		.sta = {
					.ssid 		= "Erick",
					.password 	= "fanplate",
		},
	};
	esp_wifi_set_mode(WIFI_MODE_STA);
	esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_configuration);
	esp_wifi_start();
	esp_wifi_connect();
}

//Função para inicializar e conectar ao broker MQTT:
void mqtt()
{
	//Configuração do MQTT:
	esp_event_loop_create_default();
	esp_mqtt_client_config_t mqtt_cfg = {
		.broker.address.uri = "mqtt://192.168.137.1:1884/mqtt",
		.credentials.client_id	= "ESP32",
	};
	client = esp_mqtt_client_init(&mqtt_cfg);

	//Registra o callback e inicializa a comunicação:
	esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
	esp_mqtt_client_start(client);

	//Inscrevendo nos tópicos:
	esp_mqtt_client_subscribe(client, "mode", 1);
	esp_mqtt_client_subscribe(client, "controller", 1);
	esp_mqtt_client_subscribe(client, "reference", 1);
}

//Função para configurar o ADC:
void ADC()
{
	//Configuração do ADC:
	adc_oneshot_unit_init_cfg_t init_cfg = {
		.unit_id = ADC_UNIT_1,
		.ulp_mode = ADC_ULP_MODE_DISABLE,
	};
	adc_oneshot_new_unit(&init_cfg, &handle);

	adc_oneshot_chan_cfg_t ch_cfg = {
		.bitwidth = ADC_BITWIDTH_12,
		.atten = ADC_ATTEN_DB_6,
	};
	adc_oneshot_config_channel(handle, POT, &ch_cfg);
}

//Função para configurar o timer da malha de controle:
void periodic_timer()
{
	esp_timer_handle_t periodic_timer = NULL;
	const esp_timer_create_args_t periodic_timer_args = {
		.callback = &periodic_timer_callback,
		.name = "control"
	};
	esp_timer_create(&periodic_timer_args, &periodic_timer);
	esp_timer_start_periodic(periodic_timer, ts*1e3);
}

//Função para configurar o PWM:
void PWM()
{
	//Inicialização do PWM:
	ledc_timer_config_t pwm1c = {
		.speed_mode 		= LEDC_LOW_SPEED_MODE,
		.timer_num 			= LEDC_TIMER_0,
		.duty_resolution 	= LEDC_TIMER_13_BIT,
		.freq_hz 			= 100,
		.clk_cfg 			= LEDC_AUTO_CLK
	};
	ledc_timer_config(&pwm1c);
	ledc_channel_config_t pwm1ch = {
		.speed_mode = LEDC_LOW_SPEED_MODE,
		.channel 	= LEDC_CHANNEL_0,
		.timer_sel 	= LEDC_TIMER_0,
		.intr_type 	= LEDC_INTR_DISABLE,
		.gpio_num 	= COOLER,
		.duty		= 0,
		.hpoint		= 0,
	};
	ledc_channel_config(&pwm1ch);
}

//Rotina de calibração do sensor:
void sensor_calibration()
{
	unsigned long min=valpot;
	//Definição do limite angular máximo:
	u = 100;
	set_duty(u);
	sleep(TAMA);
	unsigned long max=valpot;

	//Definição das constantes de ajuste do sinal:
	offset = min;
	sg = max-min;
}

//Função para looping de envio de dados via MQTT:
void sending_loop(void *args)
{
	Buffer *bf = args;
	//Loop de envio dos dados ao front-end:
	while(1)
	{
		if(bf->b1f)
		{
			send_data(bf->buff1);
			bf->b1f = false;
		}
		if(bf->b2f)
		{
			send_data(bf->buff2);
			bf->b2f = false;
		}
		vTaskDelay(pdMS_TO_TICKS(1));
	}
}

//Função principal:
void app_main()
{
	//Configuração das GPIO's:
	gpio_set_direction(GPIO_NUM_2, GPIO_MODE_OUTPUT);
	gpio_set_direction(GPIO_NUM_25, GPIO_MODE_OUTPUT);

	//Inicializações necessárias:
	sleep(1);
	wifi();
	sleep(3);
	mqtt();
	sleep(1);
	ADC();
	sleep(1);
	periodic_timer();
	PWM();

	//Definição da tarefa de envio:
	xTaskCreate(
			&sending_loop,
			"sending_loop",
			8192,
			&buff,
			1,
			NULL
	);

	//Confirmação de status: sistema pronto
	esp_mqtt_client_publish(client, "feedback", "F0", 0, 1, 0);

	//Rotinas de calibração:
	while(!mode)
	{
		gpio_set_level(GPIO_NUM_2, 1);
		usleep(500e3);
		gpio_set_level(GPIO_NUM_2, 0);
		usleep(500e3);
	}
	gpio_set_level(GPIO_NUM_2, 0);
	sensor_calibration();
	ref = 0;
	u = 0;
	set_duty(u);
	sleep(TAMA);
	esp_mqtt_client_publish(client, "feedback", "F1", 0, 1, 0);
	esp_mqtt_client_publish(client, "mode", "0", 0, 1, 0);
	esp_mqtt_client_publish(client, "feedback", "F2", 0, 1, 0);
	while(!ref)
	{
		printf("\n Aguardando POP...\n");
	}
	esp_mqtt_client_publish(client, "feedback", "F3", 0, 1, 0);
	pop = ref;
	ueq = 42*16/100.0;
	kp = Kp;
	ki = Ki;
	kd = Kd;
	esp_mqtt_client_publish(client, "mode", "1", 0, 1, 0);
	esp_mqtt_client_publish(client, "mode", "0", 0, 1, 0);
	esp_mqtt_client_publish(client, "feedback", "F4", 0, 1, 0);
	esp_mqtt_client_publish(client, "feedback", "F5", 0, 1, 0);

	//Aciona o LED BUILT-IN indicando sucesso das configurações, conexões iniciais e calibração:
	gpio_set_level(GPIO_NUM_2, 1);

	while(1)
	{
		while((mode!=2)&&(mode!=3))
		{
			gpio_set_level(GPIO_NUM_2, 1);
			printf("\n Aguardando comando de início... \n");
			usleep(500000);
			gpio_set_level(GPIO_NUM_2, 0);
			usleep(500000);
		}

		if((mode==2)||(mode==3))
		{
			//Leva sistema ao ponto de operação:
			ref = pop;
			u = ueq;
			set_duty(u*100/16.0);
			esp_mqtt_client_publish(client, "feedback", "F6", 0, 1, 0);
			printf("\n Levando sistema ao ponto de operação...\n");
			sleep(TAMA);
			uia = ueq;
			//Habilita o sistema:
			start = true;
			esp_mqtt_client_publish(client, "feedback", "F7", 0, 1, 0);
		}

		while(mode)
		{
			printf("\n\n! Sistema em operação !\n");
			sleep(1);
		}
		start = false;
		ref = 0;
		u = 0;
		set_duty(u);
	}
}
