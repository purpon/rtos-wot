/* Respond to a button press.
 *
 * This code combines two ways of checking for a button press -
 * busy polling (the bad way) and button interrupt (the good way).
 *
 * This sample code is in the public domain.
 */
#include "espressif/esp_common.h"
#include "esp/uart.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "esp8266.h"

/* pin config */
const int gpio_blink = 2;
const int gpio = 5;   /* gpio 0 usually has "PROGRAM" button attached */
const int active = 1; /* active == 0 for active high */
const gpio_inttype_t int_type = GPIO_INTTYPE_EDGE_NEG;
#define GPIO_HANDLER gpio14_interrupt_handler


/* This task polls for the button and prints the tick
   count when it's seen.

   Debounced to 200ms with a simple vTaskDelay.

   This is not a good example of how to wait for button input!
*/
#define MEASURE_DURATION_MILLIS 700 
static uint32_t lastPress;
static uint8_t startToBlink;
void buttonPollTask(void *pvParameters)
{
	// the press use to indicate if the button press
	uint8_t press=0;
	// previous button pressed time stamp
	uint32_t pTimeStamp=0;
	printf("Polling for button press on gpio %d...\r\n", gpio);
	xQueueHandle *psqueue = (xQueueHandle *)pvParameters;
	//printf("address = %x\n",(unsigned int)*psqueue);
	while(1) {
		// button don't press
        while(gpio_read(gpio) != active) {
			press =0;
			if((pTimeStamp!=lastPress) &&((xTaskGetTickCount()-lastPress)*portTICK_RATE_MS > MEASURE_DURATION_MILLIS)){
				pTimeStamp = lastPress;
				startToBlink=1;
			}
			// release cpu time
			taskYIELD();
        }
		// button press
		if(press!=1) {
			press =1;
			lastPress = xTaskGetTickCount();
			printf("Polled for button press at %dms\r\n", xTaskGetTickCount()*portTICK_RATE_MS);
			//printf("address = %x\n",(unsigned int)*psqueue);
			xQueueSendToBack(*psqueue,(void *) &press , portMAX_DELAY);
			//vTaskDelay(100 / portTICK_RATE_MS);
		}
    }
}

void blinkLed(void *pvParameters)
{
	startToBlink=0;
	xQueueHandle *psqueue = (xQueueHandle *)pvParameters;
	uint8_t press;
	while(1) {
		while(startToBlink) {
			// if queue is empety , break the while loop
			if(uxQueueMessagesWaiting(*psqueue)==0)
				break;
			xQueueReceive(*psqueue, &press, portMAX_DELAY);
			if(press) {
				//printf("ON\n");
				gpio_write(gpio_blink,0);
				vTaskDelay(1000 / portTICK_RATE_MS);
				gpio_write(gpio_blink,1);
				//printf("OFF\n");
				vTaskDelay(1000 / portTICK_RATE_MS);
			}
			press=0;
		}
		startToBlink =0;
		// release cpu time
		taskYIELD();
	}
}

/* This task configures the GPIO interrupt and uses it to tell
   when the button is pressed.

   The interrupt handler communicates the exact button press time to
   the task via a queue.

   This is a better example of how to wait for button input!
*/
//static int pressed = 0;

void buttonIntTask(void *pvParameters)
{
    printf("Waiting for button press interrupt on gpio %d...\r\n", gpio);
    xQueueHandle *tsqueue = (xQueueHandle *)pvParameters;
    gpio_set_interrupt(gpio, int_type);

    uint32_t last = 0;
    while(1) {
        uint32_t button_ts;
        xQueueReceive(*tsqueue, &button_ts, portMAX_DELAY);
        button_ts *= portTICK_RATE_MS;
        if(last < button_ts-200) {
            printf("Button interrupt fired at %dms\r\n", button_ts);
            last = button_ts;

            // Blink the LED.
            gpio_write(gpio_blink, 1);
            vTaskDelay(1000 / portTICK_RATE_MS);
            gpio_write(gpio_blink, 0);
            vTaskDelay(1000 / portTICK_RATE_MS);
        }
    }
}

static xQueueHandle tsqueue;
static xQueueHandle psqueue;
#define MAX_PRESS_QUEUE_LENGTH 100
void GPIO_HANDLER(void)
{
    uint32_t now = xTaskGetTickCountFromISR();
    xQueueSendToBackFromISR(tsqueue, &now, NULL);
}

void user_init(void)
{
    uart_set_baud(0, 115200);
    gpio_enable(gpio, GPIO_INPUT);
    gpio_enable(gpio_blink, GPIO_OUTPUT);
	gpio_write(gpio_blink,1);
	psqueue = xQueueCreate(10,sizeof(uint8_t));
    tsqueue = xQueueCreate(2, sizeof(uint32_t));
    if(psqueue ==NULL)
		printf("psqueue fail\n");
	else
		printf("address = %x\n",(unsigned int) psqueue);
	//xTaskCreate(buttonIntTask, (signed char *)"buttonIntTask", 256, &tsqueue, 2, NULL);
    xTaskCreate(buttonPollTask, (signed char*)"buttonPollTask", 256, &psqueue, 2, NULL);
	xTaskCreate(blinkLed, (signed char*)"blinkLed", 256, &psqueue, 2, NULL);

}
