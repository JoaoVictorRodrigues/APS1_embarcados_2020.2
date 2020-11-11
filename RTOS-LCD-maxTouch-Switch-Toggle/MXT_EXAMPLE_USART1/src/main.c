#include <asf.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "conf_board.h"
#include "conf_uart_serial.h"
#include "maxTouch/maxTouch.h"
#include "tfont.h"
#include "digital521.h"
#include "design/oxi-num-small.h"
#include "design/oxi-numero.h"
#include "design/OXI_screen_night.h"
#include "design/OXI_screen_day.h" 

/************************************************************************/
/* prototypes                                                           */
/************************************************************************/
void but1_callback(void);


/************************************************************************/
/* LCD + TOUCH                                                          */
/************************************************************************/
#define MAX_ENTRIES        10

#define AFEC_POT AFEC1
#define AFEC_POT_ID ID_AFEC1
#define AFEC_POT_CHANNEL		6		// Canal do pino PC31

#define AFEC_POT_0 AFEC0
#define AFEC_POT_ID_0 ID_AFEC0
#define AFEC_POT_CHANNEL_0		8		// Canal do pino PC31

struct ili9488_opt_t g_ili9488_display_opt;
const uint32_t BUTTON_W = 120;
const uint32_t BUTTON_H = 150;
const uint32_t BUTTON_BORDER = 2;
const uint32_t BUTTON_X = ILI9488_LCD_WIDTH/2;
const uint32_t BUTTON_Y = ILI9488_LCD_HEIGHT/2;

const uint32_t LOCX_DAY_NIGHT = 0;
const uint32_t LOCY_DAY_NIGHT = 406;
const uint32_t W_DAY_NIGHT = 112;
const uint32_t H_DAY_NIGHT = 73;
const uint32_t LOCX_PLAY = 111;
const uint32_t LOCY_PLAY = 406;
const uint32_t W_PLAY = 209;
const uint32_t H_PLAY = 73;




/************************************************************************/
/* Botoes lcd                                                           */
/************************************************************************/


/************************************************************************/
/* RTOS                                                                  */
/************************************************************************/
#define TASK_MXT_STACK_SIZE            (2*1024/sizeof(portSTACK_TYPE))
#define TASK_MXT_STACK_PRIORITY        (tskIDLE_PRIORITY)

#define TASK_LCD_STACK_SIZE            (4*1024/sizeof(portSTACK_TYPE))
#define TASK_LCD_STACK_PRIORITY        (tskIDLE_PRIORITY)

typedef struct {
  uint x;
  uint y;
} touchData;

typedef struct {
	uint value;
} adcData;


QueueHandle_t xQueueTouch;
QueueHandle_t xQueueADC_bat;
QueueHandle_t xQueueADC_oxi;

/* Semaforos */
SemaphoreHandle_t xSemaphore_bat;
SemaphoreHandle_t xSemaphore_oxi;

/** The conversion data is done flag */
volatile bool g_is_conversion_done = false;

/** The conversion data value */
volatile uint32_t g_ul_value = 0;
/************************************************************************/
/* handler/callbacks                                                    */
/************************************************************************/


/************************************************************************/
/* RTOS hooks                                                           */
/************************************************************************/

/**
* \brief Called if stack overflow during execution
*/
extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,
signed char *pcTaskName)
{
  printf("stack overflow %x %s\r\n", pxTask, (portCHAR *)pcTaskName);
  /* If the parameters have been corrupted then inspect pxCurrentTCB to
  * identify which task has overflowed its stack.
  */
  for (;;) {
  }
}

/**
* \brief This function is called by FreeRTOS idle task
*/
extern void vApplicationIdleHook(void)
{
  pmc_sleep(SAM_PM_SMODE_SLEEP_WFI);
}

/**
* \brief This function is called by FreeRTOS each tick
*/
extern void vApplicationTickHook(void)
{
}

extern void vApplicationMallocFailedHook(void)
{
  /* Called if a call to pvPortMalloc() fails because there is insufficient
  free memory available in the FreeRTOS heap.  pvPortMalloc() is called
  internally by FreeRTOS API functions that create tasks, queues, software
  timers, and semaphores.  The size of the FreeRTOS heap is set by the
  configTOTAL_HEAP_SIZE configuration constant in FreeRTOSConfig.h. */

  /* Force an assert. */
  configASSERT( ( volatile void * ) NULL );
}

/************************************************************************/
/* init                                                                 */
/************************************************************************/

static void configure_lcd(void){
  /* Initialize display parameter */
  g_ili9488_display_opt.ul_width = ILI9488_LCD_WIDTH;
  g_ili9488_display_opt.ul_height = ILI9488_LCD_HEIGHT;
  g_ili9488_display_opt.foreground_color = COLOR_CONVERT(COLOR_BLACK);
  g_ili9488_display_opt.background_color = COLOR_CONVERT(COLOR_BLACK);

  /* Initialize LCD */
  ili9488_init(&g_ili9488_display_opt);
}

/************************************************************************/
/* funcoes                                                              */
/************************************************************************/

void draw_screen(void) {
  ili9488_set_foreground_color(COLOR_CONVERT(COLOR_BLACK));
  ili9488_draw_filled_rectangle(0, 0, ILI9488_LCD_WIDTH-1, ILI9488_LCD_HEIGHT-1);
	ili9488_draw_pixmap(0, 0, OXI_screen_night.width, OXI_screen_night.height, OXI_screen_night.data);
}

// void draw_button(uint32_t clicked) {
//   static uint32_t last_state = 255; // undefined
//   if(clicked == last_state) return;
//   
//   ili9488_set_foreground_color(COLOR_CONVERT(COLOR_BLACK));
//   ili9488_draw_filled_rectangle(BUTTON_X-BUTTON_W/2, BUTTON_Y-BUTTON_H/2, BUTTON_X+BUTTON_W/2, BUTTON_Y+BUTTON_H/2);
//   if(clicked) {
//     ili9488_set_foreground_color(COLOR_CONVERT(COLOR_TOMATO));
//     ili9488_draw_filled_rectangle(BUTTON_X-BUTTON_W/2+BUTTON_BORDER, BUTTON_Y+BUTTON_BORDER, BUTTON_X+BUTTON_W/2-BUTTON_BORDER, BUTTON_Y+BUTTON_H/2-BUTTON_BORDER);
//     } else {
//     ili9488_set_foreground_color(COLOR_CONVERT(COLOR_GREEN));
//     ili9488_draw_filled_rectangle(BUTTON_X-BUTTON_W/2+BUTTON_BORDER, BUTTON_Y-BUTTON_H/2+BUTTON_BORDER, BUTTON_X+BUTTON_W/2-BUTTON_BORDER, BUTTON_Y-BUTTON_BORDER);
//   }
//   last_state = clicked;
// }

uint32_t convert_axis_system_x(uint32_t touch_y) {
  // entrada: 4096 - 0 (sistema de coordenadas atual)
  // saida: 0 - 320
  return ILI9488_LCD_WIDTH - ILI9488_LCD_WIDTH*touch_y/4096;
}

uint32_t convert_axis_system_y(uint32_t touch_x) {
  // entrada: 0 - 4096 (sistema de coordenadas atual)
  // saida: 0 - 320
  return ILI9488_LCD_HEIGHT*touch_x/4096;
}

void font_draw_text(tFont *font, const char *text, int x, int y, int spacing) {
  char *p = text;
  while(*p != NULL) {
    char letter = *p;
    int letter_offset = letter - font->start_char;
    if(letter <= font->end_char) {
      tChar *current_char = font->chars + letter_offset;
      ili9488_draw_pixmap(x, y, current_char->image->width, current_char->image->height, current_char->image->data);
      x += current_char->image->width + spacing;
    }
    p++;
  }
}

void mxt_handler(struct mxt_device *device, uint *x, uint *y)
{
  /* USART tx buffer initialized to 0 */
  uint8_t i = 0; /* Iterator */

  /* Temporary touch event data struct */
  struct mxt_touch_event touch_event;
  
  /* first touch only */
  uint first = 0;

  /* Collect touch events and put the data in a string,
  * maximum 2 events at the time */
  do {

    /* Read next next touch event in the queue, discard if read fails */
    if (mxt_read_touch_event(device, &touch_event) != STATUS_OK) {
      continue;
    }
    
    /************************************************************************/
    /* Envia dados via fila RTOS                                            */
    /************************************************************************/
    if(first == 0 ){
      *x = convert_axis_system_x(touch_event.y);
      *y = convert_axis_system_y(touch_event.x);
      first = 1;
    }
    
    i++;

    /* Check if there is still messages in the queue and
    * if we have reached the maximum numbers of events */
  } while ((mxt_is_message_pending(device)) & (i < MAX_ENTRIES));
}

static void AFEC_pot_Callback_bat(void){
	g_ul_value = afec_channel_get_value(AFEC_POT, AFEC_POT_CHANNEL);
	g_is_conversion_done = true;
	
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	xSemaphoreGiveFromISR(xSemaphore_bat, &xHigherPriorityTaskWoken);
	
	adcData adc_bat;
	adc_bat.value = g_ul_value;
	xQueueSendFromISR(xQueueADC_bat, &adc_bat, 0);
}

static void AFEC_pot_Callback_oxi(void){
	g_ul_value = afec_channel_get_value(AFEC_POT_0, AFEC_POT_CHANNEL_0);
	g_is_conversion_done = true;
	
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	xSemaphoreGiveFromISR(xSemaphore_oxi, &xHigherPriorityTaskWoken);
	
	adcData adc_oxi;
	adc_oxi.value = g_ul_value;
	xQueueSendFromISR(xQueueADC_oxi, &adc_oxi, 0);
}

static void config_AFEC_pot(Afec *afec, uint32_t afec_id, uint32_t afec_channel, afec_callback_t callback){
  /*************************************
  * Ativa e configura AFEC
  *************************************/
  /* Ativa AFEC - 0 */
  afec_enable(afec);

  /* struct de configuracao do AFEC */
  struct afec_config afec_cfg;

  /* Carrega parametros padrao */
  afec_get_config_defaults(&afec_cfg);

  /* Configura AFEC */
  afec_init(afec, &afec_cfg);

  /* Configura trigger por software */
  afec_set_trigger(afec, AFEC_TRIG_SW);

  /*** Configuracao espec?fica do canal AFEC ***/
  struct afec_ch_config afec_ch_cfg;
  afec_ch_get_config_defaults(&afec_ch_cfg);
  afec_ch_cfg.gain = AFEC_GAINVALUE_0;
  afec_ch_set_config(afec, afec_channel, &afec_ch_cfg);

  /*
  * Calibracao:
  * Because the internal ADC offset is 0x200, it should cancel it and shift
  down to 0.
  */
  afec_channel_set_analog_offset(afec, afec_channel, 0x200);

  /***  Configura sensor de temperatura ***/
  struct afec_temp_sensor_config afec_temp_sensor_cfg;

  afec_temp_sensor_get_config_defaults(&afec_temp_sensor_cfg);
  afec_temp_sensor_set_config(afec, &afec_temp_sensor_cfg);
  
  /* configura IRQ */
  afec_set_callback(afec, afec_channel,	callback, 1);
  NVIC_SetPriority(afec_id, 4);
  NVIC_EnableIRQ(afec_id);
}

int set_touch(uint32_t tx, uint32_t ty, uint32_t LocX, uint32_t LocY, uint32_t ButtonW, uint32_t ButtonH)
{
	if (tx >= LocX && tx <= LocX + ButtonW)
	{
		if (ty >= LocY && ty <= LocY + ButtonH)
		{
			return 1;
		}
	}
	return 0;
}

void update_screen(uint32_t tx, uint32_t ty,int *pause,int *day_night)
{
	if (set_touch(tx, ty, LOCX_PLAY, LOCY_PLAY, W_PLAY, H_PLAY))
	{
		*pause = !*pause;
	}
	if (set_touch(tx, ty, LOCX_DAY_NIGHT, LOCY_DAY_NIGHT, W_DAY_NIGHT, H_DAY_NIGHT))
	{
		*day_night = !*day_night;
	}
}

void update_oxi(int n){
	char buff[32];
	sprintf(buff, "%03d" , n);
	font_draw_text(&oxinumero, buff, 150, 50, 0);
}

void update_bat(int n){
	char buff[32];
	sprintf(buff, "%03d", n);
	font_draw_text(&oxinumero, buff, 150, 280, 0);
}

int filter(int p){
	return p/40;
}

void draw_hist_oxi(int *hist[]){
	char buff0[32];
	char buff1[32];
	sprintf(buff0,"%03d",hist[0]);
	sprintf(buff1,"%03d",hist[1]);
	font_draw_text(&oxinumsmall,buff0,10,60,0);
	font_draw_text(&oxinumsmall,buff1,10,110,0);
}

void draw_hist_bat(int *hist[]){
	char buff0[32];
	char buff1[32];
	sprintf(buff0,"%03d",hist[0]);
	sprintf(buff1,"%03d",hist[1]);
	font_draw_text(&oxinumsmall,buff0,30,60,0);
	font_draw_text(&oxinumsmall,buff1,30,110,0);
}
/************************************************************************/
/* tasks                                                                */
/************************************************************************/

void task_mxt(void){
  
  struct mxt_device device; /* Device data container */
  mxt_init(&device);       	/* Initialize the mXT touch device */
  touchData touch;          /* touch queue data type*/
  
  while (true) {
    /* Check for any pending messages and run message handler if any
    * message is found in the queue */
    if (mxt_is_message_pending(&device)) {
      mxt_handler(&device, &touch.x, &touch.y);
      xQueueSend( xQueueTouch, &touch, 0);           /* send mesage to queue */
      vTaskDelay(200);
      
      // limpa touch
      while (mxt_is_message_pending(&device)){
        mxt_handler(&device, NULL, NULL);
        vTaskDelay(50);
      }
    }
    
    vTaskDelay(300);
  }
}

void task_adc(void){

	/* inicializa e configura adc up*/
	config_AFEC_pot(AFEC_POT, AFEC_POT_ID, AFEC_POT_CHANNEL, AFEC_pot_Callback_bat);
	config_AFEC_pot(AFEC_POT_0, AFEC_POT_ID_0, AFEC_POT_CHANNEL_0, AFEC_pot_Callback_oxi);

	/* Selecina canal e inicializa convers?o */
	afec_channel_enable(AFEC_POT, AFEC_POT_CHANNEL);
	afec_start_software_conversion(AFEC_POT);
	
	afec_channel_enable(AFEC_POT_0, AFEC_POT_CHANNEL_0);
	afec_start_software_conversion(AFEC_POT_0);

	adcData adc;
	
	xSemaphore_bat = xSemaphoreCreateBinary();
	xSemaphore_oxi = xSemaphoreCreateBinary();
	
	if (xSemaphore_bat == NULL) printf("falha em criar o semaforo \n");
	if (xSemaphore_oxi == NULL) printf("falha em criar o semaforo \n");

	
	while(1){
		if(xSemaphoreTake(xSemaphore_bat, (TickType_t)500) == pdTRUE){
			//if(g_is_conversion_done){
			printf("%d\n", g_ul_value);
			
			vTaskDelay(500);

			/* Selecina canal e inicializa convers?o */
			afec_channel_enable(AFEC_POT, AFEC_POT_CHANNEL);
			afec_start_software_conversion(AFEC_POT);
		}
		if(xSemaphoreTake(xSemaphore_oxi, (TickType_t)500) == pdTRUE){
			//if(g_is_conversion_done){
			printf("%d\n", g_ul_value);
					
			vTaskDelay(500);

			/* Selecina canal e inicializa convers?o */
			afec_channel_enable(AFEC_POT_0, AFEC_POT_CHANNEL_0);
			afec_start_software_conversion(AFEC_POT_0);
		}
	}
}

void update_hist(int *hist[],int value){
	*hist[1] = *hist[0];
	*hist[0] = value;
	}
	


void task_lcd(void){
  xQueueTouch = xQueueCreate( 10, sizeof( touchData ) );
	xQueueADC_bat   = xQueueCreate( 5, sizeof( adcData ) );
	xQueueADC_oxi   = xQueueCreate( 5, sizeof( adcData ) );
  configure_lcd();
  
  draw_screen();
  
  // Escreve DEMO - BUT no LCD
  //font_draw_text(&batnumero, "0123", 0, 0, 0);
  //update_bat(23);
	//update_oxi(100);
	int pause=1;
	int day_night=1;
	int ZERA = 0;
	int got = 0;
	
	int hist_bat[2];
	int temp_bat;
	int hist_oxi[2];
	int temp_oxi;
  
  // strut local para armazenar msg enviada pela task do mxt
  touchData touch;
  adcData adc_bat;
	adcData adc_oxi;
  while (true) {
    if (xQueueReceive( xQueueTouch, &(touch), ( TickType_t )  500 / portTICK_PERIOD_MS)) {
			update_screen(touch.x, touch.y,&pause,&day_night);
      printf("x:%d y:%d\n", touch.x, touch.y);
			
    }
		if (!pause)
		{
				got = 1;
				if (xQueueReceive( xQueueADC_bat, &(adc_bat), ( TickType_t )  100 / portTICK_PERIOD_MS)) {
					temp_bat = filter(adc_bat.value);
					update_bat(filter(adc_bat.value));
					printf("adc_bat: %d\n", adc_bat.value);
				}
					
				if (xQueueReceive( xQueueADC_oxi, &(adc_oxi), ( TickType_t )  100 / portTICK_PERIOD_MS)) {
					temp_oxi = filter(adc_oxi.value);
					update_oxi(filter(adc_oxi.value));
				}
		} else{
				update_bat(filter(ZERA));
				update_oxi(filter(ZERA));
		}
		if(pause && got){
			got = 0;
			update_hist(&hist_bat,temp_bat);
			update_hist(&hist_oxi,temp_oxi);
			draw_hist_bat(&hist_bat);
			draw_hist_oxi(&hist_oxi);

		}

  }
}

/************************************************************************/
/* main                                                                 */
/************************************************************************/

int main(void)
{
  /* Initialize the USART configuration struct */
  const usart_serial_options_t usart_serial_options = {
    .baudrate     = USART_SERIAL_EXAMPLE_BAUDRATE,
    .charlength   = USART_SERIAL_CHAR_LENGTH,
    .paritytype   = USART_SERIAL_PARITY,
    .stopbits     = USART_SERIAL_STOP_BIT
  };

  sysclk_init(); /* Initialize system clocks */
  board_init();  /* Initialize board */
  
  /* Initialize stdio on USART */
  stdio_serial_init(USART_SERIAL_EXAMPLE, &usart_serial_options);
  
  /* Create task to handler touch */
  if (xTaskCreate(task_mxt, "mxt", TASK_MXT_STACK_SIZE, NULL, TASK_MXT_STACK_PRIORITY, NULL) != pdPASS) {
    printf("Failed to create test led task\r\n");
  }
  
  /* Create task to handler LCD */
  if (xTaskCreate(task_lcd, "lcd", TASK_LCD_STACK_SIZE, NULL, TASK_LCD_STACK_PRIORITY, NULL) != pdPASS) {
    printf("Failed to create test led task\r\n");
  }
	
	/* Create task to handler LCD */
	if (xTaskCreate(task_adc, "adc", TASK_LCD_STACK_SIZE, NULL, TASK_LCD_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create test adc task\r\n");
	}
  
  /* Start the scheduler. */
  vTaskStartScheduler();

  while(1){

  }


  return 0;
}
