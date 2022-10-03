#include <asf.h>

#include "conf_board.h"

#include "gfx_mono_ug_2832hsweg04.h"

#include "gfx_mono_text.h"

#include "sysfont.h"

/* Botao da placa */
#define BUT_PIO PIOA
#define BUT_PIO_ID ID_PIOA
#define BUT_PIO_PIN 11
#define BUT_PIO_PIN_MASK (1u << BUT_PIO_PIN)

/* FASE0 */
#define FASE0_PIO PIOD
#define FASE0_ID ID_PIOD
#define FASE0_PIN 30
#define FASE0_PIN_MASK (1u << FASE0_PIN)

/* FASE1 */
#define FASE1_PIO PIOA
#define FASE1_ID ID_PIOA
#define FASE1_PIN 6
#define FASE1_PIN_MASK (1u << FASE1_PIN)

/* FASE2 */
#define FASE2_PIO PIOC
#define FASE2_ID ID_PIOC
#define FASE2_PIN 19
#define FASE2_PIN_MASK (1u << FASE2_PIN)

/* FASE3 */
#define FASE3_PIO PIOA
#define FASE3_ID ID_PIOA
#define FASE3_PIN 2
#define FASE3_PIN_MASK (1u << FASE3_PIN)

// BOTÃO 1
#define BUT1_PIO PIOD
#define BUT1_PIO_ID ID_PIOD
#define BUT1_PIO_IDX 28
#define BUT1_IDX_MASK (1u << BUT1_PIO_IDX) // esse já está pronto.

// BOTÃO 2
#define BUT2_PIO PIOC
#define BUT2_PIO_ID ID_PIOC
#define BUT2_PIO_IDX 31
#define BUT2_IDX_MASK (1u << BUT2_PIO_IDX) // esse já está pronto.

// BOTÃO 3
#define BUT3_PIO PIOA
#define BUT3_PIO_ID ID_PIOA
#define BUT3_PIO_IDX 19
#define BUT3_IDX_MASK (1u << BUT3_PIO_IDX) // esse já está pronto.

/** RTOS  */
#define TASK_MODO_STACK_SIZE (1024 * 6 / sizeof(portSTACK_TYPE))
#define TASK_MODO_STACK_PRIORITY (tskIDLE_PRIORITY)
#define TASK_MOTOR_STACK_SIZE (1024 * 6 / sizeof(portSTACK_TYPE))
#define TASK_MOTOR_STACK_PRIORITY (tskIDLE_PRIORITY)

TimerHandle_t xTimer;
QueueHandle_t xQueueModo;
QueueHandle_t xQueueSteps;
SemaphoreHandle_t xSemaphoreRTT;

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask, signed char *pcTaskName);
extern void vApplicationIdleHook(void);
extern void vApplicationTickHook(void);
extern void vApplicationMallocFailedHook(void);
extern void xPortSysTickHandler(void);

/** prototypes */
void but_callback(void);
static void BUT_init(void);
void io_init(void);
static void RTT_init(float freqPrescale, uint32_t IrqNPulses, uint32_t rttIRQSource);
void apaga_tela();

/************************************************************************/
/* RTOS application funcs                                               */
/************************************************************************/

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask, signed char *pcTaskName) {
    printf("stack overflow %x %s\r\n", pxTask, (portCHAR *)pcTaskName);
    for (;;) {
    }
}

extern void vApplicationIdleHook(void) {}

extern void vApplicationTickHook(void) {}

extern void vApplicationMallocFailedHook(void) {
    configASSERT((volatile void *)NULL);
}

/************************************************************************/
/* handlers / callbacks                                                 */
/************************************************************************/

void but_callback1(void) {
    BaseType_t xHigherPriorityTaskWoken = pdTRUE;
    int modo = 180;
    xQueueSendFromISR(xQueueModo, &modo, &xHigherPriorityTaskWoken);
}

void but_callback2(void) {
    BaseType_t xHigherPriorityTaskWoken = pdTRUE;
    int modo = 90;
    xQueueSendFromISR(xQueueModo, &modo, &xHigherPriorityTaskWoken);
}

void but_callback3(void) {
    BaseType_t xHigherPriorityTaskWoken = pdTRUE;
    int modo = 45;
    xQueueSendFromISR(xQueueModo, &modo, &xHigherPriorityTaskWoken);
}

void RTT_Handler(void) {
    uint32_t ul_status;
    ul_status = rtt_get_status(RTT);

    /* IRQ due to Alarm */
    if ((ul_status & RTT_SR_ALMS) == RTT_SR_ALMS) {
        xSemaphoreGiveFromISR(xSemaphoreRTT, 0);
    }
}

void apaga_tela() {
    gfx_mono_draw_filled_rect(0, 0, 120, 30, GFX_PIXEL_CLR);
}

/************************************************************************/
/* TASKS                                                                */
/************************************************************************/

static void task_modo(void *pvParameters) {
    gfx_mono_ssd1306_init();
    apaga_tela();
    gfx_mono_draw_string("Modo: None", 35, 12, &sysfont);
    int modo;
    for (;;) {
        if (xQueueReceive(xQueueModo, &modo, 0)) {
            apaga_tela();
            int steps = 0;
            steps = modo / 0.17578125;
            char str[20];
            sprintf(str, "Modo: %d", modo);
            gfx_mono_draw_string(str, 35, 12, &sysfont);
            xQueueSend(xQueueSteps, &steps, 0);
        }
    }
}

static void task_motor(void *pvParameters) {
    const TickType_t xDelay = 5 / portTICK_PERIOD_MS;
    int steps;
    int fase0 = 0;
    int fase1 = 0;
    int fase2 = 0;
    int fase3 = 0;
    for (;;) {
        if (xQueueReceive(xQueueSteps, &steps, 0)) {
            fase0 = 1;
            steps = steps;
            RTT_init(1000, 1, RTT_MR_ALMIEN);
        }
        if (xSemaphoreTake(xSemaphoreRTT, 0) && steps > 0 && fase0) {
			pio_clear(FASE3_PIO, FASE3_PIN_MASK);
            pio_set(FASE0_PIO, FASE0_PIN_MASK);
            steps--;
            fase0 = 0;
            fase1 = 1;
            RTT_init(1000, 1, RTT_MR_ALMIEN);
        }
        if (xSemaphoreTake(xSemaphoreRTT, 0) && steps > 0 && fase1) {
            pio_clear(FASE0_PIO, FASE0_PIN_MASK);
            pio_set(FASE1_PIO, FASE1_PIN_MASK);
            steps--;
            fase1 = 0;
            fase2 = 1;
            RTT_init(1000, 1, RTT_MR_ALMIEN);
        }
        if (xSemaphoreTake(xSemaphoreRTT, 0) && steps > 0 && fase2) {
            pio_clear(FASE1_PIO, FASE1_PIN_MASK);
            pio_set(FASE2_PIO, FASE2_PIN_MASK);
            steps--;
            fase2 = 0;
            fase3 = 1;
            RTT_init(1000, 1, RTT_MR_ALMIEN);
        }
        if (xSemaphoreTake(xSemaphoreRTT, 0) && steps > 0 && fase3) {
            pio_clear(FASE2_PIO, FASE2_PIN_MASK);
            pio_set(FASE3_PIO, FASE3_PIN_MASK);            
            steps--;
            fase3 = 0;
            fase0 = 1;
            RTT_init(1000, 1, RTT_MR_ALMIEN);
        }
    }
}

/************************************************************************/
/* funcoes                                                              */
/************************************************************************/

static void configure_console(void) {
    const usart_serial_options_t uart_serial_options = {
        .baudrate = CONF_UART_BAUDRATE,
        .charlength = CONF_UART_CHAR_LENGTH,
        .paritytype = CONF_UART_PARITY,
        .stopbits = CONF_UART_STOP_BITS,
    };

    /* Configure console UART. */
    stdio_serial_init(CONF_UART, &uart_serial_options);

    /* Specify that stdout should not be buffered. */
    setbuf(stdout, NULL);
}

static void BUT_init(void) {
    /* configura prioridae */
    NVIC_EnableIRQ(BUT_PIO_ID);
    NVIC_SetPriority(BUT_PIO_ID, 4);

    /* conf bot�o como entrada */
    pio_configure(BUT_PIO, PIO_INPUT, BUT_PIO_PIN_MASK, PIO_PULLUP | PIO_DEBOUNCE);
    pio_set_debounce_filter(BUT_PIO, BUT_PIO_PIN_MASK, 60);
    pio_enable_interrupt(BUT_PIO, BUT_PIO_PIN_MASK);
    pio_handler_set(BUT_PIO, BUT_PIO_ID, BUT_PIO_PIN_MASK, PIO_IT_FALL_EDGE, but_callback);
}

void io_init(void) {
    pmc_enable_periph_clk(BUT2_PIO);
    pmc_enable_periph_clk(BUT3_PIO);
    pmc_enable_periph_clk(FASE0_PIO);
    pmc_enable_periph_clk(FASE1_PIO);
    pmc_enable_periph_clk(FASE2_PIO);
    pmc_enable_periph_clk(FASE3_PIO);

    // Inicializa fases como saída
    pio_set_output(FASE0_PIO, FASE0_PIN_MASK, 0, 0, 0);
    pio_set_output(FASE1_PIO, FASE1_PIN_MASK, 0, 0, 0);
    pio_set_output(FASE2_PIO, FASE2_PIN_MASK, 0, 0, 0);
    pio_set_output(FASE3_PIO, FASE3_PIN_MASK, 0, 0, 0);

    pio_configure(BUT1_PIO, PIO_INPUT, BUT1_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);
    pio_configure(BUT2_PIO, PIO_INPUT, BUT2_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);
    pio_configure(BUT3_PIO, PIO_INPUT, BUT3_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);

    pio_set_debounce_filter(BUT1_PIO, BUT1_IDX_MASK, 60);
    pio_set_debounce_filter(BUT2_PIO, BUT2_IDX_MASK, 60);
    pio_set_debounce_filter(BUT3_PIO, BUT3_IDX_MASK, 60);

    pio_handler_set(BUT1_PIO,
                    BUT1_PIO_ID,
                    BUT1_IDX_MASK,
                    PIO_IT_EDGE,
                    but_callback1);

    pio_handler_set(BUT2_PIO,
                    BUT2_PIO_ID,
                    BUT2_IDX_MASK,
                    PIO_IT_EDGE,
                    but_callback2);

    pio_handler_set(BUT3_PIO,
                    BUT3_PIO_ID,
                    BUT3_IDX_MASK,
                    PIO_IT_EDGE,
                    but_callback3);

    pio_enable_interrupt(BUT1_PIO, BUT1_IDX_MASK);
    pio_enable_interrupt(BUT2_PIO, BUT2_IDX_MASK);
    pio_enable_interrupt(BUT3_PIO, BUT3_IDX_MASK);

    pio_get_interrupt_status(BUT1_PIO);
    pio_get_interrupt_status(BUT2_PIO);
    pio_get_interrupt_status(BUT3_PIO);

    NVIC_EnableIRQ(BUT1_PIO_ID);
    NVIC_EnableIRQ(BUT2_PIO_ID);
    NVIC_EnableIRQ(BUT3_PIO_ID);

    NVIC_SetPriority(BUT1_PIO_ID, 4);
    NVIC_SetPriority(BUT2_PIO_ID, 4);
    NVIC_SetPriority(BUT3_PIO_ID, 4);
}

static void RTT_init(float freqPrescale, uint32_t IrqNPulses, uint32_t rttIRQSource) {

    uint16_t pllPreScale = (int)(((float)32768) / freqPrescale);

    rtt_sel_source(RTT, false);
    rtt_init(RTT, pllPreScale);

    if (rttIRQSource & RTT_MR_ALMIEN) {
        uint32_t ul_previous_time;
        ul_previous_time = rtt_read_timer_value(RTT);
        while (ul_previous_time == rtt_read_timer_value(RTT))
            ;
        rtt_write_alarm_time(RTT, IrqNPulses + ul_previous_time);
    }

    /* config NVIC */
    NVIC_DisableIRQ(RTT_IRQn);
    NVIC_ClearPendingIRQ(RTT_IRQn);
    NVIC_SetPriority(RTT_IRQn, 4);
    NVIC_EnableIRQ(RTT_IRQn);

    /* Enable RTT interrupt */
    if (rttIRQSource & (RTT_MR_RTTINCIEN | RTT_MR_ALMIEN))
        rtt_enable_interrupt(RTT, rttIRQSource);
    else
        rtt_disable_interrupt(RTT, RTT_MR_RTTINCIEN | RTT_MR_ALMIEN);
}

/************************************************************************/
/* main                                                                 */
/************************************************************************/

int main(void) {
    /* Initialize the SAM system */
    sysclk_init();
    board_init();

    /* Initialize the console uart */
    configure_console();
    io_init();

    xSemaphoreRTT = xSemaphoreCreateBinary();

    /* Create task to control oled */
    if (xTaskCreate(task_modo, "modo", TASK_MODO_STACK_SIZE, NULL, TASK_MODO_STACK_PRIORITY, NULL) != pdPASS) {
        printf("Failed to create oled task\r\n");
    }

    /* Create task to control motor */
    if (xTaskCreate(task_motor, "motor", TASK_MOTOR_STACK_SIZE, NULL, TASK_MOTOR_STACK_PRIORITY, NULL) != pdPASS) {
        printf("Failed to create motor task\r\n");
    }

    xQueueModo = xQueueCreate(32, sizeof(int));
    if (xQueueModo == NULL)
        printf("falha em criar a queue xQueueModo \n");

    xQueueSteps = xQueueCreate(32, sizeof(int));
    if (xQueueSteps == NULL)
        printf("falha em criar a queue xQueueSteps \n");

    /* Start the scheduler. */
    vTaskStartScheduler();

    /* RTOS n�o deve chegar aqui !! */
    while (1) {
    }

    /* Will only get here if there was insufficient memory to create the idle task. */
    return 0;
}
