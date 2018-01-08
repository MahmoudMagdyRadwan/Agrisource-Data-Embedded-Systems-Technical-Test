#include <stdint.h>
#include <stddef.h>
#include "agrisourcedata.h"
#include <pthread.h>
#include <ti/drivers/ADC.h>
#include <stdlib.h>
#include <ti/drivers/rf/RF.h>
#include <ti/drivers/PIN.h>
#include <ti/drivers/pin/PINCC26XX.h>
#include "Board.h"
#include DeviceFamily_constructPath(driverlib/rf_prop_mailbox.h)
#include "smartrf_settings/smartrf_settings.h"
#include <unistd.h>
#include "RFQueue.h"
#include <ti/drivers/GPIO.h>
#include <ti/drivers/UART.h>
static RF_Object rfObject;
static RF_Handle rfHandle;
static PIN_Handle ledPinHandle;
static PIN_State ledPinState;
static uint8_t packet[PAYLOAD_LENGTH];
static uint16_t seqNumber;
/* ADC conversion result variables */
uint16_t adcValue0;
uint32_t adcValue0MicroVolt;
uint16_t adcValue1[ADC_SAMPLE_COUNT];
uint32_t adcValue1MicroVolt[ADC_SAMPLE_COUNT];

static void callback(RF_Handle h, RF_CmdHandle ch, RF_EventMask e);

static RF_Object rfObject;
static RF_Handle rfHandle;
static PIN_Handle ledPinHandle;
static PIN_State ledPinState;

/* Receive dataQueue for RF Core to fill in data */
static dataQueue_t dataQueue;
static rfc_dataEntryGeneral_t* currentDataEntry;
static uint8_t packetLength;
static uint8_t* packetDataPointer;
static uint8_t packet[MAX_LENGTH + NUM_APPENDED_BYTES - 1]; /* The length byte is stored in a separate variable */


int main(void)
{
    UART_Handle uart;
    UART_Params uartParams;
    PIN_State   pinState;
    PIN_Handle  hPin;
    uint32_t    currentOutputVal;
    uint32_t    standbyDuration = 5;
    pthread_t           thread0, thread1;
    pthread_attr_t      attrs;
    struct sched_param  priParam;
    int                 retc;
    int                 detachState;
    uint32_t curtime;
    RF_Params rfParams;
    RF_Params_init(&rfParams);

    ADC_init();
    /* Create application threads */
    pthread_attr_init(&attrs);
    detachState = PTHREAD_CREATE_DETACHED;
    /* Set priority and stack size attributes */
    retc = pthread_attr_setdetachstate(&attrs, detachState);
    retc |= pthread_attr_setstacksize(&attrs, THREADSTACKSIZE);
    /* Create threadFxn1 thread */
    retc = pthread_create(&thread1, &attrs, threadFxn1, (void* )0);

    RF_cmdPropTx.pktLen = PAYLOAD_LENGTH;
    RF_cmdPropTx.pPkt = packet;
    RF_cmdPropTx.startTrigger.triggerType = TRIG_ABSTIME;
    RF_cmdPropTx.startTrigger.pastTrig = 1;
    RF_cmdPropTx.startTime = 0;
    rfHandle = RF_open(&rfObject, &RF_prop, (RF_RadioSetup*)&RF_cmdPropRadioDivSetup, &rfParams);
    RF_postCmd(rfHandle, (RF_Op*)&RF_cmdFs, RF_PriorityNormal, NULL, 0);
    curtime = RF_getCurrentTime();
    RF_EventMask terminationReason = RF_runCmd(rfHandle, (RF_Op*)&RF_cmdPropTx,
                                               RF_PriorityNormal, NULL, 0);

    /* Allocate LED pins */
    hPin = PIN_open(&pinState, LedPinTable);


    /* Sleep, to let the power policy transition the device to standby */
    sleep(standbyDuration);

    /* Read current output value for all pins */
    currentOutputVal =  PIN_getPortOutputValue(hPin);

    /* Toggle the LEDs, configuring all LEDs at once */
    PIN_setPortOutputValue(hPin, ~currentOutputVal);


    /* Set the Data Entity queue for received data */
    RF_cmdPropRx.pQueue = &dataQueue;
    /* Discard ignored packets from Rx queue */
    RF_cmdPropRx.rxConf.bAutoFlushIgnored = 1;
    /* Discard packets with CRC error from Rx queue */
    RF_cmdPropRx.rxConf.bAutoFlushCrcErr = 1;
    /* Implement packet length filtering to avoid PROP_ERROR_RXBUF */
    RF_cmdPropRx.maxPktLen = MAX_LENGTH;
    RF_cmdPropRx.pktConf.bRepeatOk = 1;
    RF_cmdPropRx.pktConf.bRepeatNok = 1;

    /* Request access to the radio */
    rfHandle = RF_open(&rfObject, &RF_prop,
                       (RF_RadioSetup*)&RF_cmdPropRadioDivSetup, &rfParams);

    /* Set the frequency */
    RF_postCmd(rfHandle, (RF_Op*)&RF_cmdFs, RF_PriorityNormal, NULL, 0);

    /* Enter RX mode and stay forever in RX */
    RF_EventMask terminationReason = RF_runCmd(rfHandle, (RF_Op*)&RF_cmdPropRx,
                                               RF_PriorityNormal, &callback,
                                               RF_EventRxEntryDone);

    /* Call driver init functions */
    GPIO_init();
    UART_init();
    /* Configure the LED pin */
    GPIO_setConfig(Board_GPIO_LED0, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);

    /* Turn on user LED */
    GPIO_write(Board_GPIO_LED0, Board_GPIO_LED_ON);

    /* Create a UART with data processing off. */
    UART_Params_init(&uartParams);
    uartParams.writeDataMode = UART_DATA_BINARY;
    uartParams.readDataMode = UART_DATA_BINARY;
    uartParams.readReturnMode = UART_RETURN_FULL;
    uartParams.readEcho = UART_ECHO_OFF;
    uartParams.baudRate = 115200;

    uart = UART_open(Board_UART0, &uartParams);
    UART_write(uart, echoPrompt, sizeof(echoPrompt));
    UART_read(uart, &input, 1);
    UART_write(uart, &input, 1);

    return 0;
}

