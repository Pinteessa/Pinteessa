/* Standard C -kirjastot */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

/* XDCtools-tiedostot */
#include <xdc/std.h>
#include <xdc/runtime/System.h>

/* BIOS-otsikkotiedostot */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Task.h>

/* TI-RTOS-ajurit */
#include <ti/drivers/PIN.h>
#include <ti/drivers/pin/PINCC26XX.h>
#include <ti/drivers/I2C.h>
#include <ti/drivers/Power.h>
#include <ti/drivers/power/PowerCC26XX.h>
#include <ti/drivers/UART.h>
#include <ti/drivers/i2c/I2CCC26XX.h>

/* Laitteen otsikkotiedostot */
#include "Board.h"
#include "sensors/mpu9250.h" // MPU-anturin käyttöliittymä
#include "sensors/buzzer.h"  // Summerin käyttöliittymä

// Funktiomäärittelyt
void repeatMessageWithBuzzer(char *message);
void sensorTaskFxn(UArg arg0, UArg arg1);
void uartTaskFxn(UArg arg0, UArg arg1);
void uartSendTaskFxn(UArg arg0, UArg arg1);
void buttonCallbackFxn(PIN_Handle handle, PIN_Id pinId);

// Globaalit muuttujat
static bool buttonPressed = false;  // Painikkeen tilalippu
static char detectedSymbol = '\0';  // Havaitun merkin tallennus UART-lähetykseen
static UART_Handle uart = NULL;     // Yhteinen UART-kahva

/* Tehtävien muistialueiden määrittelyt */
#define STACKSIZE 2048
Char MPUTaskStack[STACKSIZE];
Char uartTaskStack[STACKSIZE];
Char uartSendTaskStack[STACKSIZE];

// Tilakone
enum state { IDLE, UART_RECEIVE, PROCESSING };
static enum state programState = IDLE;

/* MPU-virtapinnin asetukset */
static PIN_Handle hMpuPin;
static PIN_State MpuPinState;
static PIN_Config MpuPinConfig[] = {
    Board_MPU_POWER | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    PIN_TERMINATE
};

/* Summerin pinnin asetukset */
static PIN_Handle hBuzzerPin;
static PIN_State buzzerPinState;
static PIN_Config buzzerPinConfig[] = {
    Board_BUZZER | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    PIN_TERMINATE
};

/* Painikkeen pinnin asetukset */
static PIN_Handle hButtonPin;
static PIN_State buttonPinState;
static PIN_Config buttonConfig[] = {
    Board_BUTTON0 | PIN_INPUT_EN | PIN_PULLUP | PIN_IRQ_NEGEDGE,
    PIN_TERMINATE
};

/* Painikkeen keskeytyspalautteen funktio */
void buttonCallbackFxn(PIN_Handle handle, PIN_Id pinId) {
    if (pinId == Board_BUTTON0) {
        buttonPressed = true;  // Asetetaan lippu, kun painike havaitaan
    }
}

/* UART-vastaanottotehtävän funktio */
Void uartTaskFxn(UArg arg0, UArg arg1) {
    char received[4]; // Viestin puskuri
    int bytesRead;

    while (1) {
        switch (programState) {
        case IDLE:
            programState = UART_RECEIVE; // Vaihto vastaanottotilaan
            break;

        case UART_RECEIVE:
            memset(received, 0, sizeof(received));
            bytesRead = UART_read(uart, received, sizeof(received) - 1);
            if (bytesRead > 0) {
                received[bytesRead] = '\0'; // Päätetään viesti nollamerkillä
                System_printf("Received UART: %s\n", received);
                System_flush();
                programState = PROCESSING;
            } else {
                programState = IDLE; // Palataan IDLE-tilaan
            }
            break;

        case PROCESSING:
            repeatMessageWithBuzzer(received); // Toistetaan viesti summerilla
            programState = IDLE;
            break;

        default:
            programState = IDLE; // Oletustila
            break;
        }

        Task_sleep(50000 / Clock_tickPeriod); // Pieni viive
    }
}

/* UART-lähetystehtävän funktio */
Void uartSendTaskFxn(UArg arg0, UArg arg1) {
    char formattedSymbol[4]; // Muotoiltu symboli

    while (1) {
        if (detectedSymbol != '\0') {
            // Muotoillaan ja lähetetään havaittu symboli UARTin kautta
            sprintf(formattedSymbol, "%c\r\n\0", detectedSymbol);
            UART_write(uart, formattedSymbol, strlen(formattedSymbol) + 1);

            System_printf("Sent via UART: %c\n", detectedSymbol);
            System_flush();

            detectedSymbol = '\0'; // Tyhjennetään symboli seuraavaa havaintoa varten
        }

        Task_sleep(50000 / Clock_tickPeriod); // Tarkistetaan uudet symbolit
    }
}

/* Viestin toisto summerilla */
void repeatMessageWithBuzzer(char *message) {
    int i;
    for (i = 0; message[i] != '\0'; i++) {
        if (message[i] == '.') {
            buzzerSetFrequency(1000); // Piste: 1 kHz
            Task_sleep(200000 / Clock_tickPeriod);
            buzzerSetFrequency(0); // Summeri pois päältä
        } else if (message[i] == '-') {
            buzzerSetFrequency(500); // Viiva: 500 Hz
            Task_sleep(600000 / Clock_tickPeriod);
            buzzerSetFrequency(0);
        } else if (message[i] == ' ') {
            Task_sleep(600000 / Clock_tickPeriod); // Välilyönti
        }
        Task_sleep(200000 / Clock_tickPeriod); // Merkkien välinen tauko
    }
}

/* Sensoritehtävän funktio */
Void sensorTaskFxn(UArg arg0, UArg arg1) {
    I2C_Handle i2cMPU;
    I2C_Params i2cMPUParams;

    I2CCC26XX_I2CPinCfg i2cMPUCfg = {
        .pinSDA = Board_I2C0_SDA1,
        .pinSCL = Board_I2C0_SCL1
    };

    I2C_Params_init(&i2cMPUParams);
    i2cMPUParams.bitRate = I2C_400kHz;
    i2cMPUParams.custom = (uintptr_t)&i2cMPUCfg;

    PIN_setOutputValue(hMpuPin, Board_MPU_POWER, Board_MPU_POWER_ON);
    Task_sleep(100000 / Clock_tickPeriod);

    i2cMPU = I2C_open(Board_I2C, &i2cMPUParams);
    if (i2cMPU == NULL) {
        System_abort("Error initializing I2C MPU\n");
    }

    mpu9250_setup(&i2cMPU);
    System_printf("MPU9250: Setup complete\n");
    System_flush();

    float ax, ay, az, gx, gy, gz;

    while (1) {
        if (buttonPressed) {
            detectedSymbol = ' ';
            System_printf("Space detected\n");
            System_flush();
            buttonPressed = false;
        }

        mpu9250_get_data(&i2cMPU, &ax, &ay, &az, &gx, &gy, &gz);
        if (ay > 1.0) {
            detectedSymbol = '-'; // Viiva
            // System_printf("Dash detected\n");
        } else if (ax > 1.0) {
            detectedSymbol = '.'; // Piste
            // System_printf("Dot detected\n");
        }

        System_flush();
        Task_sleep(100000 / Clock_tickPeriod);
    }
}

/* Pääohjelma */
Int main(void) {
    Task_Handle MPUTaskHandle, uartTaskHandle, uartSendTaskHandle;
    Task_Params MPUTaskParams, uartTaskParams, uartSendTaskParams;

    Board_initGeneral();
    Board_initI2C();
    Board_initUART();

    UART_Params uartParams;
    UART_Params_init(&uartParams);
    uartParams.writeDataMode = UART_DATA_TEXT;
    uartParams.readDataMode = UART_DATA_TEXT;
    uartParams.readEcho = UART_ECHO_OFF;
    uartParams.baudRate = 9600;

    uart = UART_open(Board_UART0, &uartParams);
    if (uart == NULL) {
        System_abort("Error initializing UART\n");
    }

    hMpuPin = PIN_open(&MpuPinState, MpuPinConfig);
    if (hMpuPin == NULL) {
        System_abort("MPU Pin open failed!");
    }

    hBuzzerPin = PIN_open(&buzzerPinState, buzzerPinConfig);
    if (hBuzzerPin == NULL) {
        System_abort("Buzzer Pin open failed!");
    }
    buzzerOpen(hBuzzerPin);

    hButtonPin = PIN_open(&buttonPinState, buttonConfig);
    if (hButtonPin == NULL) {
        System_abort("Button Pin open failed!");
    }
    PIN_registerIntCb(hButtonPin, buttonCallbackFxn);

    Task_Params_init(&MPUTaskParams);
    MPUTaskParams.stackSize = STACKSIZE;
    MPUTaskParams.stack = &MPUTaskStack;
    MPUTaskParams.priority = 2;
    MPUTaskHandle = Task_create(sensorTaskFxn, &MPUTaskParams, NULL);
    if (MPUTaskHandle == NULL) {
        System_abort("MPU Task creation failed!");
    }

    Task_Params_init(&uartTaskParams);
    uartTaskParams.stackSize = STACKSIZE;
    uartTaskParams.stack = &uartTaskStack;
    uartTaskParams.priority = 2;
    uartTaskHandle = Task_create(uartTaskFxn, &uartTaskParams, NULL);
    if (uartTaskHandle == NULL) {
        System_abort("UART Task creation failed!");
    }

    Task_Params_init(&uartSendTaskParams);
    uartSendTaskParams.stackSize = STACKSIZE;
    uartSendTaskParams.stack = &uartSendTaskStack;
    uartSendTaskParams.priority = 2;
    uartSendTaskHandle = Task_create(uartSendTaskFxn, &uartSendTaskParams, NULL);
    if (uartSendTaskHandle == NULL) {
        System_abort("UART Send Task creation failed!");
    }

    BIOS_start();
    return 0;
}
