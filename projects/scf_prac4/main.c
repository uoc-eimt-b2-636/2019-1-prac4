/*
 * Copyright (C) 2017 Universitat Oberta de Catalunya - http://www.uoc.edu/
 *
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Universitat Oberta de Catalunya nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

/*----------------------------------------------------------------------------*/

/* Standard includes */
#include <stdlib.h>
#include <stdio.h>

/* Free-RTOS includes */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "portmacro.h"

/* Launchpad, Wi-Fi and Sensors includes */
#include "msp432_launchpad_board.h"
#include "edu_boosterpack_microphone.h"
#include "cc3100_boosterpack.h"

/*----------------------------------------------------------------------------*/

#define SPAWN_TASK_PRIORITY     ( tskIDLE_PRIORITY + 7 )
#define MAIN_TASK_PRIORITY      ( tskIDLE_PRIORITY + 2 )
#define BLINK_TASK_PRIORITY     ( tskIDLE_PRIORITY + 1 )

#define MAIN_STACK_SIZE         ( 2048 )
#define BLINK_STACK_SIZE        ( 128 )

#define SERVER_ADDRESS          ( SL_IPV4_VAL(192,168,1,53) )
#define TCP_PORT_NUMBER         ( 8080 )

#define TX_BUFFER_SIZE          ( 512 )
#define RX_BUFFER_SIZE          ( 512 )

/*----------------------------------------------------------------------------*/

static SemaphoreHandle_t semaphore;

static uint8_t tx_buffer[TX_BUFFER_SIZE];
static uint8_t rx_buffer[RX_BUFFER_SIZE];
static const uint8_t tx_buffer_size = sizeof(tx_buffer);
static const uint8_t rx_buffer_size = sizeof(rx_buffer);

/*----------------------------------------------------------------------------*/

static void BlinkTask(void *pvParameters);
static void MainTask(void *pvParameters);
static void microphone_interrupt(void);

/*----------------------------------------------------------------------------*/

static void BlinkTask(void *pvParameters) {
    while(true)
    {
        /* Turn red LED on */
        led_red_on();

        /* Sleep for 10 ms */
        vTaskDelay(pdMS_TO_TICKS(10));

        /* Turn red LED on */
        led_red_off();

        /* Sleep for 990 ms */
        vTaskDelay(pdMS_TO_TICKS(990));
    }
}

static void MainTask(void *pvParameters) {
    SlSockAddrIn_t socket_addr;

    int32_t retVal = 0;
    int16_t udp_socket = 0;
    int16_t length;

    /* Start the microphone */
    edu_boosterpack_microphone_init();
    edu_boosterpack_microphone_callback_set(microphone_interrupt);

    /* Restore Wi-Fi */
    retVal = wifi_restore();
    if (retVal < 0) {
        led_red_on();
        while(1);
    }

    /* Initialize Wi-Fi */
    retVal = wifi_init();
    if (retVal < 0) {
        led_red_on();
        while(1);
    }

    /* Create a TCP socket */
    wifi_set_socket_address(&socket_addr, SERVER_ADDRESS, TCP_PORT_NUMBER, true);
    udp_socket = wifi_udp_client_open(&socket_addr);
    if (udp_socket < 0) {
        led_red_on();
        while(1);
    }

    /* Start collecting samples from the microphone */
    edu_boosterpack_microphone_start();

    while (true)
    {
        /* Wait to be notified from interrupt */
        if (xSemaphoreTake(semaphore, portMAX_DELAY) == pdTRUE)
        {
            /* Turn green LED on */
            led_green_on();

            /* Stop collecting samples from the microphone */
            edu_boosterpack_microphone_stop();

            /* Get data from microphone */
            uint8_t* data_buffer;
            uint16_t data_length;
            data_length = edu_boosterpack_microphone_get_data(&data_buffer);

            /* Send audio data through UDP socket */
            retVal = wifi_udp_client_send(udp_socket, &socket_addr, data_buffer, data_length);
            if (retVal < 0)
            {
                led_red_on();
                while(1);
            }

            /* Stop collecting samples from the microphone */
            edu_boosterpack_microphone_restart();
            edu_boosterpack_microphone_start();

            /* Turn green LED off */
            led_green_off();
        }
    }
}

/*----------------------------------------------------------------------------*/

int main(int argc, char** argv)
{
    int32_t retVal;

    /* Initialize the board */
    board_init();

    /*
     * Start the SimpleLink task
     */
    retVal = VStartSimpleLinkSpawnTask(SPAWN_TASK_PRIORITY);
    if (retVal < 0)
    {
        led_red_on();
        while(1);
    }

    /*
     * Create binary semaphore to
     */
    semaphore = xSemaphoreCreateBinary();
    if (semaphore == NULL)
    {
        led_red_on();
        while(1);
    }

    /* Create blink task */
    retVal = xTaskCreate(BlinkTask,
                         "BlinkTask",
                         BLINK_STACK_SIZE,
                         NULL,
                         BLINK_TASK_PRIORITY,
                         NULL );
    if (retVal < 0)
    {
        led_red_on();
        while(1);
    }

    /* Create communication task */
    retVal = xTaskCreate(MainTask,
                         "MainTask",
                         MAIN_STACK_SIZE,
                         NULL,
                         MAIN_TASK_PRIORITY,
                         NULL );
    if (retVal < 0)
    {
        led_red_on();
        while (1);
    }

    /* Start the task scheduler */
    vTaskStartScheduler();

    return 0;
}

/*----------------------------------------------------------------------------*/

static void microphone_interrupt(void)
{
    static BaseType_t xHigherPriorityTaskWoken;

    /* Unblock the task by releasing the semaphore. */
    xSemaphoreGiveFromISR(semaphore, &xHigherPriorityTaskWoken);

    if (xHigherPriorityTaskWoken == pdTRUE) {
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}
