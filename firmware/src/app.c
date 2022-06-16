/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    app.c

  Summary:
    This file contains the source code for the MPLAB Harmony application.

  Description:
    This file contains the source code for the MPLAB Harmony application.  It 
    implements the logic of the application's state machine and it may call 
    API routines of other MPLAB Harmony modules in the system, such as drivers,
    system services, and middleware.  However, it does not call any of the
    system interfaces (such as the "Initialize" and "Tasks" functions) of any of
    the modules in the system or make any assumptions about when those functions
    are called.  That is the responsibility of the configuration-specific system
    files.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2014 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 *******************************************************************************/
// DOM-IGNORE-END


// *****************************************************************************
// *****************************************************************************
// Section: Included Files 
// *****************************************************************************
// *****************************************************************************

#include "system_definitions.h"
#include "app.h"

// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************

/* Recieve data buffer */
uint8_t receiveDataBuffer[64] APP_MAKE_BUFFER_DMA_READY;

/* Transmit data buffer */
uint8_t  transmitDataBuffer[64] APP_MAKE_BUFFER_DMA_READY;

// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    This structure should be initialized by the APP_Initialize function.
    
    Application strings and buffers are be defined outside this structure.
*/

APP_DATA appData;
int dato1, dato2, respuesta;
unsigned char dato1Buffer[4]={0,0,0,0};
unsigned char dato2Buffer[4]={0,0,0,0};
unsigned char respuestaBuffer[4]={0,0,0,0};
int *dato2BufferPtr=NULL;
float *respuestafPtr=NULL;
float *datof1Ptr, *datof2Ptr, respuestaf;
float datof1, datof2, respuestaf;

int j;
int row_index;

#define MATRIX_DIM 8
float mtx_A[MATRIX_DIM][MATRIX_DIM];
float mtx_B[MATRIX_DIM][MATRIX_DIM];
float mtx_C[MATRIX_DIM][MATRIX_DIM];

bool NeedReattach;

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

void reset_matrix(float mtx[MATRIX_DIM][MATRIX_DIM]) {
    memset(mtx, 0, MATRIX_DIM*sizeof(float));
}

void reset_all_matrices() {
    reset_matrix(mtx_A);
    reset_matrix(mtx_B);
    reset_matrix(mtx_C);
}

void multiply_matrices() {
    int i, j, k;
    for(i = 0; i < MATRIX_DIM; i++)    
    {    
        for(j = 0; j < MATRIX_DIM; j++)    
        {   
            mtx_C[i][j] = 0;    
            for(k = 0; k < MATRIX_DIM; k++)    
            {
                mtx_C[i][j] += mtx_A[i][k] * mtx_B[k][j];    
            }    
        }    
    }   
}

USB_DEVICE_HID_EVENT_RESPONSE APP_USBDeviceHIDEventHandler
(
    USB_DEVICE_HID_INDEX iHID,
    USB_DEVICE_HID_EVENT event,
    void * eventData,
    uintptr_t userData
)
{
    USB_DEVICE_HID_EVENT_DATA_REPORT_SENT * reportSent;
    USB_DEVICE_HID_EVENT_DATA_REPORT_RECEIVED * reportReceived;

    /* Check type of event */
    switch (event)
    {
        case USB_DEVICE_HID_EVENT_REPORT_SENT:

            /* The eventData parameter will be USB_DEVICE_HID_EVENT_REPORT_SENT
             * pointer type containing details about the report that was
             * sent. */
            reportSent = (USB_DEVICE_HID_EVENT_DATA_REPORT_SENT *) eventData;
            if(reportSent->handle == appData.txTransferHandle )
            {
                // Transfer progressed.
                appData.hidDataTransmitted = true;
            }
            
            break;

        case USB_DEVICE_HID_EVENT_REPORT_RECEIVED:

            /* The eventData parameter will be USB_DEVICE_HID_EVENT_REPORT_RECEIVED
             * pointer type containing details about the report that was
             * received. */

            reportReceived = (USB_DEVICE_HID_EVENT_DATA_REPORT_RECEIVED *) eventData;
            if(reportReceived->handle == appData.rxTransferHandle )
            {
                // Transfer progressed.
                appData.hidDataReceived = true;
            }
          
            break;

        case USB_DEVICE_HID_EVENT_SET_IDLE:

            /* For now we just accept this request as is. We acknowledge
             * this request using the USB_DEVICE_HID_ControlStatus()
             * function with a USB_DEVICE_CONTROL_STATUS_OK flag */

            USB_DEVICE_ControlStatus(appData.usbDevHandle, USB_DEVICE_CONTROL_STATUS_OK);

            /* Save Idle rate recieved from Host */
            appData.idleRate = ((USB_DEVICE_HID_EVENT_DATA_SET_IDLE*)eventData)->duration;
            break;

        case USB_DEVICE_HID_EVENT_GET_IDLE:

            /* Host is requesting for Idle rate. Now send the Idle rate */
            USB_DEVICE_ControlSend(appData.usbDevHandle, & (appData.idleRate),1);

            /* On successfully reciveing Idle rate, the Host would acknowledge back with a
               Zero Length packet. The HID function drvier returns an event
               USB_DEVICE_HID_EVENT_CONTROL_TRANSFER_DATA_SENT to the application upon
               receiving this Zero Length packet from Host.
               USB_DEVICE_HID_EVENT_CONTROL_TRANSFER_DATA_SENT event indicates this control transfer
               event is complete */

            break;
        default:
            // Nothing to do.
            break;
    }
    return USB_DEVICE_HID_EVENT_RESPONSE_NONE;
}

void APP_USBDeviceEventHandler(USB_DEVICE_EVENT event, void * eventData, uintptr_t context)
{
    switch(event)
    {
        case USB_DEVICE_EVENT_RESET:
        case USB_DEVICE_EVENT_DECONFIGURED:

            /* Host has de configured the device or a bus reset has happened.
             * Device layer is going to de-initialize all function drivers.
             * Hence close handles to all function drivers (Only if they are
             * opened previously. */
           
            BSP_LEDOn  (APP_USB_LED_1);
            BSP_LEDOn  (APP_USB_LED_2);
            BSP_LEDOff (APP_USB_LED_3);
            appData.deviceConfigured = false;
            appData.state = APP_STATE_WAIT_FOR_CONFIGURATION;
            break;

        case USB_DEVICE_EVENT_CONFIGURED:
            /* Set the flag indicating device is configured. */
            appData.deviceConfigured = true;

            /* Save the other details for later use. */
            appData.configurationValue = ((USB_DEVICE_EVENT_DATA_CONFIGURED*)eventData)->configurationValue;

            /* Register application HID event handler */
            USB_DEVICE_HID_EventHandlerSet(USB_DEVICE_HID_INDEX_0, APP_USBDeviceHIDEventHandler, (uintptr_t)&appData);

            /* Update the LEDs */
            BSP_LEDOff (APP_USB_LED_1);
            BSP_LEDOff (APP_USB_LED_2);
            BSP_LEDOn  (APP_USB_LED_3);

            break;

        case USB_DEVICE_EVENT_SUSPENDED:

            /* Switch on green and orange, switch off red */
            BSP_LEDOff (APP_USB_LED_1);
            BSP_LEDOn  (APP_USB_LED_2);
            BSP_LEDOn  (APP_USB_LED_3);
            break;
            
        case USB_DEVICE_EVENT_POWER_DETECTED:

            /* VBUS was detected. We can attach the device */

            USB_DEVICE_Attach (appData.usbDevHandle);
            break;

        case USB_DEVICE_EVENT_POWER_REMOVED:

            /* VBUS is not available */
            USB_DEVICE_Detach(appData.usbDevHandle);
            break;

        /* These events are not used in this demo */
        case USB_DEVICE_EVENT_RESUMED:
        case USB_DEVICE_EVENT_ERROR:
        default:
            break;
    }
}

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************

/* TODO:  Add any necessary local functions.
*/


// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void APP_Initialize ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    appData.state = APP_STATE_INIT;
    
    appData.usbDevHandle = USB_DEVICE_HANDLE_INVALID;
    appData.deviceConfigured = false;
    appData.txTransferHandle = USB_DEVICE_HID_TRANSFER_HANDLE_INVALID;
    appData.rxTransferHandle = USB_DEVICE_HID_TRANSFER_HANDLE_INVALID;
    appData.hidDataReceived = false;
    appData.hidDataTransmitted = true;
    appData.receiveDataBuffer = &receiveDataBuffer[0];
    appData.transmitDataBuffer = &transmitDataBuffer[0];
    
    reset_all_matrices();
}


/******************************************************************************
  Function:
    void APP_Tasks ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Tasks (void )
{
    /* Check if device is configured.  See if it is configured with correct
     * configuration value  */

    switch(appData.state)
    {
        case APP_STATE_INIT:

            /* Open the device layer */
            appData.usbDevHandle = USB_DEVICE_Open( USB_DEVICE_INDEX_0, DRV_IO_INTENT_READWRITE );

            if(appData.usbDevHandle != USB_DEVICE_HANDLE_INVALID)
            {
                /* Register a callback with device layer to get event notification (for end point 0) */
                USB_DEVICE_EventHandlerSet(appData.usbDevHandle, APP_USBDeviceEventHandler, 0);

                appData.state = APP_STATE_WAIT_FOR_CONFIGURATION;
            }
            else
            {
                /* The Device Layer is not ready to be opened. We should try
                 * again later. */
            }

            break;

        case APP_STATE_WAIT_FOR_CONFIGURATION:

            if(appData.deviceConfigured == true)
            {
                /* Device is ready to run the main task */
                appData.hidDataReceived = false;
                appData.hidDataTransmitted = true;
                appData.state = APP_STATE_MAIN_TASK;

                /* Place a new read request. */
                USB_DEVICE_HID_ReportReceive (USB_DEVICE_HID_INDEX_0,
                        &appData.rxTransferHandle, appData.receiveDataBuffer, 64);
            }
            break;
            
        case APP_STATE_MAIN_TASK:

            if(!appData.deviceConfigured)
            {
                /* Device is not configured */
                appData.state = APP_STATE_WAIT_FOR_CONFIGURATION;
            }
            else if( appData.hidDataReceived )
            {
                /* Look at the data the host sent, to see what
                 * kind of application specific command it sent. */

switch(appData.receiveDataBuffer[0])
{
    case 0x10:
        // Receive matrix A
        if (appData.hidDataTransmitted) {
            row_index = appData.receiveDataBuffer[1]; // Row index is sent after the command

            appData.transmitDataBuffer[0] = 0x10;
            appData.transmitDataBuffer[1] = row_index;    
            
            for(j = 0; j < MATRIX_DIM; j++) {
                // Guardar dato en matriz A
                dato1Buffer[0] = appData.receiveDataBuffer[4*j+2];
                dato1Buffer[1] = appData.receiveDataBuffer[4*j+3];
                dato1Buffer[2] = appData.receiveDataBuffer[4*j+4];
                dato1Buffer[3] = appData.receiveDataBuffer[4*j+5];
                datof1Ptr = (float*)&dato1Buffer[0];
                mtx_A[row_index][j] = *datof1Ptr;
                
                // Echo back same number
                appData.transmitDataBuffer[4*j+2] = dato1Buffer[0];
                appData.transmitDataBuffer[4*j+3] = dato1Buffer[1];
                appData.transmitDataBuffer[4*j+4] = dato1Buffer[2];
                appData.transmitDataBuffer[4*j+5] = dato1Buffer[3];
            }
            
            // Enviar respuesta  
            appData.hidDataTransmitted = false;
            USB_DEVICE_HID_ReportSend(USB_DEVICE_HID_INDEX_0,
                &appData.txTransferHandle, appData.transmitDataBuffer, 64);
            // Listo para recibir siguiente fila
            appData.hidDataReceived = false;
            USB_DEVICE_HID_ReportReceive(USB_DEVICE_HID_INDEX_0,
                &appData.rxTransferHandle, appData.receiveDataBuffer, 64);
        }
        appData.state = APP_STATE_MAIN_TASK; 
        break;
    case 0x20:
        // Receive matrix B
        if (appData.hidDataTransmitted) {
            row_index = appData.receiveDataBuffer[1]; // Row index is sent after the command
            
            appData.transmitDataBuffer[0] = 0x20;
            appData.transmitDataBuffer[1] = row_index;
            
            for(j = 0; j < MATRIX_DIM; j++) {
                // Guardar dato en matriz B
                dato2Buffer[0] = appData.receiveDataBuffer[4*j+2];
                dato2Buffer[1] = appData.receiveDataBuffer[4*j+3];
                dato2Buffer[2] = appData.receiveDataBuffer[4*j+4];
                dato2Buffer[3] = appData.receiveDataBuffer[4*j+5];
                datof2Ptr = (float*)&dato2Buffer[0];
                mtx_B[row_index][j] = *datof2Ptr;
                
                // Echo back same number
                appData.transmitDataBuffer[4*j+2] = dato2Buffer[0];
                appData.transmitDataBuffer[4*j+3] = dato2Buffer[1];
                appData.transmitDataBuffer[4*j+4] = dato2Buffer[2];
                appData.transmitDataBuffer[4*j+5] = dato2Buffer[3];
            }
            
            // Enviar respuesta
            appData.hidDataTransmitted = false;
            USB_DEVICE_HID_ReportSend(USB_DEVICE_HID_INDEX_0,
                &appData.txTransferHandle, appData.transmitDataBuffer, 64);
            // Listo para recibir siguiente fila
            appData.hidDataReceived = false;
            USB_DEVICE_HID_ReportReceive(USB_DEVICE_HID_INDEX_0,
                &appData.rxTransferHandle, appData.receiveDataBuffer, 64);
        }
        appData.state = APP_STATE_MAIN_TASK;
        break;
    case 0x30:
        // Return matrix C
        if (appData.hidDataTransmitted) {
            row_index = appData.receiveDataBuffer[1];
            if (row_index == 0) {
                // Perform multiplication
                multiply_matrices(); 
            }
            
            // Write the result back to the host
            appData.transmitDataBuffer[0] = 0x30;
            appData.transmitDataBuffer[1] = row_index;
            
            
            for(j = 0; j < MATRIX_DIM; j++) {
                respuestaf = mtx_C[row_index][j];
                respuestafPtr = (float*)&respuestaBuffer[0];
                *respuestafPtr = respuestaf;
                
                // Store the answer into tx buffer
                appData.transmitDataBuffer[4*j+2] = respuestaBuffer[0];
                appData.transmitDataBuffer[4*j+3] = respuestaBuffer[1];
                appData.transmitDataBuffer[4*j+4] = respuestaBuffer[2];
                appData.transmitDataBuffer[4*j+5] = respuestaBuffer[3];
            }
            
            // Enviar respuesta
            appData.hidDataTransmitted = false;
            USB_DEVICE_HID_ReportSend(USB_DEVICE_HID_INDEX_0,
                &appData.txTransferHandle, appData.transmitDataBuffer, 64);
            // Listo para recibir siguiente fila
            appData.hidDataReceived = false;
            USB_DEVICE_HID_ReportReceive(USB_DEVICE_HID_INDEX_0,
                &appData.rxTransferHandle, appData.receiveDataBuffer, 64);
        }
        appData.state = APP_STATE_MAIN_TASK;
        break;
    default:
        appData.hidDataReceived = false;
        /* Place a new read request. */
        USB_DEVICE_HID_ReportReceive (USB_DEVICE_HID_INDEX_0,
                &appData.rxTransferHandle, appData.receiveDataBuffer, 64 );
        appData.state = APP_STATE_MAIN_TASK;        
        break;
        }
    }
    case APP_STATE_ERROR:
        break;
    default:
        break;
    }
}
 

/*******************************************************************************
 End of File
 */

