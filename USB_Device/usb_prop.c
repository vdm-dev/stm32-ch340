/**
  ******************************************************************************
  * @file    usb_prop.c
  * @author  MCD Application Team
  * @version V4.0.0
  * @date    21-January-2013
  * @brief   All processing related to Virtual Com Port Demo
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2013 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */


#include "usb_lib.h"
#include "usb_conf.h"
#include "usb_prop.h"
#include "usb_desc.h"
#include "usb_pwr.h"
#include "hw_config.h"

#include "stdio.h"

LINE_CODING linecoding =
  {
    115200, /* baud rate*/
    0x00,   /* stop bits-1*/
    0x00,   /* parity - none*/
    0x08    /* no. of bits 8*/
  };


DEVICE Device_Table =
  {
    EP_NUM,
    1
  };

DEVICE_PROP Device_Property =
  {
    vcpInit,
    vcpReset,
    Virtual_Com_Port_Status_In,
    Virtual_Com_Port_Status_Out,
    Virtual_Com_Port_Data_Setup,
    Virtual_Com_Port_NoData_Setup,
    Virtual_Com_Port_Get_Interface_Setting,
    Virtual_Com_Port_GetDeviceDescriptor,
    Virtual_Com_Port_GetConfigDescriptor,
    Virtual_Com_Port_GetStringDescriptor,
    0,
    0x08 /*MAX PACKET SIZE*/
  };

USER_STANDARD_REQUESTS User_Standard_Requests =
  {
    Virtual_Com_Port_GetConfiguration,
    Virtual_Com_Port_SetConfiguration,
    Virtual_Com_Port_GetInterface,
    Virtual_Com_Port_SetInterface,
    Virtual_Com_Port_GetStatus,
    Virtual_Com_Port_ClearFeature,
    Virtual_Com_Port_SetEndPointFeature,
    Virtual_Com_Port_SetDeviceFeature,
    Virtual_Com_Port_SetDeviceAddress
  };

ONE_DESCRIPTOR Device_Descriptor =
  {
    (uint8_t*)Virtual_Com_Port_DeviceDescriptor,
    VIRTUAL_COM_PORT_SIZ_DEVICE_DESC
  };

ONE_DESCRIPTOR Config_Descriptor =
  {
    (uint8_t*)Virtual_Com_Port_ConfigDescriptor,
    VIRTUAL_COM_PORT_SIZ_CONFIG_DESC
  };

ONE_DESCRIPTOR String_Descriptor[4] =
  {
    {(uint8_t*)Virtual_Com_Port_StringLangID, VIRTUAL_COM_PORT_SIZ_STRING_LANGID},
    {(uint8_t*)Virtual_Com_Port_StringVendor, VIRTUAL_COM_PORT_SIZ_STRING_VENDOR},
    {(uint8_t*)Virtual_Com_Port_StringProduct, VIRTUAL_COM_PORT_SIZ_STRING_PRODUCT},
    {(uint8_t*)Virtual_Com_Port_StringSerial, VIRTUAL_COM_PORT_SIZ_STRING_SERIAL}
  };

uint8_t vendorVersion[2] = {0x30, 0x00};
uint8_t vendorAttach[2] = {0xC3, 0x00};
uint8_t vendorStatus[2] = {0xFF, 0xEE};

#define CH341_BAUDRATE_FACTOR  1532620800
#define CH341_BAUDRATE_DIVMAX  3

#define CH341_REQ_READ_REG     0x95
#define CH341_REQ_WRITE_REG    0x9A
#define CH341_REQ_SERIAL_INIT  0xA1
#define CH341_REQ_MODEM_CTRL   0xA4
#define CH341_REQ_READ_VERSION 0x5F

#define CH341_COMMAND_SET_BAUD 0x1213
#define CH341_COMMAND_SET_MODE 0x1825
#define CH341_COMMAND_GET_MODE 0x1825

uint8_t *vendorRequestVersion(uint16_t Length)
{
  if (Length == 0)
  {
    pInformation->Ctrl_Info.Usb_wLength = sizeof(vendorVersion);
    return NULL;
  }
  return (uint8_t *) &vendorVersion;
}

uint8_t *vendorRequestReadAttach(uint16_t Length)
{
  if (Length == 0)
  {
    pInformation->Ctrl_Info.Usb_wLength = sizeof(vendorAttach);
    return NULL;
  }
  return (uint8_t *) &vendorAttach;
}

uint8_t *vendorRequestReadStatus(uint16_t Length)
{
  if (Length == 0)
  {
    pInformation->Ctrl_Info.Usb_wLength = sizeof(vendorStatus);
    return NULL;
  }
  return (uint8_t *) &vendorStatus;
}

void vcpSetBaud(uint8_t factor, uint8_t divisor)
{
    printf("vcpSetBaud() f: %X, d: %X, ", factor, divisor);

    uint32_t baudrate = 0x10000 - (factor << 8);

    divisor = CH341_BAUDRATE_DIVMAX - divisor;

    while (divisor > 0)
    {
        baudrate <<= 3;
        divisor--;
    }

    baudrate = CH341_BAUDRATE_FACTOR / baudrate;

    printf("baudrate: %lu\r\n", baudrate);
}

void vcpSetMode(uint8_t mode)
{
    printf("vcpSetMode() data bits: %d, stop bits: %d, parity: %s%s\r\n",
            (mode & 0x03) + 5,
            ((mode & 0x04) >> 2) + 1,
            (mode & 0x08) ? ((mode & 0x10) ? "EVEN" : "ODD") : "NONE",
            (mode & 0x08) ? ((mode & 0x20) ? " (MARK)" : " (SPACE)") : "");

}


void vcpInit(void)
{
    // Update the serial number string descriptor with the data from the unique ID
    Get_SerialNum();

    pInformation->Current_Configuration = 0;

    // Connect the device
    PowerOn();

    // Perform basic device initialization operations
    USB_SIL_Init();

    setUsartDefaultSettings();

    bDeviceState = UNCONNECTED;
}

void vcpReset(void)
{
    // Set Virtual_Com_Port DEVICE as not configured
    pInformation->Current_Configuration = 0;

    // Current Feature initialization
    pInformation->Current_Feature = Virtual_Com_Port_ConfigDescriptor[7];

    /* Set Virtual_Com_Port DEVICE with the default Interface*/
    pInformation->Current_Interface = 0;

    SetBTABLE(BTABLE_ADDRESS);

    /* Initialize Endpoint 0 */
    SetEPType(ENDP0, EP_CONTROL);
    SetEPTxStatus(ENDP0, EP_TX_STALL);
    SetEPRxAddr(ENDP0, ENDP0_RXADDR);
    SetEPTxAddr(ENDP0, ENDP0_TXADDR);
    Clear_Status_Out(ENDP0);
    SetEPRxCount(ENDP0, Device_Property.MaxPacketSize);
    SetEPRxValid(ENDP0);

    /* Initialize Endpoint 2 IN/OUT */
    SetEPType(ENDP2, EP_BULK);
    SetEPTxAddr(ENDP2, ENDP2_TXADDR);
    SetEPTxStatus(ENDP2, EP_TX_NAK);
    SetEPRxAddr(ENDP2, ENDP2_RXADDR);
    SetEPRxCount(ENDP2, VIRTUAL_COM_PORT_DATA_SIZE);
    SetEPRxStatus(ENDP2, EP_RX_VALID);

    /* Initialize Endpoint 1 IN */
    SetEPType(ENDP1, EP_INTERRUPT);
    SetEPTxAddr(ENDP1, ENDP1_TXADDR);
    SetEPTxStatus(ENDP1, EP_TX_NAK);
    SetEPRxStatus(ENDP1, EP_RX_DIS);

    /* Set this device to response on default address */
    SetDeviceAddress(0);

    bDeviceState = ATTACHED;
}

void Virtual_Com_Port_SetConfiguration(void)
{
    DEVICE_INFO *pInfo = &Device_Info;

    if (pInfo->Current_Configuration != 0)
    {
        bDeviceState = CONFIGURED;
    }
}

void Virtual_Com_Port_SetDeviceAddress (void)
{
    bDeviceState = ADDRESSED;
}

void Virtual_Com_Port_Status_In(void)
{
}

void Virtual_Com_Port_Status_Out(void)
{
}

RESULT Virtual_Com_Port_Data_Setup(uint8_t request)
{
    uint8_t *(*CopyRoutine)( uint16_t);

    CopyRoutine = NULL;

    if (request == CH341_REQ_READ_VERSION)
    {
        printf("VENDOR_VERSION\r\n");

        CopyRoutine = vendorRequestVersion;
    }
    else if (request == CH341_REQ_READ_REG)
    {
        printf("VENDOR_READ, value: %X (%X %X), index: %X (%X %X)\r\n",
                pInformation->USBwValues.w,
                pInformation->USBwValues.bw.bb0,
                pInformation->USBwValues.bw.bb1,
                pInformation->USBwIndexs.w,
                pInformation->USBwIndexs.bw.bb0,
                pInformation->USBwIndexs.bw.bb1);

        if (pInformation->USBwValues.w == 0x1825)
        {
            CopyRoutine = vendorRequestReadAttach;
        }
        else if (pInformation->USBwValues.w == 0x0607)
        {
            CopyRoutine = vendorRequestReadStatus;
        }
    }

    if (CopyRoutine == NULL)
    {
        printf("Virtual_Com_Port_Data_Setup(RequestNo: %u)\r\n", request);
        return USB_UNSUPPORT;
    }

    pInformation->Ctrl_Info.CopyData = CopyRoutine;
    pInformation->Ctrl_Info.Usb_wOffset = 0;
    (*CopyRoutine)(0);
    return USB_SUCCESS;
}

RESULT Virtual_Com_Port_NoData_Setup(uint8_t request)
{
    if (request == CH341_REQ_SERIAL_INIT)
    {
        printf("VENDOR_SERIAL_INIT, value: %X (%X %X), index: %X (%X %X)\r\n",
                pInformation->USBwValues.w,
                pInformation->USBwValues.bw.bb0,
                pInformation->USBwValues.bw.bb1,
                pInformation->USBwIndexs.w,
                pInformation->USBwIndexs.bw.bb0,
                pInformation->USBwIndexs.bw.bb1);

        return USB_SUCCESS;
    }
    else if (request == CH341_REQ_WRITE_REG)
    {
        if (pInformation->USBwValues.w == CH341_COMMAND_SET_BAUD)
        {
            vcpSetBaud(pInformation->USBwIndexs.bw.bb1, pInformation->USBwIndexs.bw.bb0);
        }
        else if (pInformation->USBwValues.w == CH341_COMMAND_SET_MODE)
        {
            vcpSetMode(pInformation->USBwIndexs.bw.bb0);
        }
        else
        {
            printf("VENDOR_WRITE, value: %X (%X %X), index: %X (%X %X)\r\n",
                    pInformation->USBwValues.w,
                    pInformation->USBwValues.bw.bb0,
                    pInformation->USBwValues.bw.bb1,
                    pInformation->USBwIndexs.w,
                    pInformation->USBwIndexs.bw.bb0,
                    pInformation->USBwIndexs.bw.bb1);
        }

        return USB_SUCCESS;
    }
    else if (request == CH341_REQ_MODEM_CTRL)
    {
        printf("VENDOR_MODEM_OUT, value: %X (%X %X), index: %X (%X %X)\r\n",
                pInformation->USBwValues.w,
                pInformation->USBwValues.bw.bb0,
                pInformation->USBwValues.bw.bb1,
                pInformation->USBwIndexs.w,
                pInformation->USBwIndexs.bw.bb0,
                pInformation->USBwIndexs.bw.bb1);

        return USB_SUCCESS;
    }
    else
    {
        printf("Virtual_Com_Port_NoData_Setup(RequestNo: %u)\r\n", request);
    }

    return USB_UNSUPPORT;
}

uint8_t *Virtual_Com_Port_GetDeviceDescriptor(uint16_t Length)
{
  return Standard_GetDescriptorData(Length, &Device_Descriptor);
}

uint8_t *Virtual_Com_Port_GetConfigDescriptor(uint16_t Length)
{
  return Standard_GetDescriptorData(Length, &Config_Descriptor);
}

uint8_t *Virtual_Com_Port_GetStringDescriptor(uint16_t Length)
{
  uint8_t wValue0 = pInformation->USBwValue0;
  if (wValue0 > 4)
  {
    return NULL;
  }
  else
  {
    return Standard_GetDescriptorData(Length, &String_Descriptor[wValue0]);
  }
}

RESULT Virtual_Com_Port_Get_Interface_Setting(uint8_t Interface, uint8_t AlternateSetting)
{
  if (AlternateSetting > 0)
  {
    return USB_UNSUPPORT;
  }
  else if (Interface > 1)
  {
    return USB_UNSUPPORT;
  }
  return USB_SUCCESS;
}
