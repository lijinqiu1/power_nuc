/****************************************************************************
 * @file     main.c
 * @version  V2.0
 * $Revision: 1 $
 * $Date: 14/10/01 5:10p $
 * @brief    NUC230_240 Series CAN Driver Sample Code
 *
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>
#include "NUC230_240.h"

#define PLL_CLOCK       50000000

int32_t i32BatteryValue[5];

/*---------------------------------------------------------------------------*/
/*  Function Declare                                                         */
/*---------------------------------------------------------------------------*/
extern char GetChar(void);
void CAN_ShowMsg(STR_CANMSG_T* Msg);

/*---------------------------------------------------------------------------------------------------------*/
/*  Function for System Entry to Power-down Mode                                                           */
/*---------------------------------------------------------------------------------------------------------*/
void PowerDownFunction(void)
{
    printf("\nSystem enter to power-down mode ...\n");

    /* To check if all the debug messages are finished */
    while(IsDebugFifoEmpty() == 0);

    SCB->SCR = 4;

    /* To program PWRCON register, it needs to disable register protection first. */
    CLK->PWRCON = (CLK->PWRCON & ~(CLK_PWRCON_PWR_DOWN_EN_Msk | CLK_PWRCON_PD_WAIT_CPU_Msk)) |
                  CLK_PWRCON_PD_WAIT_CPU_Msk | CLK_PWRCON_PD_WU_INT_EN_Msk;
    CLK->PWRCON |= CLK_PWRCON_PWR_DOWN_EN_Msk;

    __WFI();
}

/*---------------------------------------------------------------------------------------------------------*/
/* Define global variables and constants                                                                   */
/*---------------------------------------------------------------------------------------------------------*/
/* Declare a CAN message structure */
STR_CANMSG_T rrMsg;


/**
 * @brief       Timer0 IRQ
 *
 * @param       None
 *
 * @return      None
 *
 * @details     The Timer0 default IRQ, declared in startup_NUC230_240.s.
 */
void TMR0_IRQHandler(void)
{
    if(TIMER_GetIntFlag(TIMER0) == 1) {
        /* Clear Timer0 time-out interrupt flag */
        TIMER_ClearIntFlag(TIMER0);
    }

    if(TIMER_GetWakeupFlag(TIMER0) == 1) {
        /* Clear Timer0 wake-up flag */
        TIMER_ClearWakeupFlag(TIMER0);
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/* ISR to handle CAN interrupt event                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
void CAN_MsgInterrupt(CAN_T *tCAN, uint32_t u32IIDR)
{
    if(u32IIDR == 1) {
        printf("Msg-0 INT and Callback\n");
        CAN_Receive(tCAN, 0, &rrMsg);
        CAN_ShowMsg(&rrMsg);
    }
    if(u32IIDR == 5 + 1) {
        printf("Msg-5 INT and Callback \n");
        CAN_Receive(tCAN, 5, &rrMsg);
        CAN_ShowMsg(&rrMsg);
    }
    if(u32IIDR == 31 + 1) {
        printf("Msg-31 INT and Callback \n");
        CAN_Receive(tCAN, 31, &rrMsg);
        CAN_ShowMsg(&rrMsg);
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/* CAN0 interrupt handler                                                                                  */
/*---------------------------------------------------------------------------------------------------------*/
void CAN0_IRQHandler(void)
{
    uint32_t u8IIDRstatus;

    u8IIDRstatus = CAN0->IIDR;

    if(u8IIDRstatus == 0x00008000) {      /* Check Status Interrupt Flag (Error status Int and Status change Int) */
        /**************************/
        /* Status Change interrupt*/
        /**************************/
        if(CAN0->STATUS & CAN_STATUS_RXOK_Msk) {
            CAN0->STATUS &= ~CAN_STATUS_RXOK_Msk;   /* Clear RxOK status*/

            printf("RxOK INT\n") ;
        }

        if(CAN0->STATUS & CAN_STATUS_TXOK_Msk) {
            CAN0->STATUS &= ~CAN_STATUS_TXOK_Msk;    /* Clear TxOK status*/

            printf("TxOK INT\n") ;
        }

        /**************************/
        /* Error Status interrupt */
        /**************************/
        if(CAN0->STATUS & CAN_STATUS_EWARN_Msk) {
            printf("EWARN INT\n") ;
        }

        if(CAN0->STATUS & CAN_STATUS_BOFF_Msk) {
            printf("BOFF INT\n") ;
        }
    } else if(u8IIDRstatus != 0) {
        printf("=> Interrupt Pointer = %d\n", CAN0->IIDR - 1);

        CAN_MsgInterrupt(CAN0, u8IIDRstatus);

        CAN_CLR_INT_PENDING_BIT(CAN0, ((CAN0->IIDR) - 1)); /* Clear Interrupt Pending */
    } else if(CAN0->WU_STATUS == 1) {
        printf("Wake up\n");

        CAN0->WU_STATUS = 0;     /* Write '0' to clear */
    }

}

/*---------------------------------------------------------------------------------------------------------*/
/* CAN1 interrupt handler                                                                                  */
/*---------------------------------------------------------------------------------------------------------*/
void CAN1_IRQHandler(void)
{
    uint32_t u8IIDRstatus;

    u8IIDRstatus = CAN1->IIDR;

    if(u8IIDRstatus == 0x00008000) {      /* Check Status Interrupt Flag (Error status Int and Status change Int) */
        /**************************/
        /* Status Change interrupt*/
        /**************************/
        if(CAN1->STATUS & CAN_STATUS_RXOK_Msk) {
            CAN1->STATUS &= ~CAN_STATUS_RXOK_Msk;   /* Clear RxOK status*/

            printf("RxOK INT\n") ;
        }

        if(CAN1->STATUS & CAN_STATUS_TXOK_Msk) {
            CAN1->STATUS &= ~CAN_STATUS_TXOK_Msk;    /* Clear TxOK status*/

            printf("TxOK INT\n") ;
        }

        /**************************/
        /* Error Status interrupt */
        /**************************/
        if(CAN1->STATUS & CAN_STATUS_EWARN_Msk) {
            printf("EWARN INT\n") ;
        }

        if(CAN1->STATUS & CAN_STATUS_BOFF_Msk) {
            printf("BOFF INT\n") ;
        }
    } else if(u8IIDRstatus != 0) {
        printf("=> Interrupt Pointer = %d\n", CAN1->IIDR - 1);

        CAN_MsgInterrupt(CAN1, u8IIDRstatus);

        CAN_CLR_INT_PENDING_BIT(CAN1, ((CAN1->IIDR) - 1)); /* Clear Interrupt Pending */
    } else if(CAN1->WU_STATUS == 1) {
        printf("Wake up\n");

        CAN1->WU_STATUS = 0;     /* Write '0' to clear */
    }

}

/*---------------------------------------------------------------------------*/
/*  Show Message Function                                                    */
/*---------------------------------------------------------------------------*/
void CAN_ShowMsg(STR_CANMSG_T* Msg)
{
    uint8_t i;

    /* Show the message information */
    printf("Read ID=0x%X, Type=%s, DLC=%d, Data=", Msg->Id, Msg->IdType ? "EXT" : "STD", Msg->DLC);
    for(i = 0; i < Msg->DLC; i++)
        printf("%X,", Msg->Data[i]);
    printf("\n\n");
}

/*---------------------------------------------------------------------------------------------------------*/
/* Disable CAN Clock and Reset it                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
void CAN_STOP(void)
{
    /* Disable CAN0 Clock and Reset it */
//    SYS_ResetModule(CAN0_RST);
//    CLK_DisableModuleClock(CAN0_MODULE);

    /* Disable CAN1 Clock and Reset it */
    SYS_ResetModule(CAN1_RST);
    CLK_DisableModuleClock(CAN1_MODULE);
}

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable Internal RC 22.1184MHz clock */
    CLK_EnableXtalRC(CLK_PWRCON_OSC22M_EN_Msk);

    /* Waiting for Internal RC clock ready */
    CLK_WaitClockReady(CLK_CLKSTATUS_OSC22M_STB_Msk);

    /* Switch HCLK clock source to Internal RC and HCLK source divide 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLK_S_HIRC, CLK_CLKDIV_HCLK(1));

    /* Enable external XTAL 12MHz clock */
    CLK_EnableXtalRC(CLK_PWRCON_XTL12M_EN_Msk);

    /* Waiting for external XTAL clock ready */
    CLK_WaitClockReady(CLK_CLKSTATUS_XTL12M_STB_Msk);

    /* Set core clock as PLL_CLOCK from PLL */
    CLK_SetCoreClock(PLL_CLOCK);

    /* Enable UART module clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select UART module clock source */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART_S_PLL, CLK_CLKDIV_UART(1));

    /* Enable ADC module clock */
    CLK_EnableModuleClock(ADC_MODULE);

    /* ADC clock source is 22.1184MHz, set divider to 7, ADC clock is 22.1184/7 MHz */
    CLK_SetModuleClock(ADC_MODULE, CLK_CLKSEL1_ADC_S_HIRC, CLK_CLKDIV_ADC(7));

    /* Enable CAN module clock */
//    CLK_EnableModuleClock(CAN0_MODULE);
    CLK_EnableModuleClock(CAN1_MODULE);

    /* Enable Timer0 module clock */
    CLK_EnableModuleClock(TMR0_MODULE);    
    
    /* Select Timer0 module clock source */
    CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0_S_LIRC, NULL);
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set GPB multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFP &= ~(SYS_GPB_MFP_PB0_Msk | SYS_GPB_MFP_PB1_Msk);
    SYS->GPB_MFP |= SYS_GPB_MFP_PB0_UART0_RXD | SYS_GPB_MFP_PB1_UART0_TXD;

//    /* Set PD multi-function pins for CANTX0, CANRX0 */
//    SYS->GPD_MFP &= ~(SYS_GPD_MFP_PD6_Msk | SYS_GPD_MFP_PD7_Msk);
//    SYS->GPD_MFP = SYS_GPD_MFP_PD6_CAN0_RXD | SYS_GPD_MFP_PD7_CAN0_TXD;
	
    /* Disable the GPA0 - GPA3 digital input path to avoid the leakage current. */
    GPIO_DISABLE_DIGITAL_PATH(PA, 0x1F);

    /* Set PA multi-function pins for CANTX1, CANRX1 */
	SYS->GPA_MFP &= ~(SYS_GPA_MFP_PA10_Msk | SYS_GPA_MFP_PA11_Msk);
	SYS->GPA_MFP = SYS_GPA_MFP_PA10_CAN1_TXD | SYS_GPA_MFP_PA11_CAN1_RXD;
	SYS->ALT_MFP &= ~(SYS_ALT_MFP_PA10_Msk | SYS_ALT_MFP_PA11_Msk);
	SYS->ALT_MFP |= SYS_ALT_MFP_PA10_CAN1_TXD | SYS_ALT_MFP_PA11_CAN1_RXD;

    /* Configure the GPA0 - GPA3 ADC analog input pins */
    SYS->GPA_MFP &= ~(SYS_GPA_MFP_PA0_Msk | SYS_GPA_MFP_PA1_Msk | SYS_GPA_MFP_PA2_Msk | SYS_GPA_MFP_PA3_Msk | SYS_GPA_MFP_PA4_Msk) ;
    SYS->GPA_MFP |= SYS_GPA_MFP_PA0_ADC0 | SYS_GPA_MFP_PA1_ADC1 | SYS_GPA_MFP_PA2_ADC2 | SYS_GPA_MFP_PA3_ADC3 | SYS_GPA_MFP_PA4_ADC4 ;
    SYS->ALT_MFP1 = 0;

}

/*---------------------------------------------------------------------------------------------------------*/
/* Init UART                                                                                               */
/*---------------------------------------------------------------------------------------------------------*/
void UART0_Init()
{
    /* Reset IP */
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 Baudrate */
    UART_Open(UART0, 115200);
}

/*----------------------------------------------------------------------------*/
/*  Some description about how to create test environment                     */
/*----------------------------------------------------------------------------*/
void Note_Configure()
{
    printf("\n\n");
    printf("+--------------------------------------------------------------------------+\n");
    printf("|  About CAN sample code configure                                         |\n");
    printf("+--------------------------------------------------------------------------+\n");
    printf("|   The sample code provide a simple sample code for you study CAN         |\n");
    printf("|   Before execute it, please check description as below                   |\n");
    printf("|                                                                          |\n");
    printf("|   1.CAN_TX and CAN_RX should be connected to your CAN transceiver        |\n");
    printf("|   2.Using two module board and connect to the same CAN BUS               |\n");
    printf("|   3.Check the terminal resistor of bus is connected                      |\n");
    printf("|   4.Using UART0 as print message port                                    |\n");
    printf("|                                                                          |\n");
    printf("|  |--------|       |-----------|   CANBUS  |-----------|       |--------| |\n");
    printf("|  |        |------>|           |<--------->|           |<------|        | |\n");
    printf("|  |        |CAN_TX |    CAN    |   CAN_H   |   CAN     |CAN_TX |        | |\n");
    printf("|  | NUC2XX |       |Transceiver|           |Transceiver|       | NUC2XX | |\n");
    printf("|  |        |<------|           |<--------->|           |------>|        | |\n");
    printf("|  |        |CAN_RX |           |   CAN_L   |           |CAN_RX |        | |\n");
    printf("|  |--------|       |-----------|           |-----------|       |--------| |\n");
    printf("|  |                                                            |          |\n");
    printf("|  |                                                            |          |\n");
    printf("|  V                                                            V          |\n");
    printf("| UART0                                                         UART0      |\n");
    printf("|(print message)                                          (print message)  |\n");
    printf("+--------------------------------------------------------------------------+\n");
}

/*----------------------------------------------------------------------------*/
/*  Set the CAN speed                                                         */
/*----------------------------------------------------------------------------*/
void SelectCANSpeed(CAN_T  *tCAN)
{
    uint32_t unItem;
    int32_t i32Err = 0;

    // printf("Please select CAN speed you desired\n");
    // printf("[0] 1000Kbps\n");
    // printf("[1]  500Kbps\n");
    // printf("[2]  250Kbps\n");
    // printf("[3]  125Kbps\n");
    // printf("[4]  100Kbps\n");
    // printf("[5]   50Kbps\n");

    // unItem = GetChar();
    // printf("%c\n", unItem);
    // if(unItem == '1')
    //     i32Err = CAN_Open(tCAN,  500000, CAN_BASIC_MODE);//Set target baud-rate and operation mode.
    // else if(unItem == '2')
    //     i32Err = CAN_Open(tCAN,  250000, CAN_BASIC_MODE);
    // else if(unItem == '3')
    //     i32Err = CAN_Open(tCAN,  125000, CAN_BASIC_MODE);
    // else if(unItem == '4')
    //     i32Err = CAN_Open(tCAN,  100000, CAN_BASIC_MODE);
    // else if(unItem == '5')
    //     i32Err = CAN_Open(tCAN,   50000, CAN_BASIC_MODE);
    // else
        // i32Err = CAN_Open(tCAN, 1000000, CAN_BASIC_MODE);
    i32Err = CAN_Open(tCAN, 1000000, CAN_BASIC_MODE);

    if(i32Err < 0)
        printf("Set CAN bit rate is fail\n");
    else
        printf("Real baud-rate value(bps): %d\n", i32Err);

}

/*----------------------------------------------------------------------------*/
/*  Test Menu                                                                 */
/*----------------------------------------------------------------------------*/
void TestItem(void)
{
    printf("\n");
    printf("+------------------------------------------------------------------ +\n");
    printf("|  Nuvoton CAN BUS DRIVER DEMO                                      |\n");
    printf("+-------------------------------------------------------------------+\n");
    printf("|                                                                   |\n");
    printf("|     Transmit a message by basic mode                              |\n");
    printf("|     (At first, another module board should be set to              |\n");
    printf("|     [CAN_BasicMode_Receive] waiting for receiving data)           |\n");
    printf("|                                                                   |\n");
    printf("+-------------------------------------------------------------------+\n");
}

/*----------------------------------------------------------------------------*/
/*  Send Tx Msg by Basic Mode Function (Without Message RAM)                  */
/*----------------------------------------------------------------------------*/
void Test_BasicMode_Tx(CAN_T  *tCAN)
{
    int32_t delaycount;

    /* Declare a CAN message structure */
    STR_CANMSG_T msg1;
    delaycount=1000;

    /* Enable CAN interrupt */
    CAN_EnableInt(tCAN, CAN_CON_IE_Msk | CAN_CON_SIE_Msk);
    /* Set Interrupt Priority */
    NVIC_SetPriority(CAN0_IRQn, (1 << __NVIC_PRIO_BITS) - 2);
    NVIC_SetPriority(CAN1_IRQn, (1 << __NVIC_PRIO_BITS) - 2);
    /* Enable External Interrupt */
    NVIC_EnableIRQ(CAN0_IRQn);
    NVIC_EnableIRQ(CAN1_IRQn);

    /* Send Message No.1 */
    msg1.FrameType = CAN_DATA_FRAME;
    msg1.IdType   = CAN_STD_ID;
    msg1.Id       = 0x001;
    msg1.DLC      = 2;
    msg1.Data[0]  = 0x00;
    msg1.Data[1]  = 0x2;
    CAN_Transmit(tCAN, 0, &msg1);//Send CAN message
    printf("Send STD_ID:0x1,Data[0,2]\n");
    CLK_SysTickDelay(delaycount);   /* Generate the Delay Time by Systick */

    /* Send Message No.2 */
    msg1.FrameType = CAN_DATA_FRAME;
    msg1.IdType   = CAN_STD_ID;
    msg1.Id       = 0x1AC;
    msg1.DLC      = 8;
    msg1.Data[0]  = 0x11;
    msg1.Data[1]  = 0x12;
    msg1.Data[2]  = 0x13;
    msg1.Data[3]  = 0x14;
    msg1.Data[4]  = 0x15;
    msg1.Data[5]  = 0x16;
    msg1.Data[6]  = 0x17;
    msg1.Data[7]  = 0x18;
    CAN_Transmit(tCAN, 0, &msg1);//Send CAN message
    printf("Send STD_ID:0x1AC,Data[11,12,13,14,15,16,17,18]\n");
    CLK_SysTickDelay(delaycount);   /* Generate the Delay Time by Systick */

    /* Send Message No.3 */
    msg1.FrameType = CAN_DATA_FRAME;
    msg1.IdType   = CAN_STD_ID;
    msg1.Id       = 0x310;
    msg1.DLC      = 8;
    msg1.Data[0]  = 0x21;
    msg1.Data[1]  = 0x22;
    msg1.Data[2]  = 0x23;
    msg1.Data[3]  = 0x24;
    msg1.Data[4]  = 0x25;
    msg1.Data[5]  = 0x26;
    msg1.Data[6]  = 0x27;
    msg1.Data[7]  = 0x28;
    CAN_Transmit(tCAN, 0, &msg1);//Send CAN message
    printf("Send STD_ID:0x310,Data[21,22,23,24,25,26,27,28]\n");
    CLK_SysTickDelay(delaycount);   /* Generate the Delay Time by Systick */

    /* Send Message No.4 */
    msg1.FrameType = CAN_DATA_FRAME;
    msg1.IdType   = CAN_EXT_ID;
    msg1.Id       = 0x3377;
    msg1.DLC      = 8;
    msg1.Data[0]  = 0x31;
    msg1.Data[1]  = 0x32;
    msg1.Data[2]  = 0x33;
    msg1.Data[3]  = 0x34;
    msg1.Data[4]  = 0x35;
    msg1.Data[5]  = 0x36;
    msg1.Data[6]  = 0x37;
    msg1.Data[7]  = 0x38;
    CAN_Transmit(tCAN, 0, &msg1);//Send CAN message
    printf("Send EXT_ID:0x3377,Data[31,32,33,34,35,36,37,38]\n");
    CLK_SysTickDelay(delaycount);   /* Generate the Delay Time by Systick */

    /* Send Message No.5 */
    msg1.FrameType = CAN_DATA_FRAME;
    msg1.IdType   = CAN_EXT_ID;
    msg1.Id       = 0x7755;
    msg1.DLC      = 8;
    msg1.Data[0]  = 0x41;
    msg1.Data[1]  = 0x42;
    msg1.Data[2]  = 0x43;
    msg1.Data[3]  = 0x44;
    msg1.Data[4]  = 0x45;
    msg1.Data[5]  = 0x46;
    msg1.Data[6]  = 0x47;
    msg1.Data[7]  = 0x48;

    CAN_Transmit(tCAN, 0, &msg1);//Send CAN message
    printf("Send EXT_ID:0x7755,Data[41,42,43,44,45,46,47,48]\n");
    CLK_SysTickDelay(delaycount);   /* Generate the Delay Time by Systick */

}

/*---------------------------------------------------------------------------------------------------------*/
/* Function: AdcSingleCycleScanModeTest                                                                    */
/*                                                                                                         */
/* Parameters:                                                                                             */
/*   None.                                                                                                 */
/*                                                                                                         */
/* Returns:                                                                                                */
/*   None.                                                                                                 */
/*                                                                                                         */
/* Description:                                                                                            */
/*   ADC single cycle scan mode test.                                                                      */
/*---------------------------------------------------------------------------------------------------------*/
void AdcSingleCycleScanModeTest()
{
    uint8_t  u8Option;
    uint32_t u32ChannelCount;
    int32_t  i32ConversionData;

    printf("\n");
    printf("+----------------------------------------------------------------------+\n");
    printf("|                 ADC single cycle scan mode sample code               |\n");
    printf("+----------------------------------------------------------------------+\n");

    while(1) {
        printf("\n\nSelect input mode:\n");
        printf("  [1] Single end input (channel 0, 1, 2 and 3)\n");
        printf("  [2] Differential input (input channel pair 0 and 1)\n");
        printf("  Other keys: exit single cycle scan mode test\n");
        u8Option = getchar();
        if(u8Option == '1') {

            /* Set the ADC operation mode as single-cycle, input mode as single-end and
                 enable the analog input channel 0, 1, 2 and 3 */
            ADC_Open(ADC, ADC_ADCR_DIFFEN_SINGLE_END, ADC_ADCR_ADMD_SINGLE_CYCLE, 0x1F);

            /* Power on ADC module */
            ADC_POWER_ON(ADC);

            /* Clear the A/D interrupt flag for safe */
            ADC_CLR_INT_FLAG(ADC, ADC_ADF_INT);

            /* Start A/D conversion */
            ADC_START_CONV(ADC);

            /* Wait conversion done */
            while(!ADC_GET_INT_FLAG(ADC, ADC_ADF_INT));

            for(u32ChannelCount = 0; u32ChannelCount < 5; u32ChannelCount++) {
                i32ConversionData = ADC_GET_CONVERSION_DATA(ADC, u32ChannelCount);
                printf("Conversion result of channel %d: 0x%X (%d)\n", u32ChannelCount, i32ConversionData, i32ConversionData);
            }
        } else if(u8Option == '2') {

            /* Set the ADC operation mode as single-cycle, input mode as differential and
               enable analog input channel 0 and 2 */
            ADC_Open(ADC, ADC_ADCR_DIFFEN_DIFFERENTIAL, ADC_ADCR_ADMD_SINGLE_CYCLE, 0x5);

            /* Power on ADC module */
            ADC_POWER_ON(ADC);

            /* Clear the A/D interrupt flag for safe */
            ADC_CLR_INT_FLAG(ADC, ADC_ADF_INT);

            /* Start A/D conversion */
            ADC_START_CONV(ADC);

            /* Wait conversion done */
            while(!ADC_GET_INT_FLAG(ADC, ADC_ADF_INT));

            for(u32ChannelCount = 0; u32ChannelCount < 2; u32ChannelCount++) {
                i32ConversionData = ADC_GET_CONVERSION_DATA(ADC, u32ChannelCount * 2);
                printf("Conversion result of differential input pair %d: 0x%X (%d)\n", u32ChannelCount, i32ConversionData, i32ConversionData);
            }
        } else
            return ;

    }
}

void AdcSingleCycleScan(void)
{
    uint32_t u32ChannelCount;
    int32_t  i32ConversionData;

    printf("\n");
    printf("+----------------------------------------------------------------------+\n");
    printf("|                 ADC single cycle scan mode sample code               |\n");
    printf("+----------------------------------------------------------------------+\n");
    /* Set the ADC operation mode as single-cycle, input mode as single-end and
            enable the analog input channel 0, 1, 2 and 3 */
    ADC_Open(ADC, ADC_ADCR_DIFFEN_SINGLE_END, ADC_ADCR_ADMD_SINGLE_CYCLE, 0x1F);

    /* Power on ADC module */
    ADC_POWER_ON(ADC);

    /* Clear the A/D interrupt flag for safe */
    ADC_CLR_INT_FLAG(ADC, ADC_ADF_INT);

    /* Start A/D conversion */
    ADC_START_CONV(ADC);

    /* Wait conversion done */
    while(!ADC_GET_INT_FLAG(ADC, ADC_ADF_INT));

    for(u32ChannelCount = 0; u32ChannelCount < 5; u32ChannelCount++) {
        i32ConversionData = ADC_GET_CONVERSION_DATA(ADC, u32ChannelCount);
        switch(u32ChannelCount)
        {
            case 0:
                i32BatteryValue[u32ChannelCount] = i32ConversionData * 31 * 5000 / 4096;
            break;
            default:
                i32BatteryValue[u32ChannelCount] = i32ConversionData * 347 / 47 * 5000 / 4096;
            break;
        }
        printf("Conversion result of channel %d: 0x%X (%d) volt: %d mV\n", u32ChannelCount, i32ConversionData, i32ConversionData,i32BatteryValue[u32ChannelCount]);
    }
}
/*---------------------------------------------------------------------------------------------------------*/
/* MAIN function                                                                                           */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{
    /* Declare a CAN message structure */
    STR_CANMSG_T msg;
    CAN_T *tCAN;
    tCAN = (CAN_T *) CAN1;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART0 for printf */
    UART0_Init();

	GPIO_SetMode(PA, BIT9, GPIO_PMD_OUTPUT);
	PA9 =1;

	GPIO_SetMode(PA, BIT12, GPIO_PMD_OUTPUT);
	PA12 =1;

    /* To program PWRCON register, it needs to disable register protection first. */
    SYS_UnlockReg();

    /* Open Timer0 frequency to 1 Hz in toggle-output mode */
    TIMER_Open(TIMER0, TIMER_TOGGLE_MODE, 1);

    /* Enable Timer0 interrupt and wake-up function */
    TIMER_EnableInt(TIMER0);
    TIMER_EnableWakeup(TIMER0);

    /* Enable Timer0 NVIC */
    NVIC_EnableIRQ(TMR0_IRQn);

    /* Start Timer0 counting */
    TIMER_Start(TIMER0);

    /* Enable CAN interrupt */
    CAN_EnableInt(tCAN, CAN_CON_IE_Msk | CAN_CON_SIE_Msk);
    /* Set Interrupt Priority */
    NVIC_SetPriority(CAN1_IRQn, (1 << __NVIC_PRIO_BITS) - 2);
    /* Enable External Interrupt */
    NVIC_EnableIRQ(CAN1_IRQn);

    /* configuring the Bit Timing */
    SelectCANSpeed(tCAN);

    while(1)
    {
        PA12 = ~PA12;
        AdcSingleCycleScan();
        msg.FrameType = CAN_DATA_FRAME;
        msg.IdType   = CAN_STD_ID;
        msg.Id       = 0x180;
        msg.DLC      = 8;
        msg.Data[0]  = (uint8_t)(i32BatteryValue[3] - i32BatteryValue[4]);
        msg.Data[1]  = (uint8_t)((i32BatteryValue[3] - i32BatteryValue[4]) >> 8);
        msg.Data[2]  = (uint8_t)(i32BatteryValue[2] - i32BatteryValue[3]);
        msg.Data[3]  = (uint8_t)((i32BatteryValue[2] - i32BatteryValue[3]) >> 8);
        msg.Data[4]  = (uint8_t)(i32BatteryValue[1] - i32BatteryValue[2]);
        msg.Data[5]  = (uint8_t)((i32BatteryValue[1] - i32BatteryValue[2]) >> 8);
        msg.Data[6]  = (uint8_t)(i32BatteryValue[0] - i32BatteryValue[1]);
        msg.Data[7]  = (uint8_t)((i32BatteryValue[0] - i32BatteryValue[1]) >> 8);
        CAN_Transmit(tCAN, 0, &msg);//Send CAN message
        CLK_SysTickDelay(1000);   /* Generate the Delay Time by Systick */

        PowerDownFunction();
    }

}



