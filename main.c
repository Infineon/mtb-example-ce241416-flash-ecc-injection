/******************************************************************************
* File Name: main.c
*
* Description: This is the source code for the PSoC 4 MCU Flash memory ECC
*               injection Example for ModusToolbox.
*
* Related Document: See README.md 
*
*******************************************************************************
* Copyright 2025, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
*******************************************************************************/

/*******************************************************************************
 * Include header files
 ******************************************************************************/
#include "cy_pdl.h"
#include "cybsp.h"
#include "cy_retarget_io.h"
#include <stdio.h>
#include <inttypes.h>

/*******************************************************************************
* Macros
*******************************************************************************/
#define LED_DELAY_MS              (500u)
#define CY_ASSERT_FAILED          (0u)
#define SYSFAULT_INTR_PRIORITY    (2u)

/* Test value */
#define CORRECT_DATA                   (0x0123456789abcdefull)

/* The 128-bit code word used for calculating the parity has this static 64-bit component */
#define CODEWORD_UPPER_64_BIT          (0x0000000000000108ull)

/* Number of bytes in a 64-bit word */
#define BYTES_PER_64_BIT_WORD          8

/* Interval value */
#define INTERVAL_MS                    (1)

/* Available syndrome bits for 8-bit parity calculation */
#define ECC_8BIT_RAM_SYNDROM_BITS        (8)
#define ECC_8BIT_CODE_WORD_SIZE          (128)
#define ECC_8BIT_SINGLE_ERROR_BIT       (0x80)
#define ECC_8BIT_DOUBLE_ERROR_MASK      (0x7F)
#define ECC_8BIT_PARITY_BITS_MASK       (0xFF)

/* Defines for 8-bit ECC parity calculation */
#define ECC_8BIT_DATA_P0_SW              (0x44844a88952aad5b)
#define ECC_8BIT_DATA_P1_SW              (0x1108931126b3366d)
#define ECC_8BIT_DATA_P2_SW              (0x06111c2238c3c78e)
#define ECC_8BIT_DATA_P3_SW              (0x9821e043c0fc07f0)
#define ECC_8BIT_DATA_P4_SW              (0xe03e007c00fff800)
#define ECC_8BIT_DATA_P5_SW              (0xffc0007fff000000)
#define ECC_8BIT_DATA_P6_SW              (0xffffff8000000000)
#define ECC_8BIT_DATA_P7_SW              (0xd44225844ba65cb7)

/* Defines the Code/Data ECC Injection position for FLASH */
#define ECC_FLASH_INJECT_POS             (0x03ul)
#define ECC_FLASH_CW_ADDR_MASK           (0x00FFFFFFul)
#define ECC_FLASH_GET_CW_ADDR(_addr)     ((_addr >> ECC_FLASH_INJECT_POS) & ECC_FLASH_CW_ADDR_MASK)

/*******************************************************************************
* Global Variables
*******************************************************************************/
/* Create a global volatile variable where the test access will store its data to prevent compiler optimization */
volatile uint64_t g_flashReadData;

/* Flag that is set in the fault IRQ handler and checked/cleared at other places */
bool g_faultIrqOccurred = false;


/* Variable that will end up as constant data in flash and will be used for the error injection test */
const uint64_t FLASH_TEST_VAR = CORRECT_DATA;

/*******************************************************************************
* Function Name: handleFaultIrq
********************************************************************************
* Summary:
* This is the fault struct interrupt handles 
* It will occur on a code flash correctable ECC fault.
*
* Parameters:
*  none
*
* Return:
*  none
*
*******************************************************************************/
static void FaultIrq_Handler(void)
{
    printf("Fault IRQ Handler entered!\r\n");

    cy_en_sysfault_source_t errorSource = Cy_SysFault_GetErrorSource(FAULT_STRUCT0);
    uint32_t faultAddress = Cy_SysFault_GetFaultData(FAULT_STRUCT0, CY_SYSFAULT_DATA0);
    uint32_t  faultInfo = Cy_SysFault_GetFaultData(FAULT_STRUCT0, CY_SYSFAULT_DATA1);

    /* Check and display the fault information */
    if (errorSource == CPUSS_FAULT_FLASHC_C_ECC)
    {
        printf("1-bit error is detected!\r\n");
        printf("- Address:       0x%08" PRIx32 "\r\n", faultAddress);
        printf("- ECC syndromes: 0x%02" PRIx32 "\r\n", faultInfo);

        if (g_flashReadData == CORRECT_DATA)
        {
            printf("TEST Step2 OK!\r\n\r\n");
        }
        else
        {
            printf("TEST Step2 ERROR: Incorrect data read!\r\n\r\n");
        }
    }
    else
    {
        printf("TEST Step2 ERROR: Unexpected fault detected!\r\n\r\n");
    }

    /* Set flag so that test code can check that the IRQ has occurred */
    g_faultIrqOccurred = true;

    Cy_SysFault_ClearStatus(FAULT_STRUCT0);
    Cy_SysFault_ClearInterrupt(FAULT_STRUCT0);
}

/*******************************************************************************
* Function Name: Cy_SysLib_ProcessingFault
********************************************************************************
* Summary:
* This is the Processing Fault 
* It will be called by the PDL fault handler.
*
* Parameters:
*  none
*
* Return:
*  none
*
*******************************************************************************/
void Cy_SysLib_ProcessingFault(void)
{
    printf("Hard Fault Handler entered!\r\n");

    uint32_t errorData = Cy_SysFault_GetPendingFault(FAULT_STRUCT0, CY_SYSFAULT_SET0);

    if ((errorData & 0x20) != 0ul)
    {
        printf("2-bit error is detected!\r\n");
        printf("TEST Step3 OK!\r\n\r\n");
    }
    else
    {
        printf("TEST Step3 ERROR: Unknown Hard Fault!\r\n");
    }

    printf("Reset is required to return\r\n");

    /* Do not return from the fault without reset */
    for (;;)
    {
        /* Toggle the user LED state */
        Cy_GPIO_Inv(CYBSP_USER_LED1_PORT, CYBSP_USER_LED1_PIN);

        /* Wait for 0.5 seconds */
        Cy_SysLib_Delay(LED_DELAY_MS);
    }        
}

/*******************************************************************************
* Function Name: initFaultHandling
********************************************************************************
* Summary:
* This Initialize fault handling to generate an IRQ on a Code Flash correctable 
* ECC fault 
*
* Parameters:
*  none
*
* Return:
*  none
*
*******************************************************************************/
static void initFaultHandling(void)
{
    /* SysFault config, Only IRQ is needed as fault reaction in this example */
    cy_stc_sysfault_config_t sysfaultConfig = 
    {
        .resetEnable   = false,
        .outputEnable  = false,
        .triggerEnable = false,
    };

    /* Set up fault struct and enable interrupt for Code Flash correctable ECC fault */
    Cy_SysFault_ClearStatus(FAULT_STRUCT0);
    Cy_SysFault_SetMaskByIdx(FAULT_STRUCT0, CPUSS_FAULT_FLASHC_C_ECC);
    Cy_SysFault_SetInterruptMask(FAULT_STRUCT0);
    cy_en_sysfault_status_t status = Cy_SysFault_Init(FAULT_STRUCT0, &sysfaultConfig);

    if (status != CY_SYSFAULT_SUCCESS)
    {
        CY_ASSERT(0);
    };

    /* Configure Interrupt for SysFault structure */
    cy_stc_sysint_t sysfaultIntrConfig =
    {
        .intrSrc = cpuss_interrupt_fault_0_IRQn,
        .intrPriority = SYSFAULT_INTR_PRIORITY
    };

    cy_rslt_t status_sysfault = Cy_SysInt_Init(&sysfaultIntrConfig, &FaultIrq_Handler);
    if (status_sysfault != CY_SYSINT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    NVIC_ClearPendingIRQ(sysfaultIntrConfig.intrSrc);
    NVIC_EnableIRQ(sysfaultIntrConfig.intrSrc);
}

/*******************************************************************************
* Function Name: reductionXOR64
********************************************************************************
* Summary:
* It do reduction XOR operation on 64-bit value 
* ECC fault 
*
* Parameters:
*  data - value
*
* Return:
*  uint8_t - XOR reduction result
*
*******************************************************************************/
uint8_t reductionXOR64(uint64_t data)
{
    uint8_t parity = 0;

    while (data)
    {
        parity = !parity;
        data = data & (data - 1);
    }

    return parity;
}

/*******************************************************************************
* Function Name: getParityForValue
********************************************************************************
* Summary:
* This calculate the Code Flash ECC parity for the provided 64-bit value
*
* Parameters:
*  value64 - value for which ECC parity shall calculated
*
* Return:
*  uint8_t - ECC parity
*
*******************************************************************************/
static uint8_t getParityForValue(uint64_t data)
{
    /* Constants for ECC parity calculation as per the Architecture TRM */
    uint8_t parity    = 0;
    uint64_t codeWord = 0;
    uint64_t eccP[ECC_8BIT_RAM_SYNDROM_BITS] =  {   ECC_8BIT_DATA_P0_SW,
                                                    ECC_8BIT_DATA_P1_SW,
                                                    ECC_8BIT_DATA_P2_SW,
                                                    ECC_8BIT_DATA_P3_SW,
                                                    ECC_8BIT_DATA_P4_SW,
                                                    ECC_8BIT_DATA_P5_SW,
                                                    ECC_8BIT_DATA_P6_SW,
                                                    ECC_8BIT_DATA_P7_SW,
                                                };

    /* Flash ECC parity calculation does not contains address */
    codeWord = ECC_FLASH_GET_CW_ADDR(0);
    codeWord |= data;

    /* Calculate each ECC parity bit individually according to the Architecture TRM */
    for (uint32_t i = 0; i < ECC_8BIT_RAM_SYNDROM_BITS; i++)
    {
        parity |= (reductionXOR64(codeWord & eccP[i]) << i);
    }

    return parity;
}

/*******************************************************************************
* Function Name: executeTestAccess
********************************************************************************
* Summary:
* This makes a test access to our target variable in flash with all required 
* steps before and after the access
*
* Parameters:
*  none
*
* Return:
*  none
*
*******************************************************************************/
static void executeTestAccess(void)
{
    /* Initialize the read data variable which might contain data from the previous access */
    g_flashReadData = 0;
    /* Clear flag that is used to detect the IRQ occurrence */
    g_faultIrqOccurred = false;
    /* Make the test access. The address of the test variable is used to prevent compiler optimization */
    g_flashReadData = *((volatile const uint64_t*)&FLASH_TEST_VAR);
}

/*******************************************************************************
* Function Name: main
********************************************************************************
* Summary:
*  System entrance point. This function performs
*  - initial setup of device
*  - configure the SCB block as UART interface
*  - prints out "Hello World" via UART interface
*  - Blinks an LED under firmware control at 1 Hz
*
* Parameters:
*  none
*
* Return:
*  int
*
*******************************************************************************/
int main(void)
{
    cy_rslt_t result;
    cy_stc_scb_uart_context_t CYBSP_UART_context;
    uint8_t correctParity;

    /* Initialize the device and board peripherals */
    result = cybsp_init();

    /* Board init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(CY_ASSERT_FAILED);
    }

    /* Configure and enable the UART peripheral */
    Cy_SCB_UART_Init(CYBSP_UART_HW, &CYBSP_UART_config, &CYBSP_UART_context);
    Cy_SCB_UART_Enable(CYBSP_UART_HW);
    cy_retarget_io_init(CYBSP_UART_HW);

    /* Enable global interrupts */
    __enable_irq();

    /* Send a string over serial terminal */
    printf("\x1b[2J\x1b[;H");
    printf("****************** Flash ECC Error Injection ******************\r\n\r\n");

    initFaultHandling();

    correctParity = getParityForValue(FLASH_TEST_VAR);

    printf("- Address:            0x%08" PRIx32 "\r\n", (uint32_t)&FLASH_TEST_VAR);
    printf("- Correct ECC Parity: 0x%02x \r\n", correctParity);

    printf("Test Step1: Inject correct parity to prove correctness of ECC parity calculation\r\n");

    Cy_Flash_SetupEccInjection((uint32_t)&FLASH_TEST_VAR, correctParity);
    Cy_Flash_EnableEccInjection();
    executeTestAccess();

    if (g_faultIrqOccurred != false)
    {
        printf("TEST Step1 ERROR: Unexpected fault occurred!\r\n\r\n");
    }
    else if (g_flashReadData != CORRECT_DATA)
    {
        printf("TEST Step1 ERROR: Incorrect data read!\r\n\r\n");
    }
    else
    {
        printf("Fault interrupt is not detected!\r\n");
        printf("TEST Step1 OK!\r\n\r\n");
    }

    printf("Test Step2: Inject parity with 1-bit error to test correctable ECC fault\r\n");

    /* 1-bit flip Parity data */
    Cy_Flash_SetupEccInjection((uint32_t)&FLASH_TEST_VAR, correctParity ^ 0x01ul);
    executeTestAccess();

    printf("Test Step3: Inject parity with 2-bit error to test non-correctable ECC fault\r\n");

    /* 2-bit flip Parity data */
    Cy_Flash_SetupEccInjection((uint32_t)&FLASH_TEST_VAR, correctParity ^ 0x03ul);
    executeTestAccess();

    for(;;)
    {
    }
}

/* [] END OF FILE */