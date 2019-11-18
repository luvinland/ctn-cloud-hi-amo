/*******************************************************************************
*
*               Audio Framework
*               ---------------
*
********************************************************************************
*     PinDef.h
********************************************************************************
*
*     Description:  GPIO Pin Definitions for AWE
*
*******************************************************************************/
#include "I94100.h"
#include "gpio.h"

#define GPIO_DIR_INPUT 0
#define GPIO_DIR_OUTPUT 1

#define MAX_PINS 15


#define GPIO_PIN_0                 ((uint16_t)0x0001U)  /* Pin 0 selected    */
#define GPIO_PIN_1                 ((uint16_t)0x0002U)  /* Pin 1 selected    */
#define GPIO_PIN_2                 ((uint16_t)0x0004U)  /* Pin 2 selected    */
#define GPIO_PIN_3                 ((uint16_t)0x0008U)  /* Pin 3 selected    */
#define GPIO_PIN_4                 ((uint16_t)0x0010U)  /* Pin 4 selected    */
#define GPIO_PIN_5                 ((uint16_t)0x0020U)  /* Pin 5 selected    */
#define GPIO_PIN_6                 ((uint16_t)0x0040U)  /* Pin 6 selected    */
#define GPIO_PIN_7                 ((uint16_t)0x0080U)  /* Pin 7 selected    */
#define GPIO_PIN_8                 ((uint16_t)0x0100U)  /* Pin 8 selected    */
#define GPIO_PIN_9                 ((uint16_t)0x0200U)  /* Pin 9 selected    */
#define GPIO_PIN_10                ((uint16_t)0x0400U)  /* Pin 10 selected   */
#define GPIO_PIN_11                ((uint16_t)0x0800U)  /* Pin 11 selected   */
#define GPIO_PIN_12                ((uint16_t)0x1000U)  /* Pin 12 selected   */
#define GPIO_PIN_13                ((uint16_t)0x2000U)  /* Pin 13 selected   */
#define GPIO_PIN_14                ((uint16_t)0x4000U)  /* Pin 14 selected   */
#define GPIO_PIN_15                ((uint16_t)0x8000U)  /* Pin 15 selected   */
#define GPIO_PIN_All               ((uint16_t)0xFFFFU)  /* All pins selected */


#define GPIO_A    0
#define GPIO_B    1
#define GPIO_C    2
#define GPIO_D    3

typedef struct
{
    GPIO_T *port; 
    UINT32 pin;
    char szPinDesc[5];
}   PinInfoType;

PinInfoType InputPinMap[MAX_PINS] =
{
#if 0  
    {   PA, 
        GPIO_PIN_3,
        "Nused",
    },
#endif
    {  PA, 
        GPIO_PIN_9,
        "Nused",
    },
    
    {   PA, 
        GPIO_PIN_6,
        "Nused",
    },
    {   PB, 
        GPIO_PIN_2,
        "Nused",
    },
    {   PB, 
        GPIO_PIN_3,
        "Nused",
    },
    {   PB, 
        GPIO_PIN_4,
        "Nused",
    },
    {   PB, 
        GPIO_PIN_5,
        "Nused",
    },
    {   PB, 
        GPIO_PIN_6,
        "Nused",
    },
    {   PB, 
        GPIO_PIN_7,
        "Nused",
    },
    {   PC, 
        GPIO_PIN_5,
        "LED",
    },
    {   PC, 
        GPIO_PIN_6,
        "LED",
    },
    {   PC, 
        GPIO_PIN_14,
        "LED",
    },
    {   PC, 
        GPIO_PIN_15,
        "LED",
    },
    {   PD, 
        GPIO_PIN_7,
        "Nused",
    },
    {   PD, 
        GPIO_PIN_10,
        "Nused",
    },
    {   PD, 
        GPIO_PIN_13,
        "Nused",
    },
};

PinInfoType OutputPinMap[MAX_PINS] =
{ 
#if 0  
    {   PA, 
        GPIO_PIN_3,
        "Nused",
    },
#endif
    {  PA, 
        GPIO_PIN_9,
        "Nused",
    },
    
    {   PA, 
        GPIO_PIN_6,
        "Nused",
    },
    {   PB, 
        GPIO_PIN_2,
        "Nused",
    },
    {   PB, 
        GPIO_PIN_3,
        "Nused",
    },
    {   PB, 
        GPIO_PIN_4,
        "Nused",
    },
    {   PB, 
        GPIO_PIN_5,
        "Nused",
    },
    {   PB, 
        GPIO_PIN_6,
        "Nused",
    },
    {   PB, 
        GPIO_PIN_7,
        "Nused",
    },
    {   PC, 
        GPIO_PIN_5,
        "LED",
    },
    {   PC, 
        GPIO_PIN_6,
        "LED",
    },
    {   PC, 
        GPIO_PIN_14,
        "LED",
    },
    {   PC, 
        GPIO_PIN_15,
        "LED",
    },
    {   PD, 
        GPIO_PIN_7,
        "Nused",
    },
    {   PD, 
        GPIO_PIN_10,
        "Nused",
    },
    {   PD, 
        GPIO_PIN_13,
        "Nused",
    },
};   
