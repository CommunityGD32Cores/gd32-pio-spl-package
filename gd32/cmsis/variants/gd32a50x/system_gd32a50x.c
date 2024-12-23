/*!
    \file  system_gd32a50x.c
    \brief CMSIS Cortex-M33 Device Peripheral Access Layer Source File for
           GD32A50X Device Series
*/

/*
 * Copyright (c) 2009-2018 Arm Limited. All rights reserved.
 * Copyright (c) 2024 GigaDevice Semiconductor Inc.

 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the License); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/* This file refers the CMSIS standard, some adjustments are made according to GigaDevice chips */

#include "gd32a50x.h"

#define __IRC8M           (IRC8M_VALUE)            /* internal 8 MHz RC oscillator frequency */
#define __HXTAL           (HXTAL_VALUE)            /* high speed crystal oscillator frequency */

#define VECT_TAB_OFFSET  (uint32_t)0x00            /* vector table base offset */

/* select a system clock by uncommenting the following line */
/* use IRC8M */
//#define __SYSTEM_CLOCK_IRC8M                            (uint32_t)(__IRC8M)
//#define __SYSTEM_CLOCK_24M_PLL_IRC8M                    (uint32_t)(24000000)
//#define __SYSTEM_CLOCK_48M_PLL_IRC8M                    (uint32_t)(48000000)
//#define __SYSTEM_CLOCK_72M_PLL_IRC8M                    (uint32_t)(72000000)
//#define __SYSTEM_CLOCK_100M_PLL_IRC8M                   (uint32_t)(100000000)

/* use HXTAL(CK_HXTAL = 8M)*/
//#define __SYSTEM_CLOCK_HXTAL                            (uint32_t)(__HXTAL)
//#define __SYSTEM_CLOCK_24M_PLL_HXTAL                    (uint32_t)(24000000)
//#define __SYSTEM_CLOCK_48M_PLL_HXTAL                    (uint32_t)(48000000)
//#define __SYSTEM_CLOCK_72M_PLL_HXTAL                    (uint32_t)(72000000)
#define __SYSTEM_CLOCK_100M_PLL_HXTAL                   (uint32_t)(100000000)

#define RCU_MODIFY_DE_3(__delay)  do{                                     \
                                      volatile uint32_t i,reg;            \
                                      if(0 != __delay){                   \
                                          for(i=0; i<__delay; i++){       \
                                          }                               \
                                          reg = RCU_CFG0;                 \
                                          reg &= ~(RCU_CFG0_AHBPSC);      \
                                          reg |= RCU_AHB_CKSYS_DIV2;      \
                                          RCU_CFG0 = reg;                 \
                                          for(i=0; i<__delay; i++){       \
                                          }                               \
                                          reg = RCU_CFG0;                 \
                                          reg &= ~(RCU_CFG0_AHBPSC);      \
                                          reg |= RCU_AHB_CKSYS_DIV4;      \
                                          RCU_CFG0 = reg;                 \
                                          for(i=0; i<__delay; i++){       \
                                          }                               \
                                          reg = RCU_CFG0;                 \
                                          reg &= ~(RCU_CFG0_AHBPSC);      \
                                          reg |= RCU_AHB_CKSYS_DIV8;      \
                                          RCU_CFG0 = reg;                 \
                                          for(i=0; i<__delay; i++){       \
                                          }                               \
                                      }                                   \
                                  }while(0)

#define RCU_MODIFY_UP_3(__delay)  do{                                     \
                                      volatile uint32_t i,reg;            \
                                      if(0 != __delay){                   \
                                          for(i=0; i<__delay; i++){       \
                                          }                               \
                                          reg = RCU_CFG0;                 \
                                          reg &= ~(RCU_CFG0_AHBPSC);      \
                                          reg |= RCU_AHB_CKSYS_DIV4;      \
                                          RCU_CFG0 = reg;                 \
                                          for(i=0; i<__delay; i++){       \
                                          }                               \
                                          reg = RCU_CFG0;                 \
                                          reg &= ~(RCU_CFG0_AHBPSC);      \
                                          reg |= RCU_AHB_CKSYS_DIV2;      \
                                          RCU_CFG0 = reg;                 \
                                          for(i=0; i<__delay; i++){       \
                                          }                               \
                                          reg = RCU_CFG0;                 \
                                          reg &= ~(RCU_CFG0_AHBPSC);      \
                                          reg |= RCU_AHB_CKSYS_DIV1;      \
                                          RCU_CFG0 = reg;                 \
                                      }                                   \
                                  }while(0)

/* set the system clock frequency and declare the system clock configuration function */
#ifdef __SYSTEM_CLOCK_IRC8M
uint32_t SystemCoreClock = __SYSTEM_CLOCK_IRC8M;
static void system_clock_8m_irc8m(void);
#elif defined (__SYSTEM_CLOCK_24M_PLL_IRC8M)
uint32_t SystemCoreClock = __SYSTEM_CLOCK_24M_PLL_IRC8M;
static void system_clock_24m_pll_irc8m(void);
#elif defined (__SYSTEM_CLOCK_48M_PLL_IRC8M)
uint32_t SystemCoreClock = __SYSTEM_CLOCK_48M_PLL_IRC8M;
static void system_clock_48m_pll_irc8m(void);
#elif defined (__SYSTEM_CLOCK_72M_PLL_IRC8M)
uint32_t SystemCoreClock = __SYSTEM_CLOCK_72M_PLL_IRC8M;
static void system_clock_72m_pll_irc8m(void);
#elif defined (__SYSTEM_CLOCK_100M_PLL_IRC8M)
uint32_t SystemCoreClock = __SYSTEM_CLOCK_100M_PLL_IRC8M;
static void system_clock_100m_pll_irc8m(void);

#elif defined (__SYSTEM_CLOCK_HXTAL)
uint32_t SystemCoreClock = __SYSTEM_CLOCK_HXTAL;
static void system_clock_hxtal(void);
#elif defined (__SYSTEM_CLOCK_24M_PLL_HXTAL)
uint32_t SystemCoreClock = __SYSTEM_CLOCK_24M_PLL_HXTAL;
static void system_clock_24m_pll_hxtal(void);
#elif defined (__SYSTEM_CLOCK_48M_PLL_HXTAL)
uint32_t SystemCoreClock = __SYSTEM_CLOCK_48M_PLL_HXTAL;
static void system_clock_48m_pll_hxtal(void);
#elif defined (__SYSTEM_CLOCK_72M_PLL_HXTAL)
uint32_t SystemCoreClock = __SYSTEM_CLOCK_72M_PLL_HXTAL;
static void system_clock_72m_pll_hxtal(void);
#elif defined (__SYSTEM_CLOCK_100M_PLL_HXTAL)
uint32_t SystemCoreClock = __SYSTEM_CLOCK_100M_PLL_HXTAL;
static void system_clock_100m_pll_hxtal(void);
#endif /* __SYSTEM_CLOCK_IRC8M */

/* configure the system clock */
static void system_clock_config(void);

/*!
    \brief      setup the micro-controller system, initialize the system
    \param[in]  none
    \param[out] none
    \retval     none
*/
void SystemInit(void)
{
    /* FPU settings */
#if (__FPU_PRESENT == 1U) && (__FPU_USED == 1U)
    SCB->CPACR |= ((3UL << 10 * 2) | (3UL << 11 * 2)); /* set CP10 and CP11 Full Access */
#endif
    /* reset the RCU clock configuration to the default reset state */
    /* set IRC8MEN bit */
    RCU_CTL |= RCU_CTL_IRC8MEN;

    if(((RCU_CFG0 & RCU_CFG0_SCSS) == RCU_SCSS_PLL)){
        RCU_MODIFY_DE_3(0x100);
    }
    RCU_CFG0 &= ~RCU_CFG0_SCS;
    /* reset HXTALSCAL, LCKMEN, PLLMEN, PLLEN, CKMEN and HXTALEN bits */
    RCU_CTL &= ~(RCU_CTL_PLLEN | RCU_CTL_CKMEN | RCU_CTL_HXTALEN | RCU_CTL_HXTALSCAL | RCU_CTL_LCKMEN | RCU_CTL_PLLMEN);
    /* disable all interrupts */
    RCU_INT = 0x00FF0000U;

    /* Reset CFG0 and CFG1 registers */
    RCU_CFG0 = 0x00020000U;
    RCU_CFG1 = 0x00000000U;

    /* reset HXTALBPS bit */
    RCU_CTL &= ~(RCU_CTL_HXTALBPS);

    /* configure HXTALSCAL according to HXTAL_VALUE */
    if(HXTAL_VALUE > 8000000U) {
        RCU_CTL |= RCU_CTL_HXTALSCAL;
    }

    /* configure the system clock source, PLL Multiplier, AHB/APBx prescalers and Flash settings */
    system_clock_config();

    /* LXTALBPS configuration */
    if(0U == (RCU_APB1EN & RCU_APB1EN_PMUEN)) {
        /* check whether PMU clock is enabled */
        RCU_APB1EN |= RCU_APB1EN_PMUEN;
        if(0U == (PMU_CTL & PMU_CTL_BKPWEN)) {
            /* BKPWEN bit is not been set */
            PMU_CTL |= PMU_CTL_BKPWEN;
            /* LXTALBPS bit must be configured to 1 */
            RCU_BDCTL |= RCU_BDCTL_LXTALBPS;
            /* clear the BKPWEN bit */
            PMU_CTL &= ~PMU_CTL_BKPWEN;
        } else {
            /* LXTALBPS bit must be configured to 1 */
            RCU_BDCTL |= RCU_BDCTL_LXTALBPS;
        }

        /* disable the PMU clock */
        RCU_APB1EN &= ~RCU_APB1EN_PMUEN;
    } else {
        /* the PMU clock is already enabled */
        if(0U == (PMU_CTL & PMU_CTL_BKPWEN)) {
            /* BKPWEN bit is not been set */
            PMU_CTL |= PMU_CTL_BKPWEN;
            /* LXTALBPS bit must be configured to 1 */
            RCU_BDCTL |= RCU_BDCTL_LXTALBPS;
            /* clear the BKPWEN bit */
            PMU_CTL &= ~PMU_CTL_BKPWEN;
        } else {
            /* LXTALBPS bit must be configured to 1 */
            RCU_BDCTL |= RCU_BDCTL_LXTALBPS;
        }
    }

#ifdef VECT_TAB_SRAM
    nvic_vector_table_set(NVIC_VECTTAB_RAM, VECT_TAB_OFFSET);
#else
    nvic_vector_table_set(NVIC_VECTTAB_FLASH, VECT_TAB_OFFSET);
#endif
}

/*!
    \brief      configure the system clock
    \param[in]  none
    \param[out] none
    \retval     none
*/
static void system_clock_config(void)
{
#ifdef __SYSTEM_CLOCK_IRC8M
    system_clock_8m_irc8m();
#elif defined (__SYSTEM_CLOCK_24M_PLL_IRC8M)
    system_clock_24m_pll_irc8m();
#elif defined (__SYSTEM_CLOCK_48M_PLL_IRC8M)
    system_clock_48m_pll_irc8m();
#elif defined (__SYSTEM_CLOCK_72M_PLL_IRC8M)
    system_clock_72m_pll_irc8m();
#elif defined (__SYSTEM_CLOCK_100M_PLL_IRC8M)
    system_clock_100m_pll_irc8m();

#elif defined (__SYSTEM_CLOCK_HXTAL)
    system_clock_hxtal();
#elif defined (__SYSTEM_CLOCK_24M_PLL_HXTAL)
    system_clock_24m_pll_hxtal();
#elif defined (__SYSTEM_CLOCK_48M_PLL_HXTAL)
    system_clock_48m_pll_hxtal();
#elif defined (__SYSTEM_CLOCK_72M_PLL_HXTAL)
    system_clock_72m_pll_hxtal();
#elif defined (__SYSTEM_CLOCK_100M_PLL_HXTAL)
    system_clock_100m_pll_hxtal();
#endif /* __SYSTEM_CLOCK_IRC8M */
}

#ifdef __SYSTEM_CLOCK_IRC8M
/*!
    \brief      configure the system clock to 8M by IRC8M
    \param[in]  none
    \param[out] none
    \retval     none
*/
static void system_clock_8m_irc8m(void)
{
    uint32_t timeout = 0U;
    uint32_t stab_flag = 0U;

    /* enable IRC8M */
    RCU_CTL |= RCU_CTL_IRC8MEN;

    /* wait until IRC8M is stable or the startup time is longer than IRC8M_STARTUP_TIMEOUT */
    do {
        timeout++;
        stab_flag = (RCU_CTL & RCU_CTL_IRC8MSTB);
    } while((0U == stab_flag) && (IRC8M_STARTUP_TIMEOUT != timeout));

    /* if fail */
    if(0U == (RCU_CTL & RCU_CTL_IRC8MSTB)) {
        while(1) {
        }
    }

    FMC_WS = (FMC_WS & (~FMC_WS_WSCNT)) | WS_WSCNT(0);

    RCU_APB1EN |= RCU_APB1EN_PMUEN;

    /* AHB = SYSCLK */
    RCU_CFG0 |= RCU_AHB_CKSYS_DIV1;
    /* APB2 = AHB/1 */
    RCU_CFG0 |= RCU_APB2_CKAHB_DIV1;
    /* APB1 = AHB/1 */
    RCU_CFG0 |= RCU_APB1_CKAHB_DIV1;

    /* select IRC8M as system clock */
    RCU_CFG0 &= ~RCU_CFG0_SCS;
    RCU_CFG0 |= RCU_CKSYSSRC_IRC8M;

    /* wait until IRC8M is selected as system clock */
    while((RCU_CFG0 & RCU_CFG0_SCSS) != RCU_SCSS_IRC8M) {
    }
}
/*!
    \brief      configure the system clock to 24M by PLL which selects IRC8M as its clock source
    \param[in]  none
    \param[out] none
    \retval     none
*/
#elif defined (__SYSTEM_CLOCK_24M_PLL_IRC8M)
static void system_clock_24m_pll_irc8m(void)
{
    uint32_t timeout = 0U;
    uint32_t stab_flag = 0U;

    /* enable IRC8M */
    RCU_CTL |= RCU_CTL_IRC8MEN;

    /* wait until IRC8M is stable or the startup time is longer than IRC8M_STARTUP_TIMEOUT */
    do {
        timeout++;
        stab_flag = (RCU_CTL & RCU_CTL_IRC8MSTB);
    } while((0U == stab_flag) && (IRC8M_STARTUP_TIMEOUT != timeout));

    /* if fail */
    if(0U == (RCU_CTL & RCU_CTL_IRC8MSTB)) {
        while(1) {
        }
    }

    FMC_WS = (FMC_WS & (~FMC_WS_WSCNT)) | WS_WSCNT(0);

    /* LDO output voltage high mode */
    RCU_APB1EN |= RCU_APB1EN_PMUEN;

    /* IRC8M is stable */
    /* AHB = SYSCLK/8 */
    RCU_CFG0 |= RCU_AHB_CKSYS_DIV8;
    /* APB2 = AHB/1 */
    RCU_CFG0 |= RCU_APB2_CKAHB_DIV1;
    /* APB1 = AHB/2 */
    RCU_CFG0 |= RCU_APB1_CKAHB_DIV2;

    /* CK_PLL = (CK_IRC8M/2) * 6 = 24 MHz */
    RCU_CFG0 &= ~(RCU_CFG0_PLLMF | RCU_CFG0_PLLMF_4);
    RCU_CFG0 |= RCU_PLL_MUL6;

    /* enable PLL */
    RCU_CTL |= RCU_CTL_PLLEN;

    /* wait until PLL is stable */
    while(0U == (RCU_CTL & RCU_CTL_PLLSTB)) {
    }

    /* select PLL as system clock */
    RCU_CFG0 &= ~RCU_CFG0_SCS;
    RCU_CFG0 |= RCU_CKSYSSRC_PLL;

    /* wait until PLL is selected as system clock */
    while((RCU_CFG0 & RCU_CFG0_SCSS) != RCU_SCSS_PLL) {
    }

    RCU_MODIFY_UP_3(0x50);
}


#elif defined (__SYSTEM_CLOCK_48M_PLL_IRC8M)
/*!
    \brief      configure the system clock to 48M by PLL which selects IRC8M as its clock source
    \param[in]  none
    \param[out] none
    \retval     none
*/
static void system_clock_48m_pll_irc8m(void)
{
    uint32_t timeout = 0U;
    uint32_t stab_flag = 0U;

    /* enable IRC8M */
    RCU_CTL |= RCU_CTL_IRC8MEN;

    /* wait until IRC8M is stable or the startup time is longer than IRC8M_STARTUP_TIMEOUT */
    do {
        timeout++;
        stab_flag = (RCU_CTL & RCU_CTL_IRC8MSTB);
    } while((0U == stab_flag) && (IRC8M_STARTUP_TIMEOUT != timeout));

    /* if fail */
    if(0U == (RCU_CTL & RCU_CTL_IRC8MSTB)) {
        while(1) {
        }
    }

    FMC_WS = (FMC_WS & (~FMC_WS_WSCNT)) | WS_WSCNT(1);

    /* LDO output voltage high mode */
    RCU_APB1EN |= RCU_APB1EN_PMUEN;

    /* IRC8M is stable */
    /* AHB = SYSCLK/8 */
    RCU_CFG0 |= RCU_AHB_CKSYS_DIV8;
    /* APB2 = AHB/1 */
    RCU_CFG0 |= RCU_APB2_CKAHB_DIV1;
    /* APB1 = AHB/2 */
    RCU_CFG0 |= RCU_APB1_CKAHB_DIV2;

    /* CK_PLL = (CK_IRC8M/2) * 12 = 48 MHz */
    RCU_CFG0 &= ~(RCU_CFG0_PLLMF | RCU_CFG0_PLLMF_4);
    RCU_CFG0 |= RCU_PLL_MUL12;

    /* enable PLL */
    RCU_CTL |= RCU_CTL_PLLEN;

    /* wait until PLL is stable */
    while(0U == (RCU_CTL & RCU_CTL_PLLSTB)) {
    }

    /* select PLL as system clock */
    RCU_CFG0 &= ~RCU_CFG0_SCS;
    RCU_CFG0 |= RCU_CKSYSSRC_PLL;

    /* wait until PLL is selected as system clock */
    while((RCU_CFG0 & RCU_CFG0_SCSS) != RCU_SCSS_PLL) {
    }

    RCU_MODIFY_UP_3(0x50);
}

#elif defined (__SYSTEM_CLOCK_72M_PLL_IRC8M)
/*!
    \brief      configure the system clock to 72M by PLL which selects IRC8M as its clock source
    \param[in]  none
    \param[out] none
    \retval     none
*/
static void system_clock_72m_pll_irc8m(void)
{
    uint32_t timeout = 0U;
    uint32_t stab_flag = 0U;

    /* enable IRC8M */
    RCU_CTL |= RCU_CTL_IRC8MEN;

    /* wait until IRC8M is stable or the startup time is longer than IRC8M_STARTUP_TIMEOUT */
    do {
        timeout++;
        stab_flag = (RCU_CTL & RCU_CTL_IRC8MSTB);
    } while((0U == stab_flag) && (IRC8M_STARTUP_TIMEOUT != timeout));

    /* if fail */
    if(0U == (RCU_CTL & RCU_CTL_IRC8MSTB)) {
        while(1) {
        }
    }

    FMC_WS = (FMC_WS & (~FMC_WS_WSCNT)) | WS_WSCNT(2);

    /* LDO output voltage high mode */
    RCU_APB1EN |= RCU_APB1EN_PMUEN;

    /* IRC8M is stable */
    /* AHB = SYSCLK/8 */
    RCU_CFG0 |= RCU_AHB_CKSYS_DIV8;
    /* APB2 = AHB/1 */
    RCU_CFG0 |= RCU_APB2_CKAHB_DIV1;
    /* APB1 = AHB/2 */
    RCU_CFG0 |= RCU_APB1_CKAHB_DIV2;

    /* CK_PLL = (CK_IRC8M/2) * 18 = 72 MHz */
    RCU_CFG0 &= ~(RCU_CFG0_PLLMF | RCU_CFG0_PLLMF_4);
    RCU_CFG0 |= RCU_PLL_MUL18;

    /* enable PLL */
    RCU_CTL |= RCU_CTL_PLLEN;

    /* wait until PLL is stable */
    while(0U == (RCU_CTL & RCU_CTL_PLLSTB)) {
    }

    /* select PLL as system clock */
    RCU_CFG0 &= ~RCU_CFG0_SCS;
    RCU_CFG0 |= RCU_CKSYSSRC_PLL;

    /* wait until PLL is selected as system clock */
    while((RCU_CFG0 & RCU_CFG0_SCSS) != RCU_SCSS_PLL) {
    }

    RCU_MODIFY_UP_3(0x100);
}

#elif defined (__SYSTEM_CLOCK_100M_PLL_IRC8M)
/*!
    \brief      configure the system clock to 100M by PLL which selects IRC8M as its clock source
    \param[in]  none
    \param[out] none
    \retval     none
*/
static void system_clock_100m_pll_irc8m(void)
{
    uint32_t timeout = 0U;
    uint32_t stab_flag = 0U;

    /* enable IRC8M */
    RCU_CTL |= RCU_CTL_IRC8MEN;

    /* wait until IRC8M is stable or the startup time is longer than IRC8M_STARTUP_TIMEOUT */
    do {
        timeout++;
        stab_flag = (RCU_CTL & RCU_CTL_IRC8MSTB);
    } while((0U == stab_flag) && (IRC8M_STARTUP_TIMEOUT != timeout));

    /* if fail */
    if(0U == (RCU_CTL & RCU_CTL_IRC8MSTB)) {
        while(1) {
        }
    }

    FMC_WS = (FMC_WS & (~FMC_WS_WSCNT)) | WS_WSCNT(3);

    /* LDO output voltage high mode */
    RCU_APB1EN |= RCU_APB1EN_PMUEN;

    /* IRC8M is stable */
    /* AHB = SYSCLK/8 */
    RCU_CFG0 |= RCU_AHB_CKSYS_DIV8;
    /* APB2 = AHB/1 */
    RCU_CFG0 |= RCU_APB2_CKAHB_DIV1;
    /* APB1 = AHB/2 */
    RCU_CFG0 |= RCU_APB1_CKAHB_DIV2;

    /* CK_PLL = (CK_IRC8M/2) * 25 = 100 MHz */
    RCU_CFG0 &= ~(RCU_CFG0_PLLMF | RCU_CFG0_PLLMF_4);
    RCU_CFG0 |= RCU_PLL_MUL25;

    /* enable PLL */
    RCU_CTL |= RCU_CTL_PLLEN;

    /* wait until PLL is stable */
    while(0U == (RCU_CTL & RCU_CTL_PLLSTB)) {
    }

    /* select PLL as system clock */
    RCU_CFG0 &= ~RCU_CFG0_SCS;
    RCU_CFG0 |= RCU_CKSYSSRC_PLL;

    /* wait until PLL is selected as system clock */
    while((RCU_CFG0 & RCU_CFG0_SCSS) != RCU_SCSS_PLL) {
    }

    RCU_MODIFY_UP_3(0x100);
}

#elif defined (__SYSTEM_CLOCK_HXTAL)
/*!
    \brief      configure the system clock to HXTAL
    \param[in]  none
    \param[out] none
    \retval     none
*/
static void system_clock_hxtal(void)
{
    uint32_t timeout = 0U;
    uint32_t stab_flag = 0U;

    /* enable HXTAL */
    RCU_CTL |= RCU_CTL_HXTALEN;

    /* wait until HXTAL is stable or the startup time is longer than HXTAL_STARTUP_TIMEOUT */
    do {
        timeout++;
        stab_flag = (RCU_CTL & RCU_CTL_HXTALSTB);
    } while((0U == stab_flag) && (HXTAL_STARTUP_TIMEOUT != timeout));

    /* if fail */
    if(0U == (RCU_CTL & RCU_CTL_HXTALSTB)) {
        while(1) {
        }
    }

    FMC_WS = (FMC_WS & (~FMC_WS_WSCNT)) | WS_WSCNT(0);

    RCU_APB1EN |= RCU_APB1EN_PMUEN;

    /* AHB = SYSCLK */
    RCU_CFG0 |= RCU_AHB_CKSYS_DIV1;
    /* APB2 = AHB/1 */
    RCU_CFG0 |= RCU_APB2_CKAHB_DIV1;
    /* APB1 = AHB/1 */
    RCU_CFG0 |= RCU_APB1_CKAHB_DIV1;

    /* select HXTAL as system clock */
    RCU_CFG0 &= ~RCU_CFG0_SCS;
    RCU_CFG0 |= RCU_CKSYSSRC_HXTAL;

    /* wait until HXTAL is selected as system clock */
    while((RCU_CFG0 & RCU_CFG0_SCSS) != RCU_SCSS_HXTAL) {
    }
}

#elif defined (__SYSTEM_CLOCK_24M_PLL_HXTAL)
/*!
    \brief      configure the system clock to 24M by PLL which selects HXTAL as its clock source
    \param[in]  none
    \param[out] none
    \retval     none
*/
static void system_clock_24m_pll_hxtal(void)
{
    uint32_t timeout = 0U;
    uint32_t stab_flag = 0U;

    /* enable HXTAL */
    RCU_CTL |= RCU_CTL_HXTALEN;

    /* wait until HXTAL is stable or the startup time is longer than HXTAL_STARTUP_TIMEOUT */
    do {
        timeout++;
        stab_flag = (RCU_CTL & RCU_CTL_HXTALSTB);
    } while((0U == stab_flag) && (HXTAL_STARTUP_TIMEOUT != timeout));

    /* if fail */
    if(0U == (RCU_CTL & RCU_CTL_HXTALSTB)) {
        while(1) {
        }
    }

    FMC_WS = (FMC_WS & (~FMC_WS_WSCNT)) | WS_WSCNT(0);

    RCU_APB1EN |= RCU_APB1EN_PMUEN;

    /* HXTAL is stable */
    /* AHB = SYSCLK/8 */
    RCU_CFG0 |= RCU_AHB_CKSYS_DIV8;
    /* APB2 = AHB/1 */
    RCU_CFG0 |= RCU_APB2_CKAHB_DIV1;
    /* APB1 = AHB/2 */
    RCU_CFG0 |= RCU_APB1_CKAHB_DIV2;

    /* CK_PLL = (HXTAL / 2) * 6 = 24 MHz */
    RCU_CFG1 |=  RCU_PREDV_DIV2;
    RCU_CFG0 &= ~(RCU_CFG0_PLLMF | RCU_CFG0_PLLMF_4);
    RCU_CFG0 |= (RCU_PLLSRC_HXTAL | RCU_PLL_MUL6);

    /* enable PLL */
    RCU_CTL |= RCU_CTL_PLLEN;

    /* wait until PLL is stable */
    while(0U == (RCU_CTL & RCU_CTL_PLLSTB)) {
    }

    /* select PLL as system clock */
    RCU_CFG0 &= ~RCU_CFG0_SCS;
    RCU_CFG0 |= RCU_CKSYSSRC_PLL;

    /* wait until PLL is selected as system clock */
    while((RCU_CFG0 & RCU_CFG0_SCSS) != RCU_SCSS_PLL) {
    }

    RCU_MODIFY_UP_3(0x50);
}

#elif defined (__SYSTEM_CLOCK_48M_PLL_HXTAL)
/*!
    \brief      configure the system clock to 48M by PLL which selects HXTAL as its clock source
    \param[in]  none
    \param[out] none
    \retval     none
*/
static void system_clock_48m_pll_hxtal(void)
{
    uint32_t timeout = 0U;
    uint32_t stab_flag = 0U;

    /* enable HXTAL */
    RCU_CTL |= RCU_CTL_HXTALEN;

    /* wait until HXTAL is stable or the startup time is longer than HXTAL_STARTUP_TIMEOUT */
    do {
        timeout++;
        stab_flag = (RCU_CTL & RCU_CTL_HXTALSTB);
    } while((0U == stab_flag) && (HXTAL_STARTUP_TIMEOUT != timeout));

    /* if fail */
    if(0U == (RCU_CTL & RCU_CTL_HXTALSTB)) {
        while(1) {
        }
    }

    FMC_WS = (FMC_WS & (~FMC_WS_WSCNT)) | WS_WSCNT(1);

    RCU_APB1EN |= RCU_APB1EN_PMUEN;

    /* HXTAL is stable */
    /* AHB = SYSCLK/8 */
    RCU_CFG0 |= RCU_AHB_CKSYS_DIV8;
    /* APB2 = AHB/1 */
    RCU_CFG0 |= RCU_APB2_CKAHB_DIV1;
    /* APB1 = AHB/2 */
    RCU_CFG0 |= RCU_APB1_CKAHB_DIV2;

    /* CK_PLL = (HXTAL / 2) * 12 = 48 MHz */
    RCU_CFG1 |=  RCU_PREDV_DIV2;
    RCU_CFG0 &= ~(RCU_CFG0_PLLMF | RCU_CFG0_PLLMF_4);
    RCU_CFG0 |= (RCU_PLLSRC_HXTAL | RCU_PLL_MUL12);

    /* enable PLL */
    RCU_CTL |= RCU_CTL_PLLEN;

    /* wait until PLL is stable */
    while(0U == (RCU_CTL & RCU_CTL_PLLSTB)) {
    }

    /* select PLL as system clock */
    RCU_CFG0 &= ~RCU_CFG0_SCS;
    RCU_CFG0 |= RCU_CKSYSSRC_PLL;

    /* wait until PLL is selected as system clock */
    while((RCU_CFG0 & RCU_CFG0_SCSS) != RCU_SCSS_PLL) {
    }

    RCU_MODIFY_UP_3(0x50);
}

#elif defined (__SYSTEM_CLOCK_72M_PLL_HXTAL)
/*!
    \brief      configure the system clock to 72M by PLL which selects HXTAL as its clock source
    \param[in]  none
    \param[out] none
    \retval     none
*/
static void system_clock_72m_pll_hxtal(void)
{
    uint32_t timeout = 0U;
    uint32_t stab_flag = 0U;

    /* enable HXTAL */
    RCU_CTL |= RCU_CTL_HXTALEN;

    /* wait until HXTAL is stable or the startup time is longer than HXTAL_STARTUP_TIMEOUT */
    do {
        timeout++;
        stab_flag = (RCU_CTL & RCU_CTL_HXTALSTB);
    } while((0U == stab_flag) && (HXTAL_STARTUP_TIMEOUT != timeout));

    /* if fail */
    if(0U == (RCU_CTL & RCU_CTL_HXTALSTB)) {
        while(1) {
        }
    }

    FMC_WS = (FMC_WS & (~FMC_WS_WSCNT)) | WS_WSCNT(2);

    RCU_APB1EN |= RCU_APB1EN_PMUEN;

    /* HXTAL is stable */
    /* AHB = SYSCLK/8 */
    RCU_CFG0 |= RCU_AHB_CKSYS_DIV8;
    /* APB2 = AHB/1 */
    RCU_CFG0 |= RCU_APB2_CKAHB_DIV1;
    /* APB1 = AHB/2 */
    RCU_CFG0 |= RCU_APB1_CKAHB_DIV2;

    /* CK_PLL = (HXTAL / 2) * 18 = 72 MHz */
    RCU_CFG1 |=  RCU_PREDV_DIV2;
    RCU_CFG0 &= ~(RCU_CFG0_PLLMF | RCU_CFG0_PLLMF_4);
    RCU_CFG0 |= (RCU_PLLSRC_HXTAL | RCU_PLL_MUL18);

    /* enable PLL */
    RCU_CTL |= RCU_CTL_PLLEN;

    /* wait until PLL is stable */
    while(0U == (RCU_CTL & RCU_CTL_PLLSTB)) {
    }

    /* select PLL as system clock */
    RCU_CFG0 &= ~RCU_CFG0_SCS;
    RCU_CFG0 |= RCU_CKSYSSRC_PLL;

    /* wait until PLL is selected as system clock */
    while((RCU_CFG0 & RCU_CFG0_SCSS) != RCU_SCSS_PLL) {
    }

    RCU_MODIFY_UP_3(0x100);
}

#elif defined (__SYSTEM_CLOCK_100M_PLL_HXTAL)
/*!
    \brief      configure the system clock to 100M by PLL which selects HXTAL as its clock source
    \param[in]  none
    \param[out] none
    \retval     none
*/
static void system_clock_100m_pll_hxtal(void)
{
    uint32_t timeout = 0U;
    uint32_t stab_flag = 0U;

    /* enable HXTAL */
    RCU_CTL |= RCU_CTL_HXTALEN;

    /* wait until HXTAL is stable or the startup time is longer than HXTAL_STARTUP_TIMEOUT */
    do {
        timeout++;
        stab_flag = (RCU_CTL & RCU_CTL_HXTALSTB);
    } while((0U == stab_flag) && (HXTAL_STARTUP_TIMEOUT != timeout));

    /* if fail */
    if(0U == (RCU_CTL & RCU_CTL_HXTALSTB)) {
        while(1) {
        }
    }

    FMC_WS = (FMC_WS & (~FMC_WS_WSCNT)) | WS_WSCNT(3);

    RCU_APB1EN |= RCU_APB1EN_PMUEN;

    /* HXTAL is stable */
    /* AHB = SYSCLK/8 */
    RCU_CFG0 |= RCU_AHB_CKSYS_DIV8;
    /* APB2 = AHB/1 */
    RCU_CFG0 |= RCU_APB2_CKAHB_DIV1;
    /* APB1 = AHB/2 */
    RCU_CFG0 |= RCU_APB1_CKAHB_DIV2;

    /* CK_PLL = (HXTAL / 2) * 25 = 100 MHz */
    RCU_CFG1 |=  RCU_PREDV_DIV2;
    RCU_CFG0 &= ~(RCU_CFG0_PLLMF | RCU_CFG0_PLLMF_4);
    RCU_CFG0 |= (RCU_PLLSRC_HXTAL | RCU_PLL_MUL25);

    /* enable PLL */
    RCU_CTL |= RCU_CTL_PLLEN;

    /* wait until PLL is stable */
    while(0U == (RCU_CTL & RCU_CTL_PLLSTB)) {
    }

    /* select PLL as system clock */
    RCU_CFG0 &= ~RCU_CFG0_SCS;
    RCU_CFG0 |= RCU_CKSYSSRC_PLL;

    /* wait until PLL is selected as system clock */
    while((RCU_CFG0 & RCU_CFG0_SCSS) != RCU_SCSS_PLL) {
    }

    RCU_MODIFY_UP_3(0x100);
}
#endif /* __SYSTEM_CLOCK_IRC8M */

/*!
    \brief      update the SystemCoreClock with current core clock retrieved from CPU registers
    \param[in]  none
    \param[out] none
    \retval     none
*/
void SystemCoreClockUpdate(void)
{
    uint32_t sws;
    uint32_t pllsel, pllmf, ck_src, idx, clk_exp;
    uint32_t predv0;

    /* exponent of AHB, APB1 and APB2 clock divider */
    uint8_t ahb_exp[16] = {0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 6, 7, 8, 9};

    sws = RCU_CFG0 & RCU_CFG0_SCSS;
    switch(sws) {
    /* IRC8M is selected as CK_SYS */
    case RCU_SCSS_IRC8M:
        SystemCoreClock = IRC8M_VALUE;
        break;
    /* HXTAL is selected as CK_SYS */
    case RCU_SCSS_HXTAL:
        SystemCoreClock = HXTAL_VALUE;
        break;
    /* PLL is selected as CK_SYS */
    case RCU_SCSS_PLL:
        /* PLL clock source selection, HXTAL or IRC8M/2 */
        pllsel = (RCU_CFG0 & RCU_CFG0_PLLSEL);

        if(RCU_PLLSRC_HXTAL == pllsel) {
            /* PLL clock source is HXTAL */
            ck_src = HXTAL_VALUE;
            predv0 = (RCU_CFG1 & RCU_CFG1_PREDV) + 1U;
            ck_src /= predv0;
        } else {
            /* PLL clock source is IRC8M/2 */
            ck_src = IRC8M_VALUE / 2U;
        }

        /* PLL multiplication factor */
        pllmf = GET_BITS(RCU_CFG0, 18, 21);
        pllmf += ((RCU_CFG0 & RCU_CFG0_PLLMF_4) ? 15U : 0U);
        pllmf += ((0xFU == GET_BITS(RCU_CFG0, 18, 21)) ? 1U : 2U);

        SystemCoreClock = ck_src * pllmf;

        break;
    /* IRC8M is selected as CK_SYS */
    default:
        SystemCoreClock = IRC8M_VALUE;
        break;
    }

    /* calculate AHB clock frequency */
    idx = GET_BITS(RCU_CFG0, 4, 7);
    clk_exp = ahb_exp[idx];
    SystemCoreClock >>= clk_exp;
}

#ifdef __FIRMWARE_VERSION_DEFINE
/*!
    \brief      get firmware version
    \param[in]  none
    \param[out] none
    \retval     firmware version
*/
uint32_t gd32a50x_firmware_version_get(void)
{
    return __GD32A50X_STDPERIPH_VERSION;
}
#endif /* __FIRMWARE_VERSION_DEFINE */
