/**
 * @file board.c
 * @brief
 *
 * Copyright (c) 2021 Bouffalolab team
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 */
#include "hal_clock.h"
#include "hal_uart.h"
#include "bl808_glb.h"
#include "bl808_psram_uhs.h"
#include "bl808_config.h"
#include "bl808_tzc_sec.h"
#include "bflb_platform.h"
#include "bl808_ef_cfg.h"
#include "bl808_uhs_phy.h"

#define D0_DEBUG 1

#define WB_4MB_PSRAM   (1)
#define UHS_32MB_PSRAM (2)
#define UHS_64MB_PSRAM (3)
#define WB_32MB_PSRAM  (4)
#define NONE_UHS_PSRAM (-1)

struct pin_mux_cfg {
    uint8_t pin;
    uint8_t func;
};

__UNUSED__ static const struct pin_mux_cfg af_pin_table[] = {
#ifdef CONFIG_GPIO0_FUNC
    { .pin = GLB_GPIO_PIN_0,
      .func = CONFIG_GPIO0_FUNC },
#endif
#ifdef CONFIG_GPIO1_FUNC
    { .pin = GLB_GPIO_PIN_1,
      .func = CONFIG_GPIO1_FUNC },
#endif
#ifdef CONFIG_GPIO2_FUNC
    { .pin = GLB_GPIO_PIN_2,
      .func = CONFIG_GPIO2_FUNC },
#endif
#ifdef CONFIG_GPIO3_FUNC
    { .pin = GLB_GPIO_PIN_3,
      .func = CONFIG_GPIO3_FUNC },
#endif
#ifdef CONFIG_GPIO4_FUNC
    { .pin = GLB_GPIO_PIN_4,
      .func = CONFIG_GPIO4_FUNC },
#endif
#ifdef CONFIG_GPIO5_FUNC
    { .pin = GLB_GPIO_PIN_5,
      .func = CONFIG_GPIO5_FUNC },
#endif
#ifdef CONFIG_GPIO6_FUNC
    { .pin = GLB_GPIO_PIN_6,
      .func = CONFIG_GPIO6_FUNC },
#endif
#ifdef CONFIG_GPIO7_FUNC
    { .pin = GLB_GPIO_PIN_7,
      .func = CONFIG_GPIO7_FUNC },
#endif
#ifdef CONFIG_GPIO8_FUNC
    { .pin = GLB_GPIO_PIN_8,
      .func = CONFIG_GPIO8_FUNC },
#endif
#ifdef CONFIG_GPIO9_FUNC
    { .pin = GLB_GPIO_PIN_9,
      .func = CONFIG_GPIO9_FUNC },
#endif
#ifdef CONFIG_GPIO10_FUNC
    { .pin = GLB_GPIO_PIN_10,
      .func = CONFIG_GPIO10_FUNC },
#endif
#ifdef CONFIG_GPIO11_FUNC
    { .pin = GLB_GPIO_PIN_11,
      .func = CONFIG_GPIO11_FUNC },
#endif
#ifdef CONFIG_GPIO12_FUNC
    { .pin = GLB_GPIO_PIN_12,
      .func = CONFIG_GPIO12_FUNC },
#endif
#ifdef CONFIG_GPIO13_FUNC
    { .pin = GLB_GPIO_PIN_13,
      .func = CONFIG_GPIO13_FUNC },
#endif
#ifdef CONFIG_GPIO14_FUNC
    { .pin = GLB_GPIO_PIN_14,
      .func = CONFIG_GPIO14_FUNC },
#endif
#ifdef CONFIG_GPIO15_FUNC
    { .pin = GLB_GPIO_PIN_15,
      .func = CONFIG_GPIO15_FUNC },
#endif
#ifdef CONFIG_GPIO16_FUNC
    { .pin = GLB_GPIO_PIN_16,
      .func = CONFIG_GPIO16_FUNC },
#endif
#ifdef CONFIG_GPIO17_FUNC
    { .pin = GLB_GPIO_PIN_17,
      .func = CONFIG_GPIO17_FUNC },
#endif
#ifdef CONFIG_GPIO18_FUNC
    { .pin = GLB_GPIO_PIN_18,
      .func = CONFIG_GPIO18_FUNC },
#endif
#ifdef CONFIG_GPIO19_FUNC
    { .pin = GLB_GPIO_PIN_19,
      .func = CONFIG_GPIO19_FUNC },
#endif
#ifdef CONFIG_GPIO20_FUNC
    { .pin = GLB_GPIO_PIN_20,
      .func = CONFIG_GPIO20_FUNC },
#endif
#ifdef CONFIG_GPIO21_FUNC
    { .pin = GLB_GPIO_PIN_21,
      .func = CONFIG_GPIO21_FUNC },
#endif
#ifdef CONFIG_GPIO22_FUNC
    { .pin = GLB_GPIO_PIN_22,
      .func = CONFIG_GPIO22_FUNC },
#endif
#ifdef CONFIG_GPIO23_FUNC
    { .pin = GLB_GPIO_PIN_23,
      .func = CONFIG_GPIO23_FUNC },
#endif
#ifdef CONFIG_GPIO24_FUNC
    { .pin = GLB_GPIO_PIN_24,
      .func = CONFIG_GPIO24_FUNC },
#endif
#ifdef CONFIG_GPIO25_FUNC
    { .pin = GLB_GPIO_PIN_25,
      .func = CONFIG_GPIO25_FUNC },
#endif
#ifdef CONFIG_GPIO26_FUNC
    { .pin = GLB_GPIO_PIN_26,
      .func = CONFIG_GPIO26_FUNC },
#endif
#ifdef CONFIG_GPIO27_FUNC
    { .pin = GLB_GPIO_PIN_27,
      .func = CONFIG_GPIO27_FUNC },
#endif
#ifdef CONFIG_GPIO28_FUNC
    { .pin = GLB_GPIO_PIN_28,
      .func = CONFIG_GPIO28_FUNC },
#endif
#ifdef CONFIG_GPIO29_FUNC
    { .pin = GLB_GPIO_PIN_29,
      .func = CONFIG_GPIO29_FUNC },
#endif
#ifdef CONFIG_GPIO30_FUNC
    { .pin = GLB_GPIO_PIN_30,
      .func = CONFIG_GPIO30_FUNC },
#endif
#ifdef CONFIG_GPIO31_FUNC
    { .pin = GLB_GPIO_PIN_31,
      .func = CONFIG_GPIO31_FUNC },
#endif
#ifdef CONFIG_GPIO32_FUNC
    { .pin = GLB_GPIO_PIN_32,
      .func = CONFIG_GPIO32_FUNC },
#endif
#ifdef CONFIG_GPIO33_FUNC
    { .pin = GLB_GPIO_PIN_33,
      .func = CONFIG_GPIO33_FUNC },
#endif
#ifdef CONFIG_GPIO34_FUNC
    { .pin = GLB_GPIO_PIN_34,
      .func = CONFIG_GPIO34_FUNC },
#endif
#ifdef CONFIG_GPIO35_FUNC
    { .pin = GLB_GPIO_PIN_35,
      .func = CONFIG_GPIO35_FUNC },
#endif
#ifdef CONFIG_GPIO36_FUNC
    { .pin = GLB_GPIO_PIN_36,
      .func = CONFIG_GPIO36_FUNC },
#endif
#ifdef CONFIG_GPIO37_FUNC
    { .pin = GLB_GPIO_PIN_37,
      .func = CONFIG_GPIO37_FUNC },
#endif
#ifdef CONFIG_GPIO38_FUNC
    { .pin = GLB_GPIO_PIN_38,
      .func = CONFIG_GPIO38_FUNC },
#endif
#ifdef CONFIG_GPIO39_FUNC
    { .pin = GLB_GPIO_PIN_39,
      .func = CONFIG_GPIO39_FUNC },
#endif
#ifdef CONFIG_GPIO40_FUNC
    { .pin = GLB_GPIO_PIN_40,
      .func = CONFIG_GPIO40_FUNC },
#endif
#ifdef CONFIG_GPIO41_FUNC
    { .pin = GLB_GPIO_PIN_41,
      .func = CONFIG_GPIO41_FUNC },
#endif
#ifdef CONFIG_GPIO42_FUNC
    { .pin = GLB_GPIO_PIN_42,
      .func = CONFIG_GPIO42_FUNC },
#endif
#ifdef CONFIG_GPIO43_FUNC
    { .pin = GLB_GPIO_PIN_43,
      .func = CONFIG_GPIO43_FUNC },
#endif
#ifdef CONFIG_GPIO44_FUNC
    { .pin = GLB_GPIO_PIN_44,
      .func = CONFIG_GPIO44_FUNC },
#endif
#ifdef CONFIG_GPIO45_FUNC
    { .pin = GLB_GPIO_PIN_45,
      .func = CONFIG_GPIO45_FUNC },
#endif
};

__UNUSED__ static void board_d0_init_jtag_gpio(void)
{
    GLB_GPIO_Cfg_Type cfg;
    uint8_t gpiopins[4];
    uint8_t gpiofuns[4];
    uint8_t num;

#define GPIO_FUN_JTAG GPIO_FUN_JTAG_D0

    gpiopins[0] = GLB_GPIO_PIN_0;
    gpiofuns[0] = GPIO_FUN_JTAG;
    gpiopins[1] = GLB_GPIO_PIN_1;
    gpiofuns[1] = GPIO_FUN_JTAG;
    gpiopins[2] = GLB_GPIO_PIN_2;
    gpiofuns[2] = GPIO_FUN_JTAG;
    gpiopins[3] = GLB_GPIO_PIN_3;
    gpiofuns[3] = GPIO_FUN_JTAG;
    num = 4;
    cfg.pullType = GPIO_PULL_NONE;
    cfg.drive = 0;
    cfg.smtCtrl = 1;

    for (uint8_t i = 0; i < num; i++) {
        cfg.gpioPin = gpiopins[i];
        cfg.gpioFun = gpiofuns[i];
        cfg.gpioMode = GPIO_MODE_AF;
        GLB_GPIO_Init(&cfg);
    }
}

__UNUSED__ static void board_pin_mux_init(void)
{
    GLB_GPIO_Cfg_Type gpio_cfg;

    gpio_cfg.drive = 1;
    gpio_cfg.smtCtrl = 1;
    gpio_cfg.outputMode = 0;

    for (int i = 0; i < sizeof(af_pin_table) / sizeof(af_pin_table[0]); i++) {
        gpio_cfg.gpioMode = GPIO_MODE_AF;
        gpio_cfg.pullType = GPIO_PULL_UP;
        gpio_cfg.gpioPin = af_pin_table[i].pin;
        gpio_cfg.gpioFun = af_pin_table[i].func;

        if (af_pin_table[i].func == GPIO_FUN_UNUSED) {
            continue;
        } else if ((af_pin_table[i].func & 0xF0) == 0xF0) {
            /*if uart func*/
            gpio_cfg.gpioFun = GPIO_FUN_UART;
            uint8_t uart_func = af_pin_table[i].func & 0x0F;
            uint8_t uart_sig = gpio_cfg.gpioPin % 12;

            /*link to one uart sig*/
            GLB_UART_Fun_Sel((GLB_UART_SIG_Type)uart_sig, (GLB_UART_SIG_FUN_Type)uart_func);
        } else if (af_pin_table[i].func == GPIO_FUN_PWM0 || af_pin_table[i].func == GPIO_FUN_PWM1) {
            gpio_cfg.pullType = GPIO_PULL_DOWN;
        } else if ((af_pin_table[i].func & 0xE0) == 0xE0) {
            gpio_cfg.gpioFun = GPIO_FUN_CLOCK_OUT;

            uint8_t chip_out_idx = af_pin_table[i].pin % 4;
            uint8_t clk_sel = af_pin_table[i].func & 0x03;

            uint32_t tmpVal = BL_RD_REG(GLB_BASE, GLB_DIG_CLK_CFG2);

            tmpVal &= ~(3 << (GLB_CHIP_CLK_OUT_0_SEL_LEN * chip_out_idx + GLB_CHIP_CLK_OUT_0_SEL_POS));
            tmpVal &= ~(1 << (GLB_CHIP_CLK_OUT_0_EN_LEN * chip_out_idx + GLB_CHIP_CLK_OUT_0_EN_POS));
            tmpVal |= (clk_sel << (GLB_CHIP_CLK_OUT_0_SEL_LEN * chip_out_idx + GLB_CHIP_CLK_OUT_0_SEL_POS));
            tmpVal |= (1 << (GLB_CHIP_CLK_OUT_0_EN_LEN * chip_out_idx + GLB_CHIP_CLK_OUT_0_EN_POS));
            BL_WR_REG(GLB_BASE, GLB_DIG_CLK_CFG2, tmpVal);
        }
        GLB_GPIO_Init(&gpio_cfg);
    }
}

static void peripheral_clock_gate_all()
{
    uint32_t tmpVal;
    tmpVal = BL_RD_REG(GLB_BASE, GLB_CGEN_CFG1);
    tmpVal &= (~(1 << 2)); //gpip
    tmpVal &= (~(1 << 3)); //sec_dbg
    //tmpVal &= (~(1 << 4));  //sec_eng
    //tmpVal &= (~(1 << 5));  //sec_eng
    tmpVal &= (~(1 << 12)); //dma
    tmpVal &= (~(1 << 13)); //usb
    tmpVal &= (~(1 << 16)); //uart0
    tmpVal &= (~(1 << 17)); //uart1
    tmpVal &= (~(1 << 18)); //spi
    tmpVal &= (~(1 << 19)); //i2c
    tmpVal &= (~(1 << 20)); //pwm
    tmpVal &= (~(1 << 21)); //timer
    tmpVal &= (~(1 << 22)); //ir
    tmpVal &= (~(1 << 23)); //cks
    tmpVal &= (~(1 << 25)); //i2c1
    tmpVal &= (~(1 << 26)); //uart2
    tmpVal &= (~(1 << 27)); //i2s
    BL_WR_REG(GLB_BASE, GLB_CGEN_CFG1, tmpVal);

    tmpVal = BL_RD_REG(GLB_BASE, GLB_CGEN_CFG2);
    tmpVal &= (~(1 << 4));  //wifi
    tmpVal &= (~(1 << 8));  //bt/ble
    tmpVal &= (~(1 << 9));  //m154
    tmpVal &= (~(1 << 10)); //bt/ble
    tmpVal &= (~(1 << 11)); //m154
    tmpVal &= (~(1 << 19)); //emac
    //tmpVal &= (~(1 << 21)); //audio
    tmpVal &= (~(1 << 22)); //sdh
    BL_WR_REG(GLB_BASE, GLB_CGEN_CFG2, tmpVal);
}

void system_clock_init(void)
{
    GLB_Halt_CPU(GLB_CORE_ID_D0);
    GLB_Halt_CPU(GLB_CORE_ID_LP);
    GLB_Power_On_XTAL_And_PLL_CLK(XTAL_TYPE, GLB_PLL_WIFIPLL |
                                                 GLB_PLL_CPUPLL |
                                                 GLB_PLL_UHSPLL |
                                                 GLB_PLL_MIPIPLL);

#if BSP_ROOT_CLOCK_SOURCE == ROOT_CLOCK_SOURCE_XCLK
#if XTAL_TYPE == INTERNAL_RC_32M
    GLB_Set_MCU_System_CLK(GLB_MCU_SYS_CLK_RC32M);
#else
    GLB_Set_MCU_System_CLK(GLB_MCU_SYS_CLK_XTAL);
#endif
#elif BSP_ROOT_CLOCK_SOURCE == ROOT_CLOCK_SOURCE_WIFIPLL_240M
    GLB_Set_MCU_System_CLK(GLB_MCU_SYS_CLK_WIFIPLL_240M);
#elif BSP_ROOT_CLOCK_SOURCE == ROOT_CLOCK_SOURCE_WIFIPLL_320M
    GLB_Set_MCU_System_CLK(GLB_MCU_SYS_CLK_WIFIPLL_320M);
#else
#error "do not support current root clock source"
#endif
    GLB_Set_MCU_System_CLK_Div(BSP_HCLK_DIV, BSP_BCLK_DIV, BSP_LPCLK_DIV);
#if BSP_CLOCK_SOURCE_MUXPLL_160M == ROOT_CLOCK_SOURCE_WIFIPLL_160M
    GLB_Set_MCU_Muxpll_160M_Sel(0);
#elif BSP_CLOCK_SOURCE_MUXPLL_160M == ROOT_CLOCK_SOURCE_CPUPLL_160M
    GLB_Set_MCU_Muxpll_160M_Sel(1);
#elif BSP_CLOCK_SOURCE_MUXPLL_160M == ROOT_CLOCK_SOURCE_AUPLL_DIV2
    GLB_Set_MCU_Muxpll_160M_Sel(2);
#elif BSP_CLOCK_SOURCE_MUXPLL_160M == ROOT_CLOCK_SOURCE_AUPLL_DIV2P5
    GLB_Set_MCU_Muxpll_160M_Sel(3);
#else
#error "do not support current muxpll 160m clock source"
#endif

#if BSP_CLOCK_SOURCE_MUXPLL_80M == ROOT_CLOCK_SOURCE_WIFIPLL_80M
    GLB_Set_MCU_Muxpll_80M_Sel(0);
#elif BSP_CLOCK_SOURCE_MUXPLL_80M == ROOT_CLOCK_SOURCE_CPUPLL_80M
    GLB_Set_MCU_Muxpll_80M_Sel(1);
#elif BSP_CLOCK_SOURCE_MUXPLL_80M == ROOT_CLOCK_SOURCE_AUPLL_DIV5
    GLB_Set_MCU_Muxpll_80M_Sel(2);
#elif BSP_CLOCK_SOURCE_MUXPLL_80M == ROOT_CLOCK_SOURCE_AUPLL_DIV6
    GLB_Set_MCU_Muxpll_80M_Sel(3);
#else
#error "do not support current muxpll 80m clock source"
#endif

    GLB_Set_DSP_System_CLK(GLB_DSP_SYS_CLK_CPUPLL_400M);

    GLB_Release_CPU(GLB_CORE_ID_D0);
    GLB_Release_CPU(GLB_CORE_ID_LP);

    CPU_Set_MTimer_CLK(ENABLE, CPU_Get_MTimer_Source_Clock() / 1000 / 1000 - 1);
}

void peripheral_clock_init(void)
{
    uint32_t tmpVal;

    peripheral_clock_gate_all();

    tmpVal = BL_RD_REG(GLB_BASE, GLB_CGEN_CFG1);
#if defined(BSP_USING_UART0) || defined(BSP_USING_UART1) || defined(BSP_USING_UART2)
#if defined(BSP_USING_UART0)
    tmpVal |= (1 << 16);
#endif
#if defined(BSP_USING_UART1)
    tmpVal |= (1 << 17);
#endif
#if defined(BSP_USING_UART2)
    tmpVal |= (1 << 26);
#endif

#if BSP_UART_CLOCK_SOURCE == ROOT_CLOCK_SOURCE_XCLK
    GLB_Set_UART_CLK(ENABLE, HBN_UART_CLK_XCLK, BSP_UART_CLOCK_DIV);
#elif BSP_UART_CLOCK_SOURCE == BSP_CLOCK_SOURCE_BCLK
    GLB_Set_UART_CLK(ENABLE, HBN_UART_CLK_MCU_PBCLK, BSP_UART_CLOCK_DIV);
#elif BSP_UART_CLOCK_SOURCE == BSP_CLOCK_SOURCE_MUXPLL_160M
    GLB_Set_UART_CLK(ENABLE, HBN_UART_CLK_160M, BSP_UART_CLOCK_DIV);
#else
#error "do not support current uart clock source"
#endif
#endif

#if defined(BSP_USING_UART3)
#if BSP_UART_CLOCK_SOURCE == ROOT_CLOCK_SOURCE_XCLK
    GLB_Set_DSP_UART0_CLK(ENABLE, GLB_DSP_UART_CLK_DSP_XCLK, BSP_UART_CLOCK_DIV);
#elif BSP_UART_CLOCK_SOURCE == BSP_CLOCK_SOURCE_BCLK
    GLB_Set_DSP_UART0_CLK(ENABLE, GLB_DSP_UART_CLK_DSP_PBCLK, BSP_UART_CLOCK_DIV);
#elif BSP_UART_CLOCK_SOURCE == BSP_CLOCK_SOURCE_MUXPLL_160M
    GLB_Set_DSP_UART0_CLK(ENABLE, GLB_DSP_UART_CLK_MUXPLL_160M, BSP_UART_CLOCK_DIV);
#else
#error "do not support current uart clock source"
#endif
#endif

#if defined(BSP_USING_I2C0) || defined(BSP_USING_I2C1)
#if defined(BSP_USING_I2C0)
    tmpVal |= (1 << 19);
#elif defined(BSP_USING_I2C1)
    tmpVal |= (1 << 25);
#endif

#if BSP_I2C_CLOCK_SOURCE == ROOT_CLOCK_SOURCE_XCLK
    GLB_Set_I2C_CLK(ENABLE, GLB_I2C_CLK_XCLK, BSP_I2C_CLOCK_DIV);
#elif BSP_I2C_CLOCK_SOURCE == BSP_CLOCK_SOURCE_BCLK
    GLB_Set_I2C_CLK(ENABLE, GLB_I2C_CLK_BCLK, BSP_I2C_CLOCK_DIV);
#else
#error "do not support current i2c clock source"
#endif
#endif

#if defined(BSP_USING_I2C2)
#if BSP_I2C_CLOCK_SOURCE == ROOT_CLOCK_SOURCE_XCLK
    GLB_Set_DSP_I2C0_CLK(ENABLE, GLB_I2C_CLK_XCLK, BSP_I2C_CLOCK_DIV);
#elif BSP_I2C_CLOCK_SOURCE == BSP_CLOCK_SOURCE_BCLK
    GLB_Set_DSP_I2C0_CLK(ENABLE, GLB_I2C_CLK_BCLK, BSP_I2C_CLOCK_DIV);
#else
#error "do not support current i2c clock source"
#endif
#endif

#if defined(BSP_USING_I2C3)
#if BSP_I2C_CLOCK_SOURCE == ROOT_CLOCK_SOURCE_XCLK
    GLB_Set_DSP_I2C1_CLK(ENABLE, GLB_I2C_CLK_XCLK, BSP_I2C_CLOCK_DIV);
#elif BSP_I2C_CLOCK_SOURCE == BSP_CLOCK_SOURCE_BCLK
    GLB_Set_DSP_I2C1_CLK(ENABLE, GLB_I2C_CLK_BCLK, BSP_I2C_CLOCK_DIV);
#else
#error "do not support current i2c clock source"
#endif
#endif

#if defined(BSP_USING_I2S0)
    tmpVal |= (1 << 27);
    GLB_Set_I2S_CLK(ENABLE, BSP_I2S_CLOCK_DIV, GLB_I2S_DI_SEL_I2S_DI_INPUT, GLB_I2S_DO_SEL_I2S_DO_OUTPT);
#endif

#if defined(BSP_USING_SPI0)
    tmpVal |= (1 << 18);
#if BSP_SPI_CLOCK_SOURCE == ROOT_CLOCK_SOURCE_XCLK
    GLB_Set_SPI_CLK(ENABLE, GLB_SPI_CLK_XCLK, BSP_SPI_CLOCK_DIV);
#elif BSP_SPI_CLOCK_SOURCE == BSP_CLOCK_SOURCE_MUXPLL_160M
    GLB_Set_SPI_CLK(ENABLE, GLB_SPI_CLK_MCU_MUXPLL_160M, BSP_SPI_CLOCK_DIV);
#else
#error "do not support current spi clock source"
#endif
#endif

#if defined(BSP_USING_SPI1)
#if BSP_SPI_CLOCK_SOURCE == ROOT_CLOCK_SOURCE_XCLK
    GLB_Set_DSP_SPI_CLK(ENABLE, GLB_DSP_SPI_CLK_DSP_XCLK, BSP_SPI_CLOCK_DIV);
#elif BSP_SPI_CLOCK_SOURCE == BSP_CLOCK_SOURCE_MUXPLL_160M
    GLB_Set_DSP_SPI_CLK(ENABLE, GLB_DSP_SPI_CLK_DSP_MUXPLL_160M, BSP_SPI_CLOCK_DIV);
#else
#error "do not support current spi clock source"
#endif
#endif

#if defined(BSP_USING_ADC0)
    tmpVal |= (1 << 2);
#if BSP_ADC_CLOCK_SOURCE == ROOT_CLOCK_SOURCE_AUPLL_CLK
    GLB_Set_ADC_CLK(ENABLE, GLB_ADC_CLK_AUPLL, BSP_ADC_CLOCK_DIV);
#elif BSP_ADC_CLOCK_SOURCE == ROOT_CLOCK_SOURCE_XCLK
    GLB_Set_ADC_CLK(ENABLE, GLB_ADC_CLK_XCLK, BSP_ADC_CLOCK_DIV);
#else
#error "do not support current adc clock source"
#endif
#endif

#if defined(BSP_USING_DAC0)
    tmpVal |= (1 << 2);
#if BSP_ADC_CLOCK_SOURCE == ROOT_CLOCK_SOURCE_WIFIPLL_32M
    //GLB_Set_DAC_CLK(ENABLE, GLB_DAC_CLK_32M, BSP_DAC_CLOCK_DIV);
#elif BSP_ADC_CLOCK_SOURCE == ROOT_CLOCK_SOURCE_XCLK
    //GLB_Set_DAC_CLK(ENABLE, GLB_DAC_CLK_XCLK, BSP_DAC_CLOCK_DIV);
#else
#error "do not support current dac clock source"
#endif
#endif

#if defined(BSP_USING_USB)
    tmpVal |= (1 << 13);
    GLB_Set_USB_CLK_From_WIFIPLL(1);
#endif
#if defined(BSP_USING_SDH)
    uint32_t tmp_val;
    tmp_val = BL_RD_REG(PDS_BASE, PDS_CTL5);
    uint32_t tmp_val2 = BL_GET_REG_BITS_VAL(tmp_val, PDS_CR_PDS_GPIO_KEEP_EN);
    tmp_val2 &= ~(1 << 0);
    tmp_val = BL_SET_REG_BITS_VAL(tmp_val, PDS_CR_PDS_GPIO_KEEP_EN, tmp_val2);
    BL_WR_REG(PDS_BASE, PDS_CTL5, tmp_val);

    GLB_Set_SDH_CLK(1, GLB_SDH_CLK_WIFIPLL_96M, 0);
    GLB_PER_Clock_UnGate(GLB_AHB_CLOCK_SDH);
    GLB_AHB_MCU_Software_Reset(GLB_AHB_MCU_SW_SDH);
#endif

#if defined(BSP_USING_DMA)
    tmpVal |= (1 << 12);
#endif

    BL_WR_REG(GLB_BASE, GLB_CGEN_CFG1, tmpVal);

    GLB_Set_DSP_CLK(1, GLB_DSP_CLK_CPUPLL_400M, 0);
}

void board_clock_init(void)
{
    system_clock_init();
    peripheral_clock_init();
}

void bl_show_info(void)
{
    MSG("\r\n");
    MSG("  ____               __  __      _       _       _     \r\n");
    MSG(" |  _ \\             / _|/ _|    | |     | |     | |    \r\n");
    MSG(" | |_) | ___  _   _| |_| |_ __ _| | ___ | | __ _| |__  \r\n");
    MSG(" |  _ < / _ \\| | | |  _|  _/ _` | |/ _ \\| |/ _` | '_ \\ \r\n");
    MSG(" | |_) | (_) | |_| | | | || (_| | | (_) | | (_| | |_) |\r\n");
    MSG(" |____/ \\___/ \\__,_|_| |_| \\__,_|_|\\___/|_|\\__,_|_.__/ \r\n");
    MSG("\r\n");
    MSG("Build:%s,%s\r\n", __TIME__, __DATE__);
    MSG("Copyright (c) 2022 Bouffalolab team\r\n");

    MSG("#####################################################\r\n");
    MSG("          mcu xclock:%dM\r\n", Clock_System_Clock_Get(BL_SYSTEM_CLOCK_MCU_XCLK) / 1000000);
    MSG("       mcu cpu clock:%dM\r\n", Clock_System_Clock_Get(BL_SYSTEM_CLOCK_MCU_CLK) / 1000000);
    MSG("mcu peripheral clock:%dM\r\n", Clock_System_Clock_Get(BL_SYSTEM_CLOCK_MCU_PBCLK) / 1000000);

    MSG("          dsp xclock:%dM\r\n", Clock_System_Clock_Get(BL_SYSTEM_CLOCK_DSP_XCLK) / 1000000);
    MSG("       dsp cpu clock:%dM\r\n", Clock_System_Clock_Get(BL_SYSTEM_CLOCK_DSP_CLK) / 1000000);
    MSG("       dsp bus clock:%dM\r\n", Clock_System_Clock_Get(BL_SYSTEM_CLOCK_DSP_BCLK) / 1000000);
    MSG("dsp peripheral clock:%dM\r\n", Clock_System_Clock_Get(BL_SYSTEM_CLOCK_DSP_PBCLK) / 1000000);

    MSG("            lp clock:%dM\r\n", Clock_System_Clock_Get(BL_SYSTEM_CLOCK_LP_CLK) / 1000000);
    MSG("         PSRAM clock:%dM\r\n", Clock_Peripheral_Clock_Get(BL_PERIPHERAL_CLOCK_PSRAMA) / 1000000);
    MSG("UART SIG7-SIG0 :%08x\r\n", BL_RD_REG(GLB_BASE, GLB_UART_CFG1));
    MSG("UART SIG11-SIG8 :%04x\r\n", BL_RD_REG(GLB_BASE, GLB_UART_CFG2));
    MSG("#####################################################\r\n");
}

int uhs_psram_init(void)
{
    PSRAM_UHS_Cfg_Type psramDefaultCfg = {
        2000,
        PSRAM_MEM_SIZE_32MB,
        PSRAM_PAGE_SIZE_2KB,
        PSRAM_UHS_NORMAL_TEMP,
    };

    Efuse_Chip_Info_Type chip_info;
    EF_Ctrl_Get_Chip_Info(&chip_info);
    if (chip_info.psramInfo == UHS_32MB_PSRAM) {
        psramDefaultCfg.psramMemSize = PSRAM_MEM_SIZE_32MB;
    } else if (chip_info.psramInfo == UHS_64MB_PSRAM) {
        psramDefaultCfg.psramMemSize = PSRAM_MEM_SIZE_64MB;
    } else {
        return NONE_UHS_PSRAM;
    }

    Efuse_Psram_Trim_Type uhs_psram_trim;
    EF_Ctrl_Read_Psram_Trim(&uhs_psram_trim);

    //init uhs PLL; Must open uhs pll first, and then initialize uhs psram
    GLB_Config_UHS_PLL(GLB_XTAL_40M, uhsPllCfg_2000M);
    //init uhs psram ;
    // Psram_UHS_x16_Init(Clock_Peripheral_Clock_Get(BL_PERIPHERAL_CLOCK_PSRAMA) / 1000000);
    Psram_UHS_x16_Init_Override(&psramDefaultCfg);
    Tzc_Sec_PSRAMA_Access_Release();

    // example: 2000Mbps typical cal values
    uhs_phy_cal_res->rl = 39;
    uhs_phy_cal_res->rdqs = 3;
    uhs_phy_cal_res->rdq = 0;
    uhs_phy_cal_res->wl = 13;
    uhs_phy_cal_res->wdqs = 4;
    uhs_phy_cal_res->wdq = 5;
    uhs_phy_cal_res->ck = 9;
    /* TODO: use uhs psram trim update */
    set_uhs_latency_r(uhs_phy_cal_res->rl);
    cfg_dqs_rx(uhs_phy_cal_res->rdqs);
    cfg_dq_rx(uhs_phy_cal_res->rdq);
    set_uhs_latency_w(uhs_phy_cal_res->wl);
    cfg_dq_drv(uhs_phy_cal_res->wdq);
    cfg_ck_cen_drv(uhs_phy_cal_res->wdq + 4, uhs_phy_cal_res->wdq + 1);
    cfg_dqs_drv(uhs_phy_cal_res->wdqs);
    // set_odt_en();
    mr_read_back();
    return 0;
}

void board_init(void)
{
#ifndef BOOTROM
#ifdef CPU_M0
    board_clock_init();
    board_pin_mux_init();
#if D0_DEBUG
    board_d0_init_jtag_gpio();
#endif
    uint32_t ret = 0;
    ret = uhs_psram_init();
    if (ret < 0) {
        while (1)
            ;
    }
#endif
#endif
}

uint8_t board_get_debug_uart_index(void)
{
#ifdef CPU_M0
    return UART0_INDEX;
#endif
#ifdef CPU_D0
    return UART3_INDEX;
#endif
#ifdef CPU_LP
    return UART1_INDEX;
#endif
}

/* Used for boot2 iap */
void system_mtimer_clock_init(void)
{
    CPU_Reset_MTimer();
    /* set mtimer clock 1M */
    CPU_Set_MTimer_CLK(ENABLE, CPU_Get_MTimer_Source_Clock() / 1000000 - 1);
}

void system_mtimer_clock_reinit(void)
{
    /* ToDo */
}
