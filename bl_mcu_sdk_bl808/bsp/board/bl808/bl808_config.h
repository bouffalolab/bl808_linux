/**
 * @file bl602_config.h
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

#ifndef __BL808_CONFIG_H__
#define __BL808_CONFIG_H__

#if defined(bl808_bootrom)
#include "bl808_bootrom/peripheral_config.h"
#include "bl808_bootrom/clock_config.h"
#include "bl808_bootrom/pinmux_config.h"
#elif defined(bl808_barebone)
#include "bl808_barebone/peripheral_config.h"
#include "bl808_barebone/clock_config.h"
#include "bl808_barebone/pinmux_config.h"
#elif defined(bl808_fpga_c906)
#include "bl808_fpga_c906/peripheral_config.h"
#include "bl808_fpga_c906/clock_config.h"
#include "bl808_fpga_c906/pinmux_config.h"
#elif defined(bl808_fpga_e907)
#include "bl808_fpga_e907/peripheral_config.h"
#include "bl808_fpga_e907/clock_config.h"
#include "bl808_fpga_e907/pinmux_config.h"
#elif defined(bl808_boot2)
#include "bl808_boot2/peripheral_config.h"
#include "bl808_boot2/clock_config.h"
#include "bl808_boot2/pinmux_config.h"
#else
#error "do not find board,please check your board name"
#endif

#endif