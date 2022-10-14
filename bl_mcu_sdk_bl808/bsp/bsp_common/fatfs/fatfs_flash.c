/**
 * @file fatfs_flash.c
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

#include "diskio.h"
#include "string.h"
#include "hal_flash.h"

#define FLASH_BLOCK_SIZE  4096

extern const char *FR_Table[];

int flash_disk_status(void)
{
    return 0;
}
int flash_disk_initialize(void)
{
    return RES_OK;
}
int flash_disk_read(BYTE *buff, LBA_t sector, UINT count)
{
    flash_read(FLASH_START_ADDR + sector * FLASH_BLOCK_SIZE, (uint8_t *)buff, count * FLASH_BLOCK_SIZE);
    return 0;
}
int flash_disk_write(const BYTE *buff, LBA_t sector, UINT count)
{
    flash_erase(FLASH_START_ADDR + sector * FLASH_BLOCK_SIZE, 4096);
    flash_write(FLASH_START_ADDR + sector * FLASH_BLOCK_SIZE, (uint8_t *)buff, count * FLASH_BLOCK_SIZE);
    return 0;
}
int flash_disk_ioctl(BYTE cmd, void *buff)
{
    int result = 0;

    switch (cmd) {
        case CTRL_SYNC:
            result = RES_OK;
            break;

        case GET_SECTOR_SIZE:
            *(WORD *)buff = FLASH_BLOCK_SIZE;
            result = RES_OK;
            break;

        case GET_BLOCK_SIZE:
            *(DWORD *)buff = 1;
            result = RES_OK;
            break;

        case GET_SECTOR_COUNT:
            *(DWORD *)buff = FLASH_BLOCK_COUNT;
            result = RES_OK;
            break;

        default:
            result = RES_PARERR;
            break;
    }

    return result;
}

DSTATUS Translate_Result_Code(int result)
{
    //MSG("%s\r\n", FR_Table[result]);
    return result;
}

void fatfs_flash_driver_register(void)
{
    FATFS_DiskioDriverTypeDef FlashDiskioDriver;

    memset(&FlashDiskioDriver, 0, sizeof(FATFS_DiskioDriverTypeDef));

    FlashDiskioDriver.FLASH_disk_status = flash_disk_status;
    FlashDiskioDriver.FLASH_disk_initialize = flash_disk_initialize;
    FlashDiskioDriver.FLASH_disk_write = flash_disk_write;
    FlashDiskioDriver.FLASH_disk_read = flash_disk_read;
    FlashDiskioDriver.FLASH_disk_ioctl = flash_disk_ioctl;
    FlashDiskioDriver.Translate_Result_Code = Translate_Result_Code;
    disk_driver_callback_init(&FlashDiskioDriver);
}
