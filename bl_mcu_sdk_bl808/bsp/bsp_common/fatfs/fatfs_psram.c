/**
 * @file fatfs_psram.c
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
#include "misc.h"

#define PSRAM_BLOCK_SIZE 512

extern const char *FR_Table[];

int ram_disk_status(void)
{
    return 0;
}
int ram_disk_initialize(void)
{
    return RES_OK;
}
int ram_disk_read(BYTE *buff, LBA_t sector, UINT count)
{
    arch_memcpy_fast((void *)buff, (uint32_t *)(PSRAM_START_ADDR + sector * PSRAM_BLOCK_SIZE), count * PSRAM_BLOCK_SIZE);

    return 0;
}
int ram_disk_write(const BYTE *buff, LBA_t sector, UINT count)
{
    arch_memcpy_fast((uint32_t *)(PSRAM_START_ADDR + sector * PSRAM_BLOCK_SIZE), (void *)buff, count * PSRAM_BLOCK_SIZE);
    return 0;
}
int ram_disk_ioctl(BYTE cmd, void *buff)
{
    int result = 0;

    switch (cmd) {
        case CTRL_SYNC:
            result = RES_OK;
            break;

        case GET_SECTOR_SIZE:
            *(WORD *)buff = PSRAM_BLOCK_SIZE;
            result = RES_OK;
            break;

        case GET_BLOCK_SIZE:
            *(DWORD *)buff = 1;
            result = RES_OK;
            break;

        case GET_SECTOR_COUNT:
            *(DWORD *)buff = PSRAM_BLOCK_COUNT;
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

void fatfs_psram_driver_register(void)
{
    FATFS_DiskioDriverTypeDef RamDiskioDriver;

    memset(&RamDiskioDriver, 0, sizeof(FATFS_DiskioDriverTypeDef));

    RamDiskioDriver.RAM_disk_status = ram_disk_status;
    RamDiskioDriver.RAM_disk_initialize = ram_disk_initialize;
    RamDiskioDriver.RAM_disk_write = ram_disk_write;
    RamDiskioDriver.RAM_disk_read = ram_disk_read;
    RamDiskioDriver.RAM_disk_ioctl = ram_disk_ioctl;
    RamDiskioDriver.Translate_Result_Code = Translate_Result_Code;
    disk_driver_callback_init(&RamDiskioDriver);
}
