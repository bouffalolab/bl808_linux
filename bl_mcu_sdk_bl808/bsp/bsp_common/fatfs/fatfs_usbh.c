/**
 * @file fatfs_spi_sd.c
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
#include "usbh_core.h"
#include "usbh_msc.h"

struct usbh_msc *active_msc_class;

extern const char *FR_Table[];

int usb_disk_status(void)
{
    return 0;
}
int usb_disk_initialize(void)
{
    active_msc_class = (struct usbh_msc *)usbh_find_class_instance("/dev/sda");
    if (active_msc_class == NULL) {
        printf("do not find /dev/sda\r\n");
        return -1;
    }
    return RES_OK;
}
int usb_disk_read(BYTE *buff, LBA_t sector, UINT count)
{
    return usbh_msc_scsi_read10(active_msc_class, sector, buff, count);
}
int usb_disk_write(const BYTE *buff, LBA_t sector, UINT count)
{
    return usbh_msc_scsi_write10(active_msc_class, sector, buff, count);
}
int usb_disk_ioctl(BYTE cmd, void *buff)
{
    int result = 0;

    switch (cmd) {
        case CTRL_SYNC:
            result = RES_OK;
            break;

        case GET_SECTOR_SIZE:
            *(WORD *)buff = active_msc_class->blocksize;
            result = RES_OK;
            break;

        case GET_BLOCK_SIZE:
            *(DWORD *)buff = 1;
            result = RES_OK;
            break;

        case GET_SECTOR_COUNT:
            *(DWORD *)buff = active_msc_class->blocknum;
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

void fatfs_usbh_driver_register(void)
{
    FATFS_DiskioDriverTypeDef USBDiskioDriver;

    memset(&USBDiskioDriver, 0, sizeof(FATFS_DiskioDriverTypeDef));

    USBDiskioDriver.USB_disk_status = usb_disk_status;
    USBDiskioDriver.USB_disk_initialize = usb_disk_initialize;
    USBDiskioDriver.USB_disk_write = usb_disk_write;
    USBDiskioDriver.USB_disk_read = usb_disk_read;
    USBDiskioDriver.USB_disk_ioctl = usb_disk_ioctl;
    USBDiskioDriver.Translate_Result_Code = Translate_Result_Code;
    disk_driver_callback_init(&USBDiskioDriver);
}
