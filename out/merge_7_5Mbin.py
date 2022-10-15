import os
import sys

def bl_create_flash_default_data(length):
    datas = bytearray(length)
    for i in range(length):
        datas[i] = 0xff
    return datas

def bl_gen_linux_flash_bin():
    whole_img_data = bl_create_flash_default_data((7680*1024)-335872)  # offset 0x52000
    linux_dtb_file = "./hw.dtb.5M" #0x51ff8000  64k
    linux_opensbi_file = "./fw_jump.bin" # 0x3eff0000  64k
    linux_rootfs_file = "./squashfs_test.img"  # 0x58400000 4M
    linux_image_file = "./Image.lz4"  # 0x50000000 4M
    linux_out_img_file = "./whole_img_linux.bin"
    linux_dtb_file_size = os.stat(linux_dtb_file).st_size
    print("dtb size:", linux_dtb_file_size)
    fp = open(linux_dtb_file, 'rb')
    data0 = fp.read() + bytearray(0)
    fp.close()
    whole_img_data[0x0:0x0+len(data0)] = data0  # 0x0~0x10000 64k
    linux_opensbi_file_size = os.stat(linux_opensbi_file).st_size
    print("opensbi size:",linux_opensbi_file_size)
    fp = open(linux_opensbi_file, 'rb')
    data1 = fp.read() + bytearray(0)
    fp.close()
    whole_img_data[0x10000:0x10000+len(data1)] = data1  # 0x10000~0x20000 64k
    linux_image_file_size = os.stat(linux_image_file).st_size
    print("kernel img size:",linux_image_file_size)

    b0 = (linux_image_file_size & 0xff000000) >> 24
    b1 = (linux_image_file_size & 0xff0000) >> 16
    b2 = (linux_image_file_size & 0xff00) >> 8
    b3 = linux_image_file_size & 0xff
    # print(b0)
    # print(b1)
    # print(b2)
    # print(b3)
    header2 = [0x00,0x00,0x00,0x50,b3,b2,b1,b0]
    whole_img_data[0x1fff8:0x20000] = bytearray((header2)) # image header
    fp = open(linux_image_file, 'rb')
    data2 = fp.read() + bytearray(0)
    fp.close()
    whole_img_data[0x20000:0x20000+len(data2)] = data2 # 4M
    linux_rootfs_file_size = os.stat(linux_rootfs_file).st_size
    print("rootfs size:",linux_rootfs_file_size)
    fp = open(linux_rootfs_file, 'rb')
    data3 = fp.read() + bytearray(0)
    fp.close()
    # whole_img_data[0x480000-0x52000:0x480000-0x52000+len(data3)] = data3 #3M  start 5M
    whole_img_data[0x480000:0x480000+len(data3)] = data3 #3M  start 5M
    fp = open(linux_out_img_file, 'wb+')
    fp.write(whole_img_data)
    fp.close()

if __name__ == '__main__':
    print("merge bin start...")
    bl_gen_linux_flash_bin()
    print("merge done!")
