# bl808_linux

bl808 Linux

```bash
.
├── bl808_dts         # kernel dts file
├── bl_mcu_sdk_bl808  # bl_mcu_sdk for build low load bin
├── build.sh          # build script
├── linux-5.10.4-808  # linux kernel code
├── opensbi-0.6-808   # opensbi code
├── out               # bin file output dir
├── toolchain         # build need toolchain
└── README.md         # readme file
```

## Environment Setup

Ubuntu20.04 needs to use apt to install the package:

```bash
sudo apt install flex bison libncurses-dev device-tree-compiler lz4 bc
```

Download toolchains

```bash
mkdir -p toolchain/cmake toolchain/elf_newlib_toolchain toolchain/linux_toolchain
curl https://cmake.org/files/v3.19/cmake-3.19.3-Linux-x86_64.tar.gz | tar xz -C toolchain/cmake/ --strip-components=1
curl https://occ-oss-prod.oss-cn-hangzhou.aliyuncs.com/resource//1663142243961/Xuantie-900-gcc-elf-newlib-x86_64-V2.6.1-20220906.tar.gz | tar xz -C toolchain/elf_newlib_toolchain/ --strip-components=1
curl https://occ-oss-prod.oss-cn-hangzhou.aliyuncs.com/resource//1663142514282/Xuantie-900-gcc-linux-5.10.4-glibc-x86_64-V2.6.1-20220906.tar.gz | tar xz -C toolchain/linux_toolchain/ --strip-components=1
```

## Compile

Step by step

```bash
./build.sh --help
./build.sh opensbi
./build.sh kernel_config
./build.sh kernel
./build.sh dtb
./build.sh low_load
./build.sh whole_bin
```

Or

```bash
./build.sh all
```

Then find the firmwares under out

```bash
├── fw_jump.bin
├── hw.dtb.5M
├── Image.lz4
├── squashfs_test.img 
├── low_load_bl808_d0.bin 
├── low_load_bl808_m0.bin 
├── merge_7_5Mbin.py
└── whole_img_linux.bin
```

## Download Firmware

- Get the latest version of DevCube from http://dev.bouffalolab.com/download
- Connect BL808 board with PC by USB cable
- Set BL808 board to programming mode
    + Press BOOT button
    + Press RESET button
    + Release RESET button
    + Release BOOT button
- Run DevCube, select [BL808], and switch to [MCU] page
- Select the uart port and set baudrate with 2000000
- M0 Group[Group0] Image Addr [0x58000000] [PATH to low_load_bl808_m0.bin]
- D0 Group[Group1] Image Addr [0x58000000] [PATH to low_load_bl808_d0.bin]
- Click 'Create & Download' and wait until it's done
- Switch to [IOT] page
- Enable 'Single Download', set Address with 0xD2000, choose [PATH to whole_image_linux.bin]
- Click 'Create & Download' again and wait until it's done

## Boot BL808 board

Press and release RESET button, E907 will output log by IO14(TX)/IO15(RX) and C906 by IO5(RX)/IO8(TX)

## Changelog

2023-08-21
- Add Wi-Fi support. https://github.com/bouffalolab/blwnet_xram
