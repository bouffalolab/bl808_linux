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
sudo apt install flex bison libncurses-dev device-tree-compiler lz4
```

