#-----------------------------------------------------------
# libraries
#-----------------------------------------------------------
ifeq ($(CONFIG),debug)
LIBRARIES += $(MQX_ROOTDIR)/lib/frdmk64f.cw10gcc/debug/bsp/bsp.a
LIBRARIES += $(MQX_ROOTDIR)/lib/frdmk64f.cw10gcc/debug/psp/psp.a
LIBRARIES += $(MQX_ROOTDIR)/lib/frdmk64f.cw10gcc/debug/mfs/mfs.a
LIBRARIES += $(MQX_ROOTDIR)/lib/frdmk64f.cw10gcc/debug/rtcs/rtcs.a
LIBRARIES += $(MQX_ROOTDIR)/lib/frdmk64f.cw10gcc/debug/shell/shell.a
LIBRARIES += $(MQX_ROOTDIR)/lib/frdmk64f.cw10gcc/debug/usb/usbh.a
endif
ifeq ($(CONFIG),release)
LIBRARIES += $(MQX_ROOTDIR)/lib/frdmk64f.cw10gcc/release/bsp/bsp.a
LIBRARIES += $(MQX_ROOTDIR)/lib/frdmk64f.cw10gcc/release/psp/psp.a
LIBRARIES += $(MQX_ROOTDIR)/lib/frdmk64f.cw10gcc/release/mfs/mfs.a
LIBRARIES += $(MQX_ROOTDIR)/lib/frdmk64f.cw10gcc/release/rtcs/rtcs.a
LIBRARIES += $(MQX_ROOTDIR)/lib/frdmk64f.cw10gcc/release/shell/shell.a
LIBRARIES += $(MQX_ROOTDIR)/lib/frdmk64f.cw10gcc/release/usb/usbh.a
endif


#-----------------------------------------------------------
# runtime libraries
#-----------------------------------------------------------
ifeq ($(CONFIG),debug)
RT_LIBRARIES += $(TOOLCHAIN_ROOTDIR)/MCU/ARM_GCC_Support/ewl/lib/armv7e-m/__arm_start.o
RT_LIBRARIES += $(TOOLCHAIN_ROOTDIR)/MCU/ARM_GCC_Support/ewl/lib/armv7e-m/__arm_end.o
RT_LIBRARIES += $(TOOLCHAIN_ROOTDIR)/MCU/ARM_GCC_Support/ewl/lib/armv7e-m/librt.a
RT_LIBRARIES += $(TOOLCHAIN_ROOTDIR)/MCU/ARM_GCC_Support/ewl/lib/armv7e-m/libc.a
RT_LIBRARIES += $(TOOLCHAIN_ROOTDIR)/MCU/ARM_GCC_Support/ewl/lib/armv7e-m/libm.a
RT_LIBRARIES += $(TOOLCHAIN_ROOTDIR)/MCU/ARM_GCC_Support/ewl/lib/armv7e-m/libc++.a
RT_LIBRARIES += $(TOOLCHAIN_ROOTDIR)/MCU/ARM_GCC_Support/ewl/lib/armv7e-m/libstdc++.a
RT_LIBRARIES += $(TOOLCHAIN_ROOTDIR)/Cross_Tools/arm-none-eabi-gcc-4_7_3/lib/gcc/arm-none-eabi/4.7.3/armv7e-m/libgcc.a
RT_LIBRARIES += $(TOOLCHAIN_ROOTDIR)/Cross_Tools/arm-none-eabi-gcc-4_7_3/arm-none-eabi/lib/armv7e-m/libsupc++.a
endif
ifeq ($(CONFIG),release)
RT_LIBRARIES += $(TOOLCHAIN_ROOTDIR)/MCU/ARM_GCC_Support/ewl/lib/armv7e-m/__arm_start.o
RT_LIBRARIES += $(TOOLCHAIN_ROOTDIR)/MCU/ARM_GCC_Support/ewl/lib/armv7e-m/__arm_end.o
RT_LIBRARIES += $(TOOLCHAIN_ROOTDIR)/MCU/ARM_GCC_Support/ewl/lib/armv7e-m/librt.a
RT_LIBRARIES += $(TOOLCHAIN_ROOTDIR)/MCU/ARM_GCC_Support/ewl/lib/armv7e-m/libc.a
RT_LIBRARIES += $(TOOLCHAIN_ROOTDIR)/MCU/ARM_GCC_Support/ewl/lib/armv7e-m/libm.a
RT_LIBRARIES += $(TOOLCHAIN_ROOTDIR)/MCU/ARM_GCC_Support/ewl/lib/armv7e-m/libc++.a
RT_LIBRARIES += $(TOOLCHAIN_ROOTDIR)/MCU/ARM_GCC_Support/ewl/lib/armv7e-m/libstdc++.a
RT_LIBRARIES += $(TOOLCHAIN_ROOTDIR)/Cross_Tools/arm-none-eabi-gcc-4_7_3/lib/gcc/arm-none-eabi/4.7.3/armv7e-m/libgcc.a
RT_LIBRARIES += $(TOOLCHAIN_ROOTDIR)/Cross_Tools/arm-none-eabi-gcc-4_7_3/arm-none-eabi/lib/armv7e-m/libsupc++.a
endif


#-----------------------------------------------------------
# runtime library paths
#-----------------------------------------------------------


#-----------------------------------------------------------
# search paths
#-----------------------------------------------------------
ifeq ($(CONFIG),debug)
INCLUDE += $(MQX_ROOTDIR)/lib/frdmk64f.cw10gcc/debug/bsp/Generated_Code
INCLUDE += $(MQX_ROOTDIR)/lib/frdmk64f.cw10gcc/debug/bsp/Sources
INCLUDE += $(MQX_ROOTDIR)/lib/frdmk64f.cw10gcc/debug
INCLUDE += $(MQX_ROOTDIR)/lib/frdmk64f.cw10gcc/debug/bsp
INCLUDE += $(MQX_ROOTDIR)/lib/frdmk64f.cw10gcc/debug/psp
INCLUDE += $(MQX_ROOTDIR)/lib/frdmk64f.cw10gcc/debug/mfs
INCLUDE += $(MQX_ROOTDIR)/lib/frdmk64f.cw10gcc/debug/rtcs
INCLUDE += $(MQX_ROOTDIR)/lib/frdmk64f.cw10gcc/debug/shell
INCLUDE += $(MQX_ROOTDIR)/lib/frdmk64f.cw10gcc/debug/usb
endif
ifeq ($(CONFIG),release)
INCLUDE += $(MQX_ROOTDIR)/lib/frdmk64f.cw10gcc/release/bsp/Generated_Code
INCLUDE += $(MQX_ROOTDIR)/lib/frdmk64f.cw10gcc/release/bsp/Sources
INCLUDE += $(MQX_ROOTDIR)/lib/frdmk64f.cw10gcc/release
INCLUDE += $(MQX_ROOTDIR)/lib/frdmk64f.cw10gcc/release/bsp
INCLUDE += $(MQX_ROOTDIR)/lib/frdmk64f.cw10gcc/release/psp
INCLUDE += $(MQX_ROOTDIR)/lib/frdmk64f.cw10gcc/release/mfs
INCLUDE += $(MQX_ROOTDIR)/lib/frdmk64f.cw10gcc/release/rtcs
INCLUDE += $(MQX_ROOTDIR)/lib/frdmk64f.cw10gcc/release/shell
INCLUDE += $(MQX_ROOTDIR)/lib/frdmk64f.cw10gcc/release/usb
endif


#-----------------------------------------------------------
# runtime search paths
#-----------------------------------------------------------
ifeq ($(CONFIG),debug)
RT_INCLUDE += $(TOOLCHAIN_ROOTDIR)/MCU/ARM_GCC_Support/ewl/EWL_C/include
RT_INCLUDE += $(TOOLCHAIN_ROOTDIR)/MCU/ARM_GCC_Support/ewl/EWL_Runtime/include
endif
ifeq ($(CONFIG),release)
RT_INCLUDE += $(TOOLCHAIN_ROOTDIR)/MCU/ARM_GCC_Support/ewl/EWL_C/include
RT_INCLUDE += $(TOOLCHAIN_ROOTDIR)/MCU/ARM_GCC_Support/ewl/EWL_Runtime/include
endif





