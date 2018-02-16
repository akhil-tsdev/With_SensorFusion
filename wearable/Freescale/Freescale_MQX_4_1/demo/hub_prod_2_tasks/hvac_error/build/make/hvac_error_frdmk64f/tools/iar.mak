#-----------------------------------------------------------
# libraries
#-----------------------------------------------------------
ifeq ($(CONFIG),debug)
LIBRARIES += $(MQX_ROOTDIR)/lib/frdmk64f.iar/debug/bsp/bsp.a
LIBRARIES += $(MQX_ROOTDIR)/lib/frdmk64f.iar/debug/psp/psp.a
LIBRARIES += $(MQX_ROOTDIR)/lib/frdmk64f.iar/debug/mfs/mfs.a
LIBRARIES += $(MQX_ROOTDIR)/lib/frdmk64f.iar/debug/rtcs/rtcs.a
LIBRARIES += $(MQX_ROOTDIR)/lib/frdmk64f.iar/debug/shell/shell.a
LIBRARIES += $(MQX_ROOTDIR)/lib/frdmk64f.iar/debug/usb/usbh.a
endif
ifeq ($(CONFIG),release)
LIBRARIES += $(MQX_ROOTDIR)/lib/frdmk64f.iar/release/bsp/bsp.a
LIBRARIES += $(MQX_ROOTDIR)/lib/frdmk64f.iar/release/psp/psp.a
LIBRARIES += $(MQX_ROOTDIR)/lib/frdmk64f.iar/release/mfs/mfs.a
LIBRARIES += $(MQX_ROOTDIR)/lib/frdmk64f.iar/release/rtcs/rtcs.a
LIBRARIES += $(MQX_ROOTDIR)/lib/frdmk64f.iar/release/shell/shell.a
LIBRARIES += $(MQX_ROOTDIR)/lib/frdmk64f.iar/release/usb/usbh.a
endif


#-----------------------------------------------------------
# runtime libraries
#-----------------------------------------------------------


#-----------------------------------------------------------
# runtime library paths
#-----------------------------------------------------------


#-----------------------------------------------------------
# search paths
#-----------------------------------------------------------
ifeq ($(CONFIG),debug)
INCLUDE += $(MQX_ROOTDIR)/lib/frdmk64f.iar/debug/bsp/Generated_Code
INCLUDE += $(MQX_ROOTDIR)/lib/frdmk64f.iar/debug/bsp/Sources
INCLUDE += $(MQX_ROOTDIR)/lib/frdmk64f.iar/debug
INCLUDE += $(MQX_ROOTDIR)/lib/frdmk64f.iar/debug/bsp
INCLUDE += $(MQX_ROOTDIR)/lib/frdmk64f.iar/debug/psp
INCLUDE += $(MQX_ROOTDIR)/lib/frdmk64f.iar/debug/mfs
INCLUDE += $(MQX_ROOTDIR)/lib/frdmk64f.iar/debug/rtcs
INCLUDE += $(MQX_ROOTDIR)/lib/frdmk64f.iar/debug/shell
INCLUDE += $(MQX_ROOTDIR)/lib/frdmk64f.iar/debug/usb
endif
ifeq ($(CONFIG),release)
INCLUDE += $(MQX_ROOTDIR)/lib/frdmk64f.iar/release/bsp/Generated_Code
INCLUDE += $(MQX_ROOTDIR)/lib/frdmk64f.iar/release/bsp/Sources
INCLUDE += $(MQX_ROOTDIR)/lib/frdmk64f.iar/release
INCLUDE += $(MQX_ROOTDIR)/lib/frdmk64f.iar/release/bsp
INCLUDE += $(MQX_ROOTDIR)/lib/frdmk64f.iar/release/psp
INCLUDE += $(MQX_ROOTDIR)/lib/frdmk64f.iar/release/mfs
INCLUDE += $(MQX_ROOTDIR)/lib/frdmk64f.iar/release/rtcs
INCLUDE += $(MQX_ROOTDIR)/lib/frdmk64f.iar/release/shell
INCLUDE += $(MQX_ROOTDIR)/lib/frdmk64f.iar/release/usb
endif


#-----------------------------------------------------------
# runtime search paths
#-----------------------------------------------------------





