#*********************************************************************************************************
#
#                                 北京翼辉信息技术有限公司
#
#                                   微型安全实时操作系统
#
#                                       MS-RTOS(TM)
#
#                               Copyright All Rights Reserved
#
#--------------文件信息--------------------------------------------------------------------------------
#
# 文   件   名: lorawan.mk
#
# 创   建   人: IoT Studio
#
# 文件创建日期: 2020 年 11 月 16 日
#
# 描        述: 本文件由 IoT Studio 生成，用于配置 Makefile 功能，请勿手动修改
#*********************************************************************************************************

#*********************************************************************************************************
# Clear setting
#*********************************************************************************************************
include $(CLEAR_VARS_MK)

#*********************************************************************************************************
# Target
#*********************************************************************************************************
LOCAL_TARGET_NAME := liblorawan.a

#*********************************************************************************************************
# Source list
#*********************************************************************************************************
LOCAL_SRCS :=  \
src/LoRaMac-node/src/boards/mcu/utilities.c \
src/LoRaMac-node/src/mac/LoRaMac.c \
src/LoRaMac-node/src/mac/LoRaMacAdr.c \
src/LoRaMac-node/src/mac/LoRaMacClassB.c \
src/LoRaMac-node/src/mac/LoRaMacCommands.c \
src/LoRaMac-node/src/mac/LoRaMacConfirmQueue.c \
src/LoRaMac-node/src/mac/LoRaMacCrypto.c \
src/LoRaMac-node/src/mac/LoRaMacParser.c \
src/LoRaMac-node/src/mac/LoRaMacSerializer.c \
src/LoRaMac-node/src/mac/region/Region.c \
src/LoRaMac-node/src/mac/region/RegionAS923.c \
src/LoRaMac-node/src/mac/region/RegionAU915.c \
src/LoRaMac-node/src/mac/region/RegionBaseUS.c \
src/LoRaMac-node/src/mac/region/RegionCN470.c \
src/LoRaMac-node/src/mac/region/RegionCN779.c \
src/LoRaMac-node/src/mac/region/RegionCommon.c \
src/LoRaMac-node/src/mac/region/RegionEU433.c \
src/LoRaMac-node/src/mac/region/RegionEU868.c \
src/LoRaMac-node/src/mac/region/RegionIN865.c \
src/LoRaMac-node/src/mac/region/RegionKR920.c \
src/LoRaMac-node/src/mac/region/RegionRU864.c \
src/LoRaMac-node/src/mac/region/RegionUS915.c \
src/LoRaMac-node/src/peripherals/soft-se/aes.c \
src/LoRaMac-node/src/peripherals/soft-se/cmac.c \
src/LoRaMac-node/src/peripherals/soft-se/soft-se-hal.c \
src/LoRaMac-node/src/peripherals/soft-se/soft-se.c \
src/LoRaMac-node/src/radio/sx1276/sx1276.c \
src/LoRaMac-node/src/system/adc.c \
src/LoRaMac-node/src/system/delay.c \
src/LoRaMac-node/src/system/eeprom.c \
src/LoRaMac-node/src/system/fifo.c \
src/LoRaMac-node/src/system/gpio.c \
src/LoRaMac-node/src/system/gps.c \
src/LoRaMac-node/src/system/i2c.c \
src/LoRaMac-node/src/system/nvmm.c \
src/LoRaMac-node/src/system/systime.c \
src/LoRaMac-node/src/system/timer.c \
src/LoRaMac-node/src/system/uart.c

#*********************************************************************************************************
# Header file search path (eg. LOCAL_INC_PATH := -I"Your header files search path")
#*********************************************************************************************************
LOCAL_INC_PATH := \
-I"./src/LoRaMac-node/src/boards" \
-I"./src/LoRaMac-node/src/boards/mcu" \
-I"./src/LoRaMac-node/src/mac/" \
-I"./src/LoRaMac-node/src/mac/region" \
-I"./src/LoRaMac-node/src/peripherals" \
-I"./src/LoRaMac-node/src/peripherals/soft-se" \
-I"./src/LoRaMac-node/src/radio" \
-I"./src/LoRaMac-node/src/radio/sx1276" \
-I"./src/LoRaMac-node/src/system" \
-I"./src/LoRaMac-node/src"

#*********************************************************************************************************
# Pre-defined macro (eg. -DYOUR_MARCO=1)
#*********************************************************************************************************
LOCAL_DSYMBOL := 

#*********************************************************************************************************
# Compiler flags
#*********************************************************************************************************
LOCAL_CFLAGS   := 
LOCAL_CXXFLAGS := 

#*********************************************************************************************************
# Depend library (eg. LOCAL_DEPEND_LIB := -la LOCAL_DEPEND_LIB_PATH := -L"Your library search path")
#*********************************************************************************************************
LOCAL_DEPEND_LIB      := 
LOCAL_DEPEND_LIB_PATH := 

#*********************************************************************************************************
# C++ config
#*********************************************************************************************************
LOCAL_USE_CXX        := no
LOCAL_USE_CXX_EXCEPT := no

#*********************************************************************************************************
# Code coverage config
#*********************************************************************************************************
LOCAL_USE_GCOV := no

#*********************************************************************************************************
# Stack check config
#*********************************************************************************************************
LOCAL_USE_STACK_CHECK := no

#*********************************************************************************************************
# User link command
#*********************************************************************************************************
LOCAL_PRE_LINK_CMD   := 
LOCAL_POST_LINK_CMD  := 
LOCAL_PRE_STRIP_CMD  := 
LOCAL_POST_STRIP_CMD := 

#*********************************************************************************************************
# Depend target
#*********************************************************************************************************
LOCAL_DEPEND_TARGET := 

include $(STATIC_LIBRARY_MK)

#*********************************************************************************************************
# End
#*********************************************************************************************************
