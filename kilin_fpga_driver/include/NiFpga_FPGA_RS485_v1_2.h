/*
 * Generated with the FPGA Interface C API Generator 19.0
 * for NI-RIO 19.0 or later.
 */
#ifndef __NiFpga_FPGA_RS485_v1_2_h__
#define __NiFpga_FPGA_RS485_v1_2_h__

#ifndef NiFpga_Version
   #define NiFpga_Version 190
#endif

#include "NiFpga.h"

/**
 * The filename of the FPGA bitfile.
 *
 * This is a #define to allow for string literal concatenation. For example:
 *
 *    static const char* const Bitfile = "C:\\" NiFpga_FPGA_RS485_v1_2_Bitfile;
 */
#define NiFpga_FPGA_RS485_v1_2_Bitfile "/home/admin/kilin_sbRIO_ws/kilin_fpga_driver/fpga_bitfile/NiFpga_FPGA_RS485_v1_2.lvbitx"

/**
 * The signature of the FPGA bitfile.
 */
static const char* const NiFpga_FPGA_RS485_v1_2_Signature = "F7868B107A2FC233CDE9829EEDA81707";

#if NiFpga_Cpp
extern "C"
{
#endif

typedef enum
{
   NiFpga_FPGA_RS485_v1_2_IndicatorBool_ChecksumOK = 0x1805E,
   NiFpga_FPGA_RS485_v1_2_IndicatorBool_Conn9_4r = 0x18016,
   NiFpga_FPGA_RS485_v1_2_IndicatorBool_Conn9_5r = 0x18012,
   NiFpga_FPGA_RS485_v1_2_IndicatorBool_Conn9_6r = 0x1800A,
   NiFpga_FPGA_RS485_v1_2_IndicatorBool_Mod1CAN0Complete = 0x18076,
   NiFpga_FPGA_RS485_v1_2_IndicatorBool_Mod1CAN0ID1RXsuccess = 0x18116,
   NiFpga_FPGA_RS485_v1_2_IndicatorBool_Mod1CAN0ID2RXsuccess = 0x1812E,
   NiFpga_FPGA_RS485_v1_2_IndicatorBool_Mod1CAN0RXTimeout = 0x18156,
   NiFpga_FPGA_RS485_v1_2_IndicatorBool_Mod1CAN0TXTimeout = 0x1815A,
   NiFpga_FPGA_RS485_v1_2_IndicatorBool_Mod1CAN0success = 0x1807A,
   NiFpga_FPGA_RS485_v1_2_IndicatorBool_Mod1CAN1Complete = 0x18086,
   NiFpga_FPGA_RS485_v1_2_IndicatorBool_Mod1CAN1ID1RXsuccess = 0x18172,
   NiFpga_FPGA_RS485_v1_2_IndicatorBool_Mod1CAN1ID2RXsuccess = 0x1818A,
   NiFpga_FPGA_RS485_v1_2_IndicatorBool_Mod1CAN1RXTimeout = 0x181B2,
   NiFpga_FPGA_RS485_v1_2_IndicatorBool_Mod1CAN1TXTimeout = 0x181B6,
   NiFpga_FPGA_RS485_v1_2_IndicatorBool_Mod1CAN1success = 0x1808A,
   NiFpga_FPGA_RS485_v1_2_IndicatorBool_RS485_CKS_OK1 = 0x180A6,
   NiFpga_FPGA_RS485_v1_2_IndicatorBool_RS485_CKS_OK2 = 0x180C2,
   NiFpga_FPGA_RS485_v1_2_IndicatorBool_RS485_CKS_OK3 = 0x180E2,
   NiFpga_FPGA_RS485_v1_2_IndicatorBool_RS485_CKS_OK4 = 0x18102,
   NiFpga_FPGA_RS485_v1_2_IndicatorBool_RS485_RX_finish1 = 0x180B2,
   NiFpga_FPGA_RS485_v1_2_IndicatorBool_RS485_RX_finish2 = 0x180D2,
   NiFpga_FPGA_RS485_v1_2_IndicatorBool_RS485_RX_finish3 = 0x180F2,
   NiFpga_FPGA_RS485_v1_2_IndicatorBool_RS485_RX_finish4 = 0x18112,
   NiFpga_FPGA_RS485_v1_2_IndicatorBool_RXfinish = 0x18062,
} NiFpga_FPGA_RS485_v1_2_IndicatorBool;

typedef enum
{
   NiFpga_FPGA_RS485_v1_2_IndicatorU8_Information = 0x18046,
} NiFpga_FPGA_RS485_v1_2_IndicatorU8;

typedef enum
{
   NiFpga_FPGA_RS485_v1_2_IndicatorI16_Mod1CAN0CompleteCounter = 0x1814E,
   NiFpga_FPGA_RS485_v1_2_IndicatorI16_Mod1CAN1CompleteCounter = 0x181AA,
} NiFpga_FPGA_RS485_v1_2_IndicatorI16;

typedef enum
{
   NiFpga_FPGA_RS485_v1_2_IndicatorI32_IRQ0_cnt = 0x18030,
   NiFpga_FPGA_RS485_v1_2_IndicatorI32_IRQ1_cnt = 0x18024,
   NiFpga_FPGA_RS485_v1_2_IndicatorI32_Mod1CAN0RXCounter = 0x18168,
   NiFpga_FPGA_RS485_v1_2_IndicatorI32_Mod1CAN0RXSTORCounter = 0x18160,
   NiFpga_FPGA_RS485_v1_2_IndicatorI32_Mod1CAN1RXCounter = 0x181C4,
   NiFpga_FPGA_RS485_v1_2_IndicatorI32_Mod1CAN1RXSTORCounter = 0x181BC,
   NiFpga_FPGA_RS485_v1_2_IndicatorI32_RS485_RX_count1 = 0x18094,
   NiFpga_FPGA_RS485_v1_2_IndicatorI32_RS485_RX_count2 = 0x180C8,
   NiFpga_FPGA_RS485_v1_2_IndicatorI32_RS485_RX_count3 = 0x180E8,
   NiFpga_FPGA_RS485_v1_2_IndicatorI32_RS485_RX_count4 = 0x18108,
   NiFpga_FPGA_RS485_v1_2_IndicatorI32_RS485_TX_count1 = 0x18098,
   NiFpga_FPGA_RS485_v1_2_IndicatorI32_RS485_TX_count2 = 0x180BC,
   NiFpga_FPGA_RS485_v1_2_IndicatorI32_RS485_TX_count3 = 0x180DC,
   NiFpga_FPGA_RS485_v1_2_IndicatorI32_RS485_TX_count4 = 0x180FC,
} NiFpga_FPGA_RS485_v1_2_IndicatorI32;

typedef enum
{
   NiFpga_FPGA_RS485_v1_2_IndicatorU32_Mod1CAN0RXTimeus = 0x18150,
   NiFpga_FPGA_RS485_v1_2_IndicatorU32_Mod1CAN1RXTimeus = 0x181AC,
} NiFpga_FPGA_RS485_v1_2_IndicatorU32;

typedef enum
{
   NiFpga_FPGA_RS485_v1_2_ControlBool_Conn9_1w = 0x1801E,
   NiFpga_FPGA_RS485_v1_2_ControlBool_Conn9_2w = 0x1801A,
   NiFpga_FPGA_RS485_v1_2_ControlBool_Conn9_3w = 0x1800E,
   NiFpga_FPGA_RS485_v1_2_ControlBool_Digital = 0x18056,
   NiFpga_FPGA_RS485_v1_2_ControlBool_IRQ0_wait_until_cleared = 0x1803A,
   NiFpga_FPGA_RS485_v1_2_ControlBool_IRQ1_wait_until_cleared = 0x1802E,
   NiFpga_FPGA_RS485_v1_2_ControlBool_MOD1CAN0Transmit = 0x1807E,
   NiFpga_FPGA_RS485_v1_2_ControlBool_MOD1CAN1Transmit = 0x1808E,
   NiFpga_FPGA_RS485_v1_2_ControlBool_Power = 0x1804E,
   NiFpga_FPGA_RS485_v1_2_ControlBool_RS485_Transmit1 = 0x1809E,
   NiFpga_FPGA_RS485_v1_2_ControlBool_RS485_Transmit2 = 0x180BA,
   NiFpga_FPGA_RS485_v1_2_ControlBool_RS485_Transmit3 = 0x180DA,
   NiFpga_FPGA_RS485_v1_2_ControlBool_RS485_Transmit4 = 0x180FA,
   NiFpga_FPGA_RS485_v1_2_ControlBool_Signal = 0x18052,
   NiFpga_FPGA_RS485_v1_2_ControlBool_stop = 0x18006,
} NiFpga_FPGA_RS485_v1_2_ControlBool;

typedef enum
{
   NiFpga_FPGA_RS485_v1_2_ControlU32_IOrwuSec = 0x18020,
   NiFpga_FPGA_RS485_v1_2_ControlU32_IRQ0_period_us = 0x18034,
   NiFpga_FPGA_RS485_v1_2_ControlU32_IRQ1_period_us = 0x18028,
   NiFpga_FPGA_RS485_v1_2_ControlU32_Mod1CAN0 = 0x1806C,
   NiFpga_FPGA_RS485_v1_2_ControlU32_Mod1CAN0ID1 = 0x18128,
   NiFpga_FPGA_RS485_v1_2_ControlU32_Mod1CAN0ID1FC = 0x18118,
   NiFpga_FPGA_RS485_v1_2_ControlU32_Mod1CAN0ID2 = 0x18140,
   NiFpga_FPGA_RS485_v1_2_ControlU32_Mod1CAN0ID2FC = 0x18130,
   NiFpga_FPGA_RS485_v1_2_ControlU32_Mod1CAN0RXTimeoutus = 0x18164,
   NiFpga_FPGA_RS485_v1_2_ControlU32_Mod1CAN1 = 0x18090,
   NiFpga_FPGA_RS485_v1_2_ControlU32_Mod1CAN1ID1 = 0x18184,
   NiFpga_FPGA_RS485_v1_2_ControlU32_Mod1CAN1ID1FC = 0x18174,
   NiFpga_FPGA_RS485_v1_2_ControlU32_Mod1CAN1ID2 = 0x1819C,
   NiFpga_FPGA_RS485_v1_2_ControlU32_Mod1CAN1ID2FC = 0x1818C,
   NiFpga_FPGA_RS485_v1_2_ControlU32_Mod1CAN1RXTimeoutus = 0x181C0,
   NiFpga_FPGA_RS485_v1_2_ControlU32_Module_Information = 0x1803C,
   NiFpga_FPGA_RS485_v1_2_ControlU32_PowerBoard = 0x18000,
   NiFpga_FPGA_RS485_v1_2_ControlU32_RS485Control = 0x18040,
} NiFpga_FPGA_RS485_v1_2_ControlU32;

typedef enum
{
   NiFpga_FPGA_RS485_v1_2_IndicatorArrayU8_DataArray = 0x18064,
   NiFpga_FPGA_RS485_v1_2_IndicatorArrayU8_DataRX = 0x18068,
   NiFpga_FPGA_RS485_v1_2_IndicatorArrayU8_Mod1CAN0ID1RX = 0x1811C,
   NiFpga_FPGA_RS485_v1_2_IndicatorArrayU8_Mod1CAN0ID2RX = 0x18134,
   NiFpga_FPGA_RS485_v1_2_IndicatorArrayU8_Mod1CAN1ID1RX = 0x18178,
   NiFpga_FPGA_RS485_v1_2_IndicatorArrayU8_Mod1CAN1ID2RX = 0x18190,
   NiFpga_FPGA_RS485_v1_2_IndicatorArrayU8_RS485_RX_Buf1 = 0x180AC,
   NiFpga_FPGA_RS485_v1_2_IndicatorArrayU8_RS485_RX_Buf2 = 0x180CC,
   NiFpga_FPGA_RS485_v1_2_IndicatorArrayU8_RS485_RX_Buf3 = 0x180EC,
   NiFpga_FPGA_RS485_v1_2_IndicatorArrayU8_RS485_RX_Buf4 = 0x1810C,
   NiFpga_FPGA_RS485_v1_2_IndicatorArrayU8_RS485_RX_Data1 = 0x180A8,
   NiFpga_FPGA_RS485_v1_2_IndicatorArrayU8_RS485_RX_Data2 = 0x180C4,
   NiFpga_FPGA_RS485_v1_2_IndicatorArrayU8_RS485_RX_Data3 = 0x180E4,
   NiFpga_FPGA_RS485_v1_2_IndicatorArrayU8_RS485_RX_Data4 = 0x18104,
} NiFpga_FPGA_RS485_v1_2_IndicatorArrayU8;

typedef enum
{
   NiFpga_FPGA_RS485_v1_2_IndicatorArrayU8Size_DataArray = 49,
   NiFpga_FPGA_RS485_v1_2_IndicatorArrayU8Size_DataRX = 64,
   NiFpga_FPGA_RS485_v1_2_IndicatorArrayU8Size_Mod1CAN0ID1RX = 8,
   NiFpga_FPGA_RS485_v1_2_IndicatorArrayU8Size_Mod1CAN0ID2RX = 8,
   NiFpga_FPGA_RS485_v1_2_IndicatorArrayU8Size_Mod1CAN1ID1RX = 8,
   NiFpga_FPGA_RS485_v1_2_IndicatorArrayU8Size_Mod1CAN1ID2RX = 8,
   NiFpga_FPGA_RS485_v1_2_IndicatorArrayU8Size_RS485_RX_Buf1 = 32,
   NiFpga_FPGA_RS485_v1_2_IndicatorArrayU8Size_RS485_RX_Buf2 = 32,
   NiFpga_FPGA_RS485_v1_2_IndicatorArrayU8Size_RS485_RX_Buf3 = 32,
   NiFpga_FPGA_RS485_v1_2_IndicatorArrayU8Size_RS485_RX_Buf4 = 32,
   NiFpga_FPGA_RS485_v1_2_IndicatorArrayU8Size_RS485_RX_Data1 = 20,
   NiFpga_FPGA_RS485_v1_2_IndicatorArrayU8Size_RS485_RX_Data2 = 20,
   NiFpga_FPGA_RS485_v1_2_IndicatorArrayU8Size_RS485_RX_Data3 = 20,
   NiFpga_FPGA_RS485_v1_2_IndicatorArrayU8Size_RS485_RX_Data4 = 20,
} NiFpga_FPGA_RS485_v1_2_IndicatorArrayU8Size;

typedef enum
{
   NiFpga_FPGA_RS485_v1_2_IndicatorArrayU16_Data = 0x18048,
} NiFpga_FPGA_RS485_v1_2_IndicatorArrayU16;

typedef enum
{
   NiFpga_FPGA_RS485_v1_2_IndicatorArrayU16Size_Data = 24,
} NiFpga_FPGA_RS485_v1_2_IndicatorArrayU16Size;

typedef enum
{
   NiFpga_FPGA_RS485_v1_2_ControlArrayBool_Mod1CAN0Select = 0x18072,
   NiFpga_FPGA_RS485_v1_2_ControlArrayBool_Mod1CAN1Select = 0x18082,
} NiFpga_FPGA_RS485_v1_2_ControlArrayBool;

typedef enum
{
   NiFpga_FPGA_RS485_v1_2_ControlArrayBoolSize_Mod1CAN0Select = 2,
   NiFpga_FPGA_RS485_v1_2_ControlArrayBoolSize_Mod1CAN1Select = 2,
} NiFpga_FPGA_RS485_v1_2_ControlArrayBoolSize;

typedef enum
{
   NiFpga_FPGA_RS485_v1_2_ControlArrayU8_DataTx = 0x18058,
   NiFpga_FPGA_RS485_v1_2_ControlArrayU8_Mod1CAN0ID1TX = 0x18124,
   NiFpga_FPGA_RS485_v1_2_ControlArrayU8_Mod1CAN0ID2TX = 0x1813C,
   NiFpga_FPGA_RS485_v1_2_ControlArrayU8_Mod1CAN1ID1TX = 0x18180,
   NiFpga_FPGA_RS485_v1_2_ControlArrayU8_Mod1CAN1ID2TX = 0x18198,
   NiFpga_FPGA_RS485_v1_2_ControlArrayU8_RS485_TX_Data1 = 0x180A0,
   NiFpga_FPGA_RS485_v1_2_ControlArrayU8_RS485_TX_Data2 = 0x180B4,
   NiFpga_FPGA_RS485_v1_2_ControlArrayU8_RS485_TX_Data3 = 0x180D4,
   NiFpga_FPGA_RS485_v1_2_ControlArrayU8_RS485_TX_Data4 = 0x180F4,
} NiFpga_FPGA_RS485_v1_2_ControlArrayU8;

typedef enum
{
   NiFpga_FPGA_RS485_v1_2_ControlArrayU8Size_DataTx = 12,
   NiFpga_FPGA_RS485_v1_2_ControlArrayU8Size_Mod1CAN0ID1TX = 8,
   NiFpga_FPGA_RS485_v1_2_ControlArrayU8Size_Mod1CAN0ID2TX = 8,
   NiFpga_FPGA_RS485_v1_2_ControlArrayU8Size_Mod1CAN1ID1TX = 8,
   NiFpga_FPGA_RS485_v1_2_ControlArrayU8Size_Mod1CAN1ID2TX = 8,
   NiFpga_FPGA_RS485_v1_2_ControlArrayU8Size_RS485_TX_Data1 = 12,
   NiFpga_FPGA_RS485_v1_2_ControlArrayU8Size_RS485_TX_Data2 = 12,
   NiFpga_FPGA_RS485_v1_2_ControlArrayU8Size_RS485_TX_Data3 = 12,
   NiFpga_FPGA_RS485_v1_2_ControlArrayU8Size_RS485_TX_Data4 = 12,
} NiFpga_FPGA_RS485_v1_2_ControlArrayU8Size;

#if !NiFpga_VxWorks

/* Indicator: Mod1CAN0RXBuffer */

/* Use NiFpga_ReadArrayU8() to access Mod1CAN0RXBuffer */
const uint32_t NiFpga_FPGA_RS485_v1_2_IndicatorClusterArray_Mod1CAN0RXBuffer_Resource = 0x1815C;
const uint32_t NiFpga_FPGA_RS485_v1_2_IndicatorClusterArray_Mod1CAN0RXBuffer_Size = 8;
const uint32_t NiFpga_FPGA_RS485_v1_2_IndicatorClusterArray_Mod1CAN0RXBuffer_PackedSizeInBytes = 192;

typedef struct NiFpga_FPGA_RS485_v1_2_IndicatorClusterArray_Mod1CAN0RXBuffer_Type{
   uint32_t timestamphigh;
   uint32_t timestamplow;
   uint32_t identifier;
   uint8_t type;
   uint8_t infoA;
   uint8_t infoB;
   uint8_t datalength;
   uint8_t data[8];
}NiFpga_FPGA_RS485_v1_2_IndicatorClusterArray_Mod1CAN0RXBuffer_Type;

void NiFpga_FPGA_RS485_v1_2_IndicatorClusterArray_Mod1CAN0RXBuffer_UnpackArray(
   const uint8_t* const packedData,
   NiFpga_FPGA_RS485_v1_2_IndicatorClusterArray_Mod1CAN0RXBuffer_Type* const destination);

void NiFpga_FPGA_RS485_v1_2_IndicatorClusterArray_Mod1CAN0RXBuffer_PackArray(
   uint8_t* const packedData,
   const NiFpga_FPGA_RS485_v1_2_IndicatorClusterArray_Mod1CAN0RXBuffer_Type* const source);

/* Indicator: Mod1CAN1RXBuffer */

/* Use NiFpga_ReadArrayU8() to access Mod1CAN1RXBuffer */
const uint32_t NiFpga_FPGA_RS485_v1_2_IndicatorClusterArray_Mod1CAN1RXBuffer_Resource = 0x181B8;
const uint32_t NiFpga_FPGA_RS485_v1_2_IndicatorClusterArray_Mod1CAN1RXBuffer_Size = 8;
const uint32_t NiFpga_FPGA_RS485_v1_2_IndicatorClusterArray_Mod1CAN1RXBuffer_PackedSizeInBytes = 192;

typedef struct NiFpga_FPGA_RS485_v1_2_IndicatorClusterArray_Mod1CAN1RXBuffer_Type{
   uint32_t timestamphigh;
   uint32_t timestamplow;
   uint32_t identifier;
   uint8_t type;
   uint8_t infoA;
   uint8_t infoB;
   uint8_t datalength;
   uint8_t data[8];
}NiFpga_FPGA_RS485_v1_2_IndicatorClusterArray_Mod1CAN1RXBuffer_Type;

void NiFpga_FPGA_RS485_v1_2_IndicatorClusterArray_Mod1CAN1RXBuffer_UnpackArray(
   const uint8_t* const packedData,
   NiFpga_FPGA_RS485_v1_2_IndicatorClusterArray_Mod1CAN1RXBuffer_Type* const destination);

void NiFpga_FPGA_RS485_v1_2_IndicatorClusterArray_Mod1CAN1RXBuffer_PackArray(
   uint8_t* const packedData,
   const NiFpga_FPGA_RS485_v1_2_IndicatorClusterArray_Mod1CAN1RXBuffer_Type* const source);

/* Indicator: Mod1CAN0ID1RXFrame */
/* Use NiFpga_ReadArrayU8() to access Mod1CAN0ID1RXFrame */
const uint32_t NiFpga_FPGA_RS485_v1_2_IndicatorCluster_Mod1CAN0ID1RXFrame_Resource = 0x18120;
const uint32_t NiFpga_FPGA_RS485_v1_2_IndicatorCluster_Mod1CAN0ID1RXFrame_PackedSizeInBytes = 24;

typedef struct NiFpga_FPGA_RS485_v1_2_IndicatorCluster_Mod1CAN0ID1RXFrame_Type{
   uint32_t timestamphigh;
   uint32_t timestamplow;
   uint32_t identifier;
   uint8_t type;
   uint8_t infoA;
   uint8_t infoB;
   uint8_t datalength;
   uint8_t data[8];
}NiFpga_FPGA_RS485_v1_2_IndicatorCluster_Mod1CAN0ID1RXFrame_Type;


void NiFpga_FPGA_RS485_v1_2_IndicatorCluster_Mod1CAN0ID1RXFrame_UnpackCluster(
   const uint8_t* const packedData,
   NiFpga_FPGA_RS485_v1_2_IndicatorCluster_Mod1CAN0ID1RXFrame_Type* const destination);

void NiFpga_FPGA_RS485_v1_2_IndicatorCluster_Mod1CAN0ID1RXFrame_PackCluster(
   uint8_t* const packedData,
   const NiFpga_FPGA_RS485_v1_2_IndicatorCluster_Mod1CAN0ID1RXFrame_Type* const source);

/* Indicator: Mod1CAN0ID2RXFrame */
/* Use NiFpga_ReadArrayU8() to access Mod1CAN0ID2RXFrame */
const uint32_t NiFpga_FPGA_RS485_v1_2_IndicatorCluster_Mod1CAN0ID2RXFrame_Resource = 0x18138;
const uint32_t NiFpga_FPGA_RS485_v1_2_IndicatorCluster_Mod1CAN0ID2RXFrame_PackedSizeInBytes = 24;

typedef struct NiFpga_FPGA_RS485_v1_2_IndicatorCluster_Mod1CAN0ID2RXFrame_Type{
   uint32_t timestamphigh;
   uint32_t timestamplow;
   uint32_t identifier;
   uint8_t type;
   uint8_t infoA;
   uint8_t infoB;
   uint8_t datalength;
   uint8_t data[8];
}NiFpga_FPGA_RS485_v1_2_IndicatorCluster_Mod1CAN0ID2RXFrame_Type;


void NiFpga_FPGA_RS485_v1_2_IndicatorCluster_Mod1CAN0ID2RXFrame_UnpackCluster(
   const uint8_t* const packedData,
   NiFpga_FPGA_RS485_v1_2_IndicatorCluster_Mod1CAN0ID2RXFrame_Type* const destination);

void NiFpga_FPGA_RS485_v1_2_IndicatorCluster_Mod1CAN0ID2RXFrame_PackCluster(
   uint8_t* const packedData,
   const NiFpga_FPGA_RS485_v1_2_IndicatorCluster_Mod1CAN0ID2RXFrame_Type* const source);

/* Indicator: Mod1CAN0RXError */
/* Use NiFpga_ReadArrayU8() to access Mod1CAN0RXError */
const uint32_t NiFpga_FPGA_RS485_v1_2_IndicatorCluster_Mod1CAN0RXError_Resource = 0x18148;
const uint32_t NiFpga_FPGA_RS485_v1_2_IndicatorCluster_Mod1CAN0RXError_PackedSizeInBytes = 5;

typedef struct NiFpga_FPGA_RS485_v1_2_IndicatorCluster_Mod1CAN0RXError_Type{
   NiFpga_Bool status;
   int32_t code;
}NiFpga_FPGA_RS485_v1_2_IndicatorCluster_Mod1CAN0RXError_Type;


void NiFpga_FPGA_RS485_v1_2_IndicatorCluster_Mod1CAN0RXError_UnpackCluster(
   const uint8_t* const packedData,
   NiFpga_FPGA_RS485_v1_2_IndicatorCluster_Mod1CAN0RXError_Type* const destination);

void NiFpga_FPGA_RS485_v1_2_IndicatorCluster_Mod1CAN0RXError_PackCluster(
   uint8_t* const packedData,
   const NiFpga_FPGA_RS485_v1_2_IndicatorCluster_Mod1CAN0RXError_Type* const source);

/* Indicator: Mod1CAN0RXFrame */
/* Use NiFpga_ReadArrayU8() to access Mod1CAN0RXFrame */
const uint32_t NiFpga_FPGA_RS485_v1_2_IndicatorCluster_Mod1CAN0RXFrame_Resource = 0x1816C;
const uint32_t NiFpga_FPGA_RS485_v1_2_IndicatorCluster_Mod1CAN0RXFrame_PackedSizeInBytes = 24;

typedef struct NiFpga_FPGA_RS485_v1_2_IndicatorCluster_Mod1CAN0RXFrame_Type{
   uint32_t timestamphigh;
   uint32_t timestamplow;
   uint32_t identifier;
   uint8_t type;
   uint8_t infoA;
   uint8_t infoB;
   uint8_t datalength;
   uint8_t data[8];
}NiFpga_FPGA_RS485_v1_2_IndicatorCluster_Mod1CAN0RXFrame_Type;


void NiFpga_FPGA_RS485_v1_2_IndicatorCluster_Mod1CAN0RXFrame_UnpackCluster(
   const uint8_t* const packedData,
   NiFpga_FPGA_RS485_v1_2_IndicatorCluster_Mod1CAN0RXFrame_Type* const destination);

void NiFpga_FPGA_RS485_v1_2_IndicatorCluster_Mod1CAN0RXFrame_PackCluster(
   uint8_t* const packedData,
   const NiFpga_FPGA_RS485_v1_2_IndicatorCluster_Mod1CAN0RXFrame_Type* const source);

/* Indicator: Mod1CAN0TXError */
/* Use NiFpga_ReadArrayU8() to access Mod1CAN0TXError */
const uint32_t NiFpga_FPGA_RS485_v1_2_IndicatorCluster_Mod1CAN0TXError_Resource = 0x18144;
const uint32_t NiFpga_FPGA_RS485_v1_2_IndicatorCluster_Mod1CAN0TXError_PackedSizeInBytes = 5;

typedef struct NiFpga_FPGA_RS485_v1_2_IndicatorCluster_Mod1CAN0TXError_Type{
   NiFpga_Bool status;
   int32_t code;
}NiFpga_FPGA_RS485_v1_2_IndicatorCluster_Mod1CAN0TXError_Type;


void NiFpga_FPGA_RS485_v1_2_IndicatorCluster_Mod1CAN0TXError_UnpackCluster(
   const uint8_t* const packedData,
   NiFpga_FPGA_RS485_v1_2_IndicatorCluster_Mod1CAN0TXError_Type* const destination);

void NiFpga_FPGA_RS485_v1_2_IndicatorCluster_Mod1CAN0TXError_PackCluster(
   uint8_t* const packedData,
   const NiFpga_FPGA_RS485_v1_2_IndicatorCluster_Mod1CAN0TXError_Type* const source);

/* Indicator: Mod1CAN1ID1RXFrame */
/* Use NiFpga_ReadArrayU8() to access Mod1CAN1ID1RXFrame */
const uint32_t NiFpga_FPGA_RS485_v1_2_IndicatorCluster_Mod1CAN1ID1RXFrame_Resource = 0x1817C;
const uint32_t NiFpga_FPGA_RS485_v1_2_IndicatorCluster_Mod1CAN1ID1RXFrame_PackedSizeInBytes = 24;

typedef struct NiFpga_FPGA_RS485_v1_2_IndicatorCluster_Mod1CAN1ID1RXFrame_Type{
   uint32_t timestamphigh;
   uint32_t timestamplow;
   uint32_t identifier;
   uint8_t type;
   uint8_t infoA;
   uint8_t infoB;
   uint8_t datalength;
   uint8_t data[8];
}NiFpga_FPGA_RS485_v1_2_IndicatorCluster_Mod1CAN1ID1RXFrame_Type;


void NiFpga_FPGA_RS485_v1_2_IndicatorCluster_Mod1CAN1ID1RXFrame_UnpackCluster(
   const uint8_t* const packedData,
   NiFpga_FPGA_RS485_v1_2_IndicatorCluster_Mod1CAN1ID1RXFrame_Type* const destination);

void NiFpga_FPGA_RS485_v1_2_IndicatorCluster_Mod1CAN1ID1RXFrame_PackCluster(
   uint8_t* const packedData,
   const NiFpga_FPGA_RS485_v1_2_IndicatorCluster_Mod1CAN1ID1RXFrame_Type* const source);

/* Indicator: Mod1CAN1ID2RXFrame */
/* Use NiFpga_ReadArrayU8() to access Mod1CAN1ID2RXFrame */
const uint32_t NiFpga_FPGA_RS485_v1_2_IndicatorCluster_Mod1CAN1ID2RXFrame_Resource = 0x18194;
const uint32_t NiFpga_FPGA_RS485_v1_2_IndicatorCluster_Mod1CAN1ID2RXFrame_PackedSizeInBytes = 24;

typedef struct NiFpga_FPGA_RS485_v1_2_IndicatorCluster_Mod1CAN1ID2RXFrame_Type{
   uint32_t timestamphigh;
   uint32_t timestamplow;
   uint32_t identifier;
   uint8_t type;
   uint8_t infoA;
   uint8_t infoB;
   uint8_t datalength;
   uint8_t data[8];
}NiFpga_FPGA_RS485_v1_2_IndicatorCluster_Mod1CAN1ID2RXFrame_Type;


void NiFpga_FPGA_RS485_v1_2_IndicatorCluster_Mod1CAN1ID2RXFrame_UnpackCluster(
   const uint8_t* const packedData,
   NiFpga_FPGA_RS485_v1_2_IndicatorCluster_Mod1CAN1ID2RXFrame_Type* const destination);

void NiFpga_FPGA_RS485_v1_2_IndicatorCluster_Mod1CAN1ID2RXFrame_PackCluster(
   uint8_t* const packedData,
   const NiFpga_FPGA_RS485_v1_2_IndicatorCluster_Mod1CAN1ID2RXFrame_Type* const source);

/* Indicator: Mod1CAN1RXError */
/* Use NiFpga_ReadArrayU8() to access Mod1CAN1RXError */
const uint32_t NiFpga_FPGA_RS485_v1_2_IndicatorCluster_Mod1CAN1RXError_Resource = 0x181A4;
const uint32_t NiFpga_FPGA_RS485_v1_2_IndicatorCluster_Mod1CAN1RXError_PackedSizeInBytes = 5;

typedef struct NiFpga_FPGA_RS485_v1_2_IndicatorCluster_Mod1CAN1RXError_Type{
   NiFpga_Bool status;
   int32_t code;
}NiFpga_FPGA_RS485_v1_2_IndicatorCluster_Mod1CAN1RXError_Type;


void NiFpga_FPGA_RS485_v1_2_IndicatorCluster_Mod1CAN1RXError_UnpackCluster(
   const uint8_t* const packedData,
   NiFpga_FPGA_RS485_v1_2_IndicatorCluster_Mod1CAN1RXError_Type* const destination);

void NiFpga_FPGA_RS485_v1_2_IndicatorCluster_Mod1CAN1RXError_PackCluster(
   uint8_t* const packedData,
   const NiFpga_FPGA_RS485_v1_2_IndicatorCluster_Mod1CAN1RXError_Type* const source);

/* Indicator: Mod1CAN1RXFrame */
/* Use NiFpga_ReadArrayU8() to access Mod1CAN1RXFrame */
const uint32_t NiFpga_FPGA_RS485_v1_2_IndicatorCluster_Mod1CAN1RXFrame_Resource = 0x181C8;
const uint32_t NiFpga_FPGA_RS485_v1_2_IndicatorCluster_Mod1CAN1RXFrame_PackedSizeInBytes = 24;

typedef struct NiFpga_FPGA_RS485_v1_2_IndicatorCluster_Mod1CAN1RXFrame_Type{
   uint32_t timestamphigh;
   uint32_t timestamplow;
   uint32_t identifier;
   uint8_t type;
   uint8_t infoA;
   uint8_t infoB;
   uint8_t datalength;
   uint8_t data[8];
}NiFpga_FPGA_RS485_v1_2_IndicatorCluster_Mod1CAN1RXFrame_Type;


void NiFpga_FPGA_RS485_v1_2_IndicatorCluster_Mod1CAN1RXFrame_UnpackCluster(
   const uint8_t* const packedData,
   NiFpga_FPGA_RS485_v1_2_IndicatorCluster_Mod1CAN1RXFrame_Type* const destination);

void NiFpga_FPGA_RS485_v1_2_IndicatorCluster_Mod1CAN1RXFrame_PackCluster(
   uint8_t* const packedData,
   const NiFpga_FPGA_RS485_v1_2_IndicatorCluster_Mod1CAN1RXFrame_Type* const source);

/* Indicator: Mod1CAN1TXError */
/* Use NiFpga_ReadArrayU8() to access Mod1CAN1TXError */
const uint32_t NiFpga_FPGA_RS485_v1_2_IndicatorCluster_Mod1CAN1TXError_Resource = 0x181A0;
const uint32_t NiFpga_FPGA_RS485_v1_2_IndicatorCluster_Mod1CAN1TXError_PackedSizeInBytes = 5;

typedef struct NiFpga_FPGA_RS485_v1_2_IndicatorCluster_Mod1CAN1TXError_Type{
   NiFpga_Bool status;
   int32_t code;
}NiFpga_FPGA_RS485_v1_2_IndicatorCluster_Mod1CAN1TXError_Type;


void NiFpga_FPGA_RS485_v1_2_IndicatorCluster_Mod1CAN1TXError_UnpackCluster(
   const uint8_t* const packedData,
   NiFpga_FPGA_RS485_v1_2_IndicatorCluster_Mod1CAN1TXError_Type* const destination);

void NiFpga_FPGA_RS485_v1_2_IndicatorCluster_Mod1CAN1TXError_PackCluster(
   uint8_t* const packedData,
   const NiFpga_FPGA_RS485_v1_2_IndicatorCluster_Mod1CAN1TXError_Type* const source);

#endif /* !NiFpga_VxWorks */


#if NiFpga_Cpp
}
#endif

#endif
