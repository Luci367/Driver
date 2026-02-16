/******************************************************************************
 *  File: Std_Types.h
 *  Module: Standard Types
 *
 *  Standard type definitions for embedded C.
 *
 *  Copyright (C) 2026
 *****************************************************************************/

#ifndef STD_TYPES_H
#define STD_TYPES_H

/******************************************************************************
 *  INCLUDES
 *****************************************************************************/

#include <stdint.h>
#include <stdbool.h>

/******************************************************************************
 *  TYPE DEFINITIONS
 *****************************************************************************/

/* Fixed-width integer types */
typedef uint8_t         uint8;
typedef uint16_t        uint16;
typedef uint32_t        uint32;
typedef uint64_t        uint64;
typedef int8_t          sint8;
typedef int16_t         sint16;
typedef int32_t         sint32;
typedef int64_t         sint64;

/* Boolean type */
typedef bool            boolean;

#ifndef TRUE
#define TRUE            ((boolean)1U)
#endif

#ifndef FALSE
#define FALSE           ((boolean)0U)
#endif

#ifndef NULL_PTR
#define NULL_PTR        ((void *)0)
#endif

/******************************************************************************
 *  STANDARD RETURN TYPE
 *****************************************************************************/

typedef uint8           Std_ReturnType;

#define E_OK            ((Std_ReturnType)0x00U)
#define E_NOT_OK        ((Std_ReturnType)0x01U)

/******************************************************************************
 *  VERSION INFO TYPE
 *****************************************************************************/

typedef struct
{
    uint16              vendorID;
    uint16              moduleID;
    uint8               sw_major_version;
    uint8               sw_minor_version;
    uint8               sw_patch_version;
} Std_VersionInfoType;

/******************************************************************************
 *  COMPILER ABSTRACTION MACROS
 *****************************************************************************/

/* Function declaration macros */
#define FUNC(rettype, memclass)             rettype

/* Variable declaration macros */
#define VAR(vartype, memclass)              vartype

/* Pointer to variable macros */
#define P2VAR(ptrtype, memclass, ptrclass)  ptrtype *

/* Pointer to constant macros */
#define P2CONST(ptrtype, memclass, ptrclass) const ptrtype *

/* Constant pointer to variable macros */
#define CONSTP2VAR(ptrtype, memclass, ptrclass) ptrtype * const

/* Constant pointer to constant macros */
#define CONSTP2CONST(ptrtype, memclass, ptrclass) const ptrtype * const

/* Pointer to function macros */
#define P2FUNC(rettype, ptrclass, fctname)  rettype (*fctname)

/* Constant macros */
#define CONST(consttype, memclass)          const consttype

/******************************************************************************
 *  MEMORY CLASS MACROS
 *****************************************************************************/

#define AUTOMATIC
#define TYPEDEF

/* Code section macros - CAN */
#define CAN_CODE
#define CAN_VAR
#define CAN_CONST
#define CAN_APPL_CODE
#define CAN_APPL_DATA
#define CAN_APPL_CONST

/* Code section macros - CAN XL */
#define CANXL_CODE
#define CANXL_VAR
#define CANXL_CONST
#define CANXL_APPL_CODE
#define CANXL_APPL_DATA
#define CANXL_APPL_CONST

/******************************************************************************
 *  ON/OFF MACROS
 *****************************************************************************/

#define STD_ON          (1U)
#define STD_OFF         (0U)

#define STD_HIGH        (1U)
#define STD_LOW         (0U)

#define STD_ACTIVE      (1U)
#define STD_IDLE        (0U)

#endif /* STD_TYPES_H */
