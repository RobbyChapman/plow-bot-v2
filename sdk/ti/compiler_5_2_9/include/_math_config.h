/* _math_config.h internal header */
#ifndef _MATH_CONFIG
#define _MATH_CONFIG
#include <errno.h>
#include <ymath.h>

#pragma diag_push
#pragma CHECK_MISRA("-19.7") /* macros required for implementation */
/* keep macros as direct #defines and not function-like macros or function
   names surrounded by parentheses to support all original supported use cases
   including taking their address through the macros and prefixing with
   namespace macros */
#pragma CHECK_MISRA("-19.4")

_C_STD_BEGIN
		/* FLOAT PROPERTIES */
#ifndef _D0

 /* Undefined _D0 means "default": little-endian */

 #define _D0	3	/* little-endian 64-bit */
 #define _D1	2
 #define _D2	1
 #define _D3	0

 #define _DBIAS	0x3fe
 #define _DOFF		4

 #define _FBIAS	0x7e
 #define _FOFF	7
 #define _FRND	1

 #define _DLONG	0       /* 0 means long double is 64 bits */
 #define _LBIAS	0x3fe
 #define _LOFF	4

#elif _D0 == 0		/* other params defined in <yvals.h> */
 #define _D1	1	/* big-endian 64-bit */
 #define _D2	2
 #define _D3	3
#elif _D0 == 1
 #define _D1    0       /* TMS470 special FPALIB endianness */
 #define _D2    3
 #define _D3    2
#else /* _D0 */
 #define _D1	2	/* little-endian 64-bit */
 #define _D2	1
 #define _D3	0
#endif /* _D0 */

		/* IEEE 754 64-bit properties */
#define _DFRAC	((unsigned short)((1 << _DOFF) - 1))
#define _DMASK	((unsigned short)(0x7fff & ~_DFRAC))
#define _DMAX	((unsigned short)((1 << (15 - _DOFF)) - 1))
#define _DSIGN	((unsigned short)0x8000)

                /* C "double" properties */
#if defined(_32_BIT_DOUBLE)
#define DSIGN(x)	FSIGN(x)
#define HUGE_EXP	FHUGE_EXP
#define HUGE_RAD	FHUGE_RAD
#define SAFE_EXP	FSAFE_EXP
#else
#define DSIGN(x)	(((unsigned short *)(char *)&(x))[_D0] & _DSIGN)
#define HUGE_EXP	(int)(_DMAX * 900L / 1000)
#define HUGE_RAD	2.73e9	/* ~ 2^33 / pi */
#define SAFE_EXP	((short)(_DMAX >> 1))
#endif

		/* IEEE 754 32-bit properties */
#define _FFRAC	((unsigned short)((1 << _FOFF) - 1))
#define _FMASK	((unsigned short)(0x7fff & ~_FFRAC))
#define _FMAX	((unsigned short)((1 << (15 - _FOFF)) - 1))
#define _FSIGN	((unsigned short)0x8000)
                /* C "float" properties */
#define FSIGN(x)	(((unsigned short *)(char *)&(x))[_F0] & _FSIGN)
#define FHUGE_EXP	(int)(_FMAX * 900L / 1000)
#define FHUGE_RAD	40.7	/* ~ 2^7 / pi */
#define FSAFE_EXP	((short)(_FMAX >> 1))

 #if _D0 == 0
  #define _F0	0	/* big-endian */
  #define _F1	1

 #else /* _D0 == 0 */
  #define _F0	1	/* little-endian */
  #define _F1	0
 #endif /* _D0 == 0 */

		/* IEEE 754 80- and 128-bit properties */
#define _LFRAC	((unsigned short)(-1))
#define _LMASK	((unsigned short)0x7fff)
#define _LMAX	((unsigned short)0x7fff)
#define _LSIGN	((unsigned short)0x8000)

                /* C "long double" properties */
#if defined(_32_BIT_LDOUBLE)
#define LSIGN(x)	FSIGN(x)
#define LHUGE_EXP	FHUGE_EXP
#define LHUGE_RAD	FHUGE_RAD
#define LSAFE_EXP	FSAFE_EXP
#elif _DLONG == 0
#define LSIGN(x)	(((unsigned short *)(char *)&(x))[_D0] & _DSIGN)
#define LHUGE_EXP	(int)(_DMAX * 900L / 1000)
#define LHUGE_RAD	2.73e9	/* ~ 2^33 / pi */
#define LSAFE_EXP	((short)(_DMAX >> 1))
#else
#define LSIGN(x)	(((unsigned short *)(char *)&(x))[_L0] & _LSIGN)
#define LHUGE_EXP	(int)(_LMAX * 900L / 1000)
#define LHUGE_RAD	2.73e9	/* ~ 2^33 / pi */
#define LSAFE_EXP	((short)(_LMAX >> 1))
#endif

 #if _D0 == 0
  #define _L0	0	/* big-endian */
  #define _L1	1
  #define _L2	2
  #define _L3	3
  #define _L4	4
  #define _L5	5	/* 128-bit only */
  #define _L6	6
  #define _L7	7

 #elif _DLONG == 0
  #define _L0	0       /* little-endian, 64-bit */
  #define _L1	1
  #define _L2	2
  #define _L3	3    
  #define _L4	xxx	/* should never be used */
  #define _L5	xxx
  #define _L6	xxx
  #define _L7	xxx

 #elif _DLONG == 1
  #define _L0	4	/* little-endian, 80-bit */
  #define _L1	3
  #define _L2	2
  #define _L3	1
  #define _L4	0
  #define _L5	xxx	/* should never be used */
  #define _L6	xxx
  #define _L7	xxx

 #else /* _DLONG */
  #define _L0	7	/* little-endian, 128-bit */
  #define _L1	6
  #define _L2	5
  #define _L3	4
  #define _L4	3
  #define _L5	2
  #define _L6	1
  #define _L7	0
 #endif /* _DLONG */

		/* return values for _Stopfx/_Stoflt */
#define FL_ERR	0
#define FL_DEC	1
#define FL_HEX	2
#define FL_INF	3
#define FL_NAN	4
#define FL_NEG	8

_C_STD_END
#endif /* _MATH_CONFIG */

#pragma diag_pop

/*
 * Copyright (c) 1992-2004 by P.J. Plauger.  ALL RIGHTS RESERVED.
 * Consult your license regarding permissions and restrictions.
V4.02:1476 */
