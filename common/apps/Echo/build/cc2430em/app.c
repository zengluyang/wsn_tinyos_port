
#define MANGLED_NESC_APP_C
#define nx_struct struct
#define nx_union union
#define dbg(mode, format) ((void)0)
#define dbg_clear(mode, format) ((void)0)
#define dbg_active(mode) 0
//# 151 "/usr/lib/gcc/i686-pc-cygwin/3.4.4/include/stddef.h" 3
typedef int ptrdiff_t;
//#line 213
typedef unsigned int size_t;
//#line 325
typedef short unsigned int wchar_t;
//# 8 "/usr/lib/ncc/deputy_nodeputy.h"
/*struct __nesc_attr_nonnull {
}  ;*/
//#line 9
/*struct __nesc_attr_bnd {
//#line 9
  void *lo, *hi;
}  ;*/
//#line 10
/*struct __nesc_attr_bnd_nok {
//#line 10
  void *lo, *hi;
}  ;*/
//#line 11
/*struct __nesc_attr_count {
//#line 11
  int n;
}  ;*/
//#line 12
/*struct __nesc_attr_count_nok {
//#line 12
  int n;
}  ;*/
//#line 13
/*struct __nesc_attr_one {
}  ;*/
//#line 14
/*struct __nesc_attr_one_nok {
}  ;*/
//#line 15
/*struct __nesc_attr_dmemset {
//#line 15
  int a1, a2, a3;
}  ;*/
//#line 16
/*struct __nesc_attr_dmemcpy {
//#line 16
  int a1, a2, a3;
}  ;*/
//#line 17
/*struct __nesc_attr_nts {
}  ;*/
//# 47 "/opt/tinyos-2.x-contrib/diku/mcs51/tos/chips/mcs51/keil_stdint.h" 3
typedef signed char int8_t;
typedef short int16_t;



typedef long int32_t;


/*typedef long long int int64_t;*/




typedef unsigned char uint8_t;
typedef unsigned short uint16_t;



typedef unsigned long uint32_t;


/*typedef unsigned long long int uint64_t;*/
//# 42 "/opt/tinyos-2.x-contrib/diku/mcs51/tos/chips/mcs51/stdint.h" 3
typedef signed char int_least8_t;
typedef short int int_least16_t;
typedef long int_least32_t;









typedef unsigned char uint_least8_t;
typedef unsigned short uint_least16_t;
typedef unsigned long uint_least32_t;
//#line 70
typedef signed char int_fast8_t;





typedef int int_fast16_t;
typedef int int_fast32_t;





typedef unsigned char uint_fast8_t;





typedef unsigned int uint_fast16_t;
typedef unsigned int uint_fast32_t;
//#line 105
typedef int intptr_t;


typedef unsigned int uintptr_t;








/*__extension__*/ 
typedef
         float* intmax_t;
      
/*__extension__*/ 
typedef 
         float* uintmax_t;
      
//# 231 "/usr/include/inttypes.h" 3
//#line 228
typedef struct __nesc_unnamed4242 {
  intmax_t quot;
  intmax_t rem;
} imaxdiv_t;
/*YYYYY } imaxdiv_t*///# 385 "/usr/lib/ncc/nesc_nx.h"
typedef struct { unsigned char _data[1]; } /*__attribute__((packed))*/ nx_int8_t;typedef int8_t __nesc_nxbase_nx_int8_t  ;
typedef struct { unsigned char _data[2]; } /*__attribute__((packed))*/ nx_int16_t;typedef int16_t __nesc_nxbase_nx_int16_t  ;
typedef struct { unsigned char _data[4]; } /*__attribute__((packed))*/ nx_int32_t;typedef int32_t __nesc_nxbase_nx_int32_t  ;
typedef struct { unsigned char _data[8]; } /*__attribute__((packed))*/ nx_int64_t;/*typedef int64_t __nesc_nxbase_nx_int64_t  ;*/
typedef struct { unsigned char _data[1]; } /*__attribute__((packed))*/ nx_uint8_t;typedef uint8_t __nesc_nxbase_nx_uint8_t  ;
typedef struct { unsigned char _data[2]; } /*__attribute__((packed))*/ nx_uint16_t;typedef uint16_t __nesc_nxbase_nx_uint16_t  ;
typedef struct { unsigned char _data[4]; } /*__attribute__((packed))*/ nx_uint32_t;typedef uint32_t __nesc_nxbase_nx_uint32_t  ;
typedef struct { unsigned char _data[8]; } /*__attribute__((packed))*/ nx_uint64_t;/*typedef uint64_t __nesc_nxbase_nx_uint64_t  ;*/


typedef struct { unsigned char _data[1]; } /*__attribute__((packed))*/ nxle_int8_t;typedef int8_t __nesc_nxbase_nxle_int8_t  ;
typedef struct { unsigned char _data[2]; } /*__attribute__((packed))*/ nxle_int16_t;typedef int16_t __nesc_nxbase_nxle_int16_t  ;
typedef struct { unsigned char _data[4]; } /*__attribute__((packed))*/ nxle_int32_t;typedef int32_t __nesc_nxbase_nxle_int32_t  ;
typedef struct { unsigned char _data[8]; } /*__attribute__((packed))*/ nxle_int64_t;/*typedef int64_t __nesc_nxbase_nxle_int64_t  ;*/
typedef struct { unsigned char _data[1]; } /*__attribute__((packed))*/ nxle_uint8_t;typedef uint8_t __nesc_nxbase_nxle_uint8_t  ;
typedef struct { unsigned char _data[2]; } /*__attribute__((packed))*/ nxle_uint16_t;typedef uint16_t __nesc_nxbase_nxle_uint16_t  ;
typedef struct { unsigned char _data[4]; } /*__attribute__((packed))*/ nxle_uint32_t;typedef uint32_t __nesc_nxbase_nxle_uint32_t  ;
typedef struct { unsigned char _data[8]; } /*__attribute__((packed))*/ nxle_uint64_t;/*typedef uint64_t __nesc_nxbase_nxle_uint64_t  ;*/
//# 13 "/opt/tinyos-2.x-contrib/diku/mcs51/tos/chips/mcs51/string.h"
extern void *memset(void *s, char val, int n);
//# 14 "/usr/include/sys/lock.h" 3
typedef void *_LOCK_T;
//# 14 "/usr/include/sys/_types.h" 3
typedef long _off_t;
/*__extension__*/ 
//#line 15
typedef
         float* _off64_t;
      


typedef int _ssize_t;
//# 354 "/usr/lib/gcc/i686-pc-cygwin/3.4.4/include/stddef.h" 3
typedef unsigned int wint_t;
//# 35 "/usr/include/sys/_types.h" 3
//#line 27
typedef struct __nesc_unnamed4243 {

  int __count;
  union __nesc_unnamed4244 {

    wint_t __wch;
    unsigned char __wchb[4];
  } __value;
/*YYYYY   } __value*/} _mbstate_t;

typedef _LOCK_T _flock_t;


typedef void *_iconv_t;
//# 19 "/usr/include/sys/reent.h" 3
typedef unsigned long __ULong;
//#line 40
struct _Bigint {

  struct _Bigint *_next;
  int _k, _maxwds, _sign, _wds;
  __ULong _x[1];
};


struct __tm {

  int __tm_sec;
  int __tm_min;
  int __tm_hour;
  int __tm_mday;
  int __tm_mon;
  int __tm_year;
  int __tm_wday;
  int __tm_yday;
  int __tm_isdst;
};







struct _on_exit_args {
  void *_fnargs[32];
  void *_dso_handle[32];

  __ULong _fntypes;


  __ULong _is_cxa;
};









struct _atexit {
  struct _atexit *_next;
  int _ind;

  void (*_fns[32])(void );
  struct _on_exit_args _on_exit_args;
};









struct __sbuf {
  unsigned char *_base;
  int _size;
};






typedef long _fpos_t;



typedef _off64_t _fpos64_t;
//#line 166
struct __sFILE {
  unsigned char *_p;
  int _r;
  int _w;
  short _flags;
  short _file;
  struct __sbuf _bf;
  int _lbfsize;






  char *_cookie;

  _ssize_t (*_read)();
  _ssize_t (*_write)();

  _fpos_t (*_seek)();
  int (*_close)();


  struct __sbuf _ub;
  unsigned char *_up;
  int _ur;


  unsigned char _ubuf[3];
  unsigned char _nbuf[1];


  struct __sbuf _lb;


  int _blksize;
  int _offset;


  struct _reent *__data;



  _flock_t _lock;
};



struct __sFILE64 {
  unsigned char *_p;
  int _r;
  int _w;
  short _flags;
  short _file;
  struct __sbuf _bf;
  int _lbfsize;

  struct _reent *__data;


  char *_cookie;

  _ssize_t (*_read)();
  _ssize_t (*_write)();

  _fpos_t (*_seek)();
  int (*_close)();


  struct __sbuf _ub;
  unsigned char *_up;
  int _ur;


  unsigned char _ubuf[3];
  unsigned char _nbuf[1];


  struct __sbuf _lb;


  int _blksize;
  int _flags2;

  _off64_t _offset;
  _fpos64_t (*_seek64)();


  _flock_t _lock;
};

typedef struct __sFILE64 __FILE;




struct _glue {

  struct _glue *_next;
  int _niobs;
  __FILE *_iobs;
};
//#line 290
struct _rand48 {
  unsigned short _seed[3];
  unsigned short _mult[3];
  unsigned short _add;
};
//#line 565
struct _reent {

  int _errno;




  __FILE *_stdin, *_stdout, *_stderr;

  int _inc;
  char _emergency[25];

  int _current_category;
  char *_current_locale;

  int __sdidinit;

  void (*__cleanup)();


  struct _Bigint *_result;
  int _result_k;
  struct _Bigint *_p5s;
  struct _Bigint **_freelist;


  int _cvtlen;
  char *_cvtbuf;

  union __nesc_unnamed4245 {

    struct __nesc_unnamed4246 {

      unsigned int _unused_rand;
      char *_strtok_last;
      char _asctime_buf[26];
      struct __tm _localtime_buf;
      int _gamma_signgam;
      /*__extension__*/ 
         float* _rand_next;
      
      struct _rand48 _r48;
      _mbstate_t _mblen_state;
      _mbstate_t _mbtowc_state;
      _mbstate_t _wctomb_state;
      char _l64a_buf[8];
      char _signal_buf[24];
      int _getdate_err;
      _mbstate_t _mbrlen_state;
      _mbstate_t _mbrtowc_state;
      _mbstate_t _mbsrtowcs_state;
      _mbstate_t _wcrtomb_state;
      _mbstate_t _wcsrtombs_state;
    } _reent;



    struct __nesc_unnamed4247 {


      unsigned char *_nextf[30];
      unsigned int _nmalloc[30];
    } _unused;
  } _new;


  struct _atexit *_atexit;
  struct _atexit _atexit0;


  void (**_sig_func)(int arg_0x7f2108a0);




  struct _glue __sglue;
  __FILE __sf[3];
};
//#line 799
struct _reent;
struct _reent;









struct _reent;
//# 32 "/usr/include/stdlib.h" 3
//#line 28
typedef struct __nesc_unnamed4248 {

  int quot;
  int rem;
} div_t;
/*YYYYY } div_t*/




//#line 34
typedef struct __nesc_unnamed4249 {

  long quot;
  long rem;
} ldiv_t;
/*YYYYY } ldiv_t*/





//#line 41
typedef struct __nesc_unnamed4250 {


         float* quot;
      

         float* rem;
      
} lldiv_t;
/*YYYYY } lldiv_t*///# 24 "/opt/tinyos-2.x-contrib/diku/common/tos/system/tos.h"
typedef uint8_t bool  ;
enum __nesc_unnamed4251 {
//#line 25
  FALSE = 0, TRUE = 1
};









typedef nx_int8_t nx_bool;






/*struct __nesc_attr_atmostonce {
};*/
//#line 44
/*struct __nesc_attr_atleastonce {
};*/
//#line 45
/*struct __nesc_attr_exactlyonce {
};*/
//# 40 "/opt/tinyos-2.x/tos/types/TinyError.h"
enum __nesc_unnamed4252 {
  SUCCESS = 0, 
  FAIL = 1, 
  ESIZE = 2, 
  ECANCEL = 3, 
  EOFF = 4, 
  EBUSY = 5, 
  EINVAL = 6, 
  ERETRY = 7, 
  ERESERVE = 8, 
  EALREADY = 9, 
  ENOMEM = 10, 
  ENOACK = 11, 
  ELAST = 11
};

typedef uint8_t error_t  ;
//# 160 "/opt/tinyos-2.x-contrib/diku/mcs51/tos/chips/mcs51/io8051.h"
sbit EA = 0xAF;
//#line 197
sbit P1_3 = 0x93;
//#line 210
sbit P2_0 = 0xA0;
//# 67 "/opt/tinyos-2.x-contrib/diku/mcs51/tos/chips/cc2430/ioCC2430.h"
enum __nesc_unnamed4253 {
  CC2430_IEN0_EA = 0x7, 
  CC2430_IEN0_STIE = 0x5, 
  CC2430_IEN0_ENCIE = 0x4, 
  CC2430_IEN0_URX1IE = 0x3, 
  CC2430_IEN0_URX0IE = 0x2, 
  CC2430_IEN0_ADCIE = 0x1, 
  CC2430_IEN0_RFERRIE = 0x0
};





enum __nesc_unnamed4254 {
  CC2430_IEN2_WDTIE = 0x5, 
  CC2430_IEN2_P1IE = 0x4, 
  CC2430_IEN2_UTX1IE = 0x3, 
  CC2430_IEN2_UTX0IE = 0x2, 
  CC2430_IEN2_P2IE = 0x1, 
  CC2430_IEN2_RFIE = 0x0
};





enum __nesc_unnamed4255 {
  CC2430_IRCON_STIF = 0x7, 
  CC2430_IRCON_P0IF = 0x5, 
  CC2430_IRCON_T4IF = 0x4, 
  CC2430_IRCON_T3IF = 0x3, 
  CC2430_IRCON_T2IF = 0x2, 
  CC2430_IRCON_T1IF = 0x1, 
  CC2430_IRCON_DMAIF = 0x0
};





enum __nesc_unnamed4256 {
  CC2430_IRCON2_WDTIF = 0x4, 
  CC2430_IRCON2_P1IF = 0x3, 
  CC2430_IRCON2_UTX1IF = 0x2, 
  CC2430_IRCON2_UTX0IF = 0x1, 
  CC2430_IRCON2_P2IF = 0x0
};





enum __nesc_unnamed4257 {
  CC2430_RFIM_RREG_PD = 0x7, 
  CC2430_RFIM_TXDONE = 0x6, 
  CC2430_RFIM_FIFOP = 0x5, 
  CC2430_RFIM_SFD = 0x4, 
  CC2430_RFIM_CCA = 0x3, 
  CC2430_RFIM_CSP_WT = 0x2, 
  CC2430_RFIM_CSP_STOP = 0x1, 
  CC2430_RFIM_CSP_INT = 0x0
};





enum __nesc_unnamed4258 {
  CC2430_RFIF_RREG_ON = 0x7, 
  CC2430_RFIF_TXDONE = 0x6, 
  CC2430_RFIF_FIFOP = 0x5, 
  CC2430_RFIF_SFD = 0x4, 
  CC2430_RFIF_CCA = 0x3, 
  CC2430_RFIF_CSP_WT = 0x2, 
  CC2430_RFIF_CSP_STOP = 0x1, 
  CC2430_RFIF_CSP_INT = 0x0
};
//#line 168
sfr U0CSR = 0x86;
//#line 183
sbit URX0IF = 0x8B;
//#line 200
sfr IEN2 = 0x9A;
//#line 219
sbit URX0IE = 0xAA;
//#line 256
sfr SLEEP = 0xBE;


sfr U0BUF = 0xC1;
sfr U0BAUD = 0xC2;

sfr U0UCR = 0xC4;
sfr U0GCR = 0xC5;
sfr CLKCON = 0xC6;
//#line 334
sbit UTX0IF = 0xE9;
//#line 347
sfr PERCFG = 0xF1;




sfr P1_DIR = 0xFE;
sfr P2_DIR = 0xFF;


sfr P0_ALT = 0xF3;
//#line 393
//typedef uint16_t uint16_t xdata;
//typedef uint8_t uint8_t xdata;

//typedef uint16_t uint16_t code;
//typedef uint8_t uint8_t code;
//#line 479
enum __nesc_unnamed4259 {
  CC2430_RFPWR_ADI_RADIO_PD = 0x4, 
  CC2430_RFPWR_RREG_RADIO_PD = 0x3, 
  CC2430_RFPWR_RREG_DELAY = 0x0, 
  CC2430_RFPWR_RREG_DELAY_MASK = 0x7
};

enum __nesc_unnamed4260 {
  CC2430_RREG_DELAY_0 = 0x0, 
  CC2430_RREG_DELAY_31 = 0x1, 
  CC2430_RREG_DELAY_63 = 0x2, 
  CC2430_RREG_DELAY_125 = 0x3, 
  CC2430_RREG_DELAY_250 = 0x4, 
  CC2430_RREG_DELAY_500 = 0x5, 
  CC2430_RREG_DELAY_1000 = 0x6, 
  CC2430_RREG_DELAY_2000 = 0x7
};

enum __nesc_unnamed4261 {
  CC2430_MDMCTRL0L_AUTOCRC = 0x5, 
  CC2430_MDMCTRL0L_AUTOACK = 0x4
};

enum __nesc_unnamed4262 {
  CC2430_MDMCTRL0H_FRAME_FILT = 0x6, 
  CC2430_MDMCTRL0H_RESERVED_FRAME_MODE = 0x5, 
  CC2430_MDMCTRL0H_PAN_COORDINATOR = 0x4, 
  CC2430_MDMCTRL0H_ADDR_DECODE = 0x3, 
  CC2430_MDMCTRL0H_CCA_HYST = 0x0
};

enum __nesc_unnamed4263 {
  CC2430_RFSTATUS_TX_ACTIVE = 0x4, 
  CC2430_RFSTATUS_FIFO = 0x3, 
  CC2430_RFSTATUS_FIFOP = 0x2, 
  CC2430_RFSTATUS_SFD = 0x1, 
  CC2430_RFSTATUS_CCA = 0x0
};
//# 102 "/opt/tinyos-2.x-contrib/diku/mcs51/tos/chips/mcs51/mcs51hardware.h"
typedef uint8_t __nesc_atomic_t;

static   /*inline*/   void __nesc_disable_interrupt();
static   /*inline*/   void __nesc_enable_interrupt();

  /*inline*/   __nesc_atomic_t __nesc_atomic_start(void )  ;





  /*inline*/   void __nesc_atomic_end(__nesc_atomic_t oldSreg)  ;
//# 71 "/opt/tinyos-2.x-contrib/diku/mcs51/tos/chips/cc2430/timer/CC2430Timer.h"
enum __nesc_unnamed4264 {
  CC2430_SLEEP_OSC32K_CALDIS = 7, 
  CC2430_SLEEP_XOSC_STB = 6, 
  CC2430_SLEEP_HFRC_STB = 5, 
  CC2430_SLEEP_DIV = 3, 
  CC2430_SLEEP_OSC_PD = 2, 
  CC2430_SLEEP_MODE = 0, 
  CC2430_SLEEP_MODE_MASK = 3
};

enum __nesc_unnamed4265 {
  CC2430_SLEEP_POWERMODE_0 = 0, 
  CC2430_SLEEP_POWERMODE_1 = 1, 
  CC2430_SLEEP_POWERMODE_2 = 2, 
  CC2430_SLEEP_POWERMODE_3 = 3
};

enum __nesc_unnamed4266 {
  CC2430_CLKCON_OSC32K = 7, 
  CC2430_CLKCON_OSC = 6, 
  CC2430_CLKCON_TICKSPD = 3, 
  CC2430_CLKCON_CLKSPD = 0, 
  CC2430_CLKCON_TICKSPD_MASK = 0x38
};




enum cc2430_tick_spd_t {
  CC2430_TICKF_DIV_1 = 0x0 << CC2430_CLKCON_TICKSPD, 
  CC2430_TICKF_DIV_2 = 0x1 << CC2430_CLKCON_TICKSPD, 
  CC2430_TICKF_DIV_4 = 0x2 << CC2430_CLKCON_TICKSPD, 
  CC2430_TICKF_DIV_8 = 0x3 << CC2430_CLKCON_TICKSPD, 
  CC2430_TICKF_DIV_16 = 0x4 << CC2430_CLKCON_TICKSPD, 
  CC2430_TICKF_DIV_32 = 0x5 << CC2430_CLKCON_TICKSPD, 
  CC2430_TICKF_DIV_64 = 0x6 << CC2430_CLKCON_TICKSPD, 
  CC2430_TICKF_DIV_128 = 0x7 << CC2430_CLKCON_TICKSPD
};










enum __nesc_unnamed4267 {
  CC2430_T1CTL_CH2IF = 0x7, 
  CC2430_T1CTL_CH1IF = 0x6, 
  CC2430_T1CTL_CH0IF = 0x5, 
  CC2430_T1CTL_OVFIF = 0x4, 
  CC2430_T1CTL_DIV = 0x2, 
  CC2430_T1CTL_MODE = 0x0, 
  CC2430_T1CTL_MODE_MASK = 0x3, 
  CC2430_T1CTL_DIV_MASK = 0xc, 
  CC2430_T1CTL_IF_MASK = 0xf0
};






enum cc2430_timer1_mode_t {
  CC2430_TIMER1_MODE_OFF = 0x0 << CC2430_T1CTL_MODE, 
  CC2430_TIMER1_MODE_FREE = 0x1 << CC2430_T1CTL_MODE, 
  CC2430_TIMER1_MODE_MODULO = 0x2 << CC2430_T1CTL_MODE, 
  CC2430_TIMER1_MODE_UPDOWN = 0x3 << CC2430_T1CTL_MODE
};






enum cc2430_timer1_if_t {
  CC2430_T1_CH2IF = 1 << CC2430_T1CTL_CH2IF, 
  CC2430_T1_CH1IF = 1 << CC2430_T1CTL_CH1IF, 
  CC2430_T1_CH0IF = 1 << CC2430_T1CTL_CH0IF, 
  CC2430_T1_OVFIF = 1 << CC2430_T1CTL_OVFIF
};





enum cc2430_timer1_prescaler_t {
  CC2430_TIMER1_DIV_1 = 0x0 << CC2430_T1CTL_DIV, 
  CC2430_TIMER1_DIV_8 = 0x1 << CC2430_T1CTL_DIV, 
  CC2430_TIMER1_DIV_32 = 0x2 << CC2430_T1CTL_DIV, 
  CC2430_TIMER1_DIV_128 = 0x3 << CC2430_T1CTL_DIV
};





enum __nesc_unnamed4268 {
  CC2430_T1CCTLx_CPSEL = 0x7, 
  CC2430_T1CCTLx_IM = 0x6, 
  CC2430_T1CCTLx_CMP = 0x3, 
  CC2430_T1CCTLx_MODE = 0x2, 
  CC2430_T1CCTLx_CAP = 0x0
};










enum cc2430_timerMAC_mode_t {
  CC2430_TIMERMAC_MODE_IDLE = 0x0, 
  CC2430_TIMERMAC_MODE_RUN = 0x1
};





enum cc2430_timerMAC_T2CNF_t {
  CC2430_T2CNF_CMPIF = 0x7, 
  CC2430_T2CNF_PERIF = 0x6, 
  CC2430_T2CNF_OFCMPIF = 0x5, 
  CC2430_T2CNF_CMSEL = 0x3, 
  CC2430_T2CNF_SYNC = 0x1, 
  CC2430_T2CNF_RUN = 0x0
};






enum cc2430_timerMAC_T2PEROF2_t {
  CC2430_T2PEROF2_CMPIM = 0x7, 
  CC2430_T2PEROF2_PERIM = 0x6, 
  CC2430_T2PEROF2_OFCMPIM = 0x5
};

enum cc2430_timerMAC_if_t {
  CC2430_TMAC_CMPIF = 1 << CC2430_T2CNF_CMPIF, 
  CC2430_TMAC_PERIF = 1 << CC2430_T2CNF_PERIF, 
  CC2430_TMAC_OFCMPIF = 1 << CC2430_T2CNF_OFCMPIF
};

enum cc2430_timerMAC_interval_t {
  CC2430_TIMERWDT_32768 = 0, 
  CC2430_TIMERWDT_8192 = 1, 
  CC2430_TIMERWDT_512 = 2, 
  CC2430_TIMERWDT_64 = 3
};
//#line 240
enum __nesc_unnamed4269 {
  CC2430_TIMIF_OVFIM = 0x6
};

enum cc2430_timer34_if_t {
  CC2430_TIMIF_T4CH1IF = 0x5, 
  CC2430_TIMIF_T4CH0IF = 0x4, 
  CC2430_TIMIF_T4OVFIF = 0x3, 
  CC2430_TIMIF_T3CH1IF = 0x2, 
  CC2430_TIMIF_T3CH0IF = 0x1, 
  CC2430_TIMIF_T3OVFIF = 0x0
};






enum cc2430_timer3_4_mode_t {
  CC2430_TIMER3_4_MODE_FREE = 0x0, 
  CC2430_TIMER3_4_MODE_DOWN = 0x1, 
  CC2430_TIMER3_4_MODE_MODULO = 0x2, 
  CC2430_TIMER3_4_MODE_UPDOWN = 0x3
};





enum __nesc_unnamed4270 {
  CC2430_T34CTL_DIV = 0x5, 
  CC2430_T34CTL_START = 0x4, 
  CC2430_T34CTL_OVFIM = 0x3, 
  CC2430_T34CTL_WDTIF = 0x3, 
  CC2430_T34CTL_CLR = 0x2, 
  CC2430_T34CTL_MODE = 0x0, 
  CC2430_T34CTL_MODE_MASK = 0x3, 
  CC2430_T34CTL_DIV_MASK = 0xe0
};





enum cc2430_timer3_4_prescaler_t {
  CC2430_TIMER3_4_DIV_1 = 0x0 << CC2430_T34CTL_DIV, 
  CC2430_TIMER3_4_DIV_2 = 0x1 << CC2430_T34CTL_DIV, 
  CC2430_TIMER3_4_DIV_4 = 0x2 << CC2430_T34CTL_DIV, 
  CC2430_TIMER3_4_DIV_8 = 0x3 << CC2430_T34CTL_DIV, 
  CC2430_TIMER3_4_DIV_16 = 0x4 << CC2430_T34CTL_DIV, 
  CC2430_TIMER3_4_DIV_32 = 0x5 << CC2430_T34CTL_DIV, 
  CC2430_TIMER3_4_DIV_64 = 0x6 << CC2430_T34CTL_DIV, 
  CC2430_TIMER3_4_DIV_128 = 0x7 << CC2430_T34CTL_DIV
};










enum __nesc_unnamed4271 {
  CC2430_WDCTL_CLR = 0x4, 
  CC2430_WDCTL_EN = 0x3, 
  CC2430_WDCTL_MODE = 0x2, 
  CC2430_WDCTL_INT = 0x0, 
  CC2430_WDCTL_INT_MASK = 0x3
};

enum cc2430_timerWDT_mode_t {
  CC2430_TIMERWDT_MODE_WDT = 0x0 << CC2430_WDCTL_MODE, 
  CC2430_TIMERWDT_MODE_TIMER = 0x1 << CC2430_WDCTL_MODE
};
//# 42 "/opt/tinyos-2.x-contrib/diku/mcs51/tos/chips/mcs51/Timer.h"
typedef uint8_t TMilli;
typedef uint8_t T32khz;
typedef uint8_t TMicro;
//# 32 "/opt/tinyos-2.x/tos/types/Leds.h"
enum __nesc_unnamed4272 {
  LEDS_LED0 = 1 << 0, 
  LEDS_LED1 = 1 << 1, 
  LEDS_LED2 = 1 << 2, 
  LEDS_LED3 = 1 << 3, 
  LEDS_LED4 = 1 << 4, 
  LEDS_LED5 = 1 << 5, 
  LEDS_LED6 = 1 << 6, 
  LEDS_LED7 = 1 << 7
};
//# 51 "/opt/tinyos-2.x/tos/interfaces/Init.nc"
static error_t PlatformP__Init__init(void );
//# 56 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static error_t SchedulerBasicP__TaskBasic__postTask(
//# 45 "/opt/tinyos-2.x/tos/system/SchedulerBasicP.nc"
uint8_t arg_0x7ef72230);
//# 64 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static void SchedulerBasicP__TaskBasic__default__runTask(
//# 45 "/opt/tinyos-2.x/tos/system/SchedulerBasicP.nc"
uint8_t arg_0x7ef72230);
//# 46 "/opt/tinyos-2.x/tos/interfaces/Scheduler.nc"
static void SchedulerBasicP__Scheduler__init(void );
//#line 61
static void SchedulerBasicP__Scheduler__taskLoop(void );
//#line 54
static bool SchedulerBasicP__Scheduler__runNextTask(void );
//# 59 "/opt/tinyos-2.x/tos/interfaces/McuSleep.nc"
static void McuSleepC__McuSleep__sleep(void );
//# 49 "/opt/tinyos-2.x/tos/interfaces/Boot.nc"
static void EchoP__Boot__booted(void );
//# 64 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static void EchoP__print__runTask(void );
//# 47 "/opt/tinyos-2.x/tos/lib/serial/SerialByteComm.nc"
static void EchoP__uart0__get(uint8_t _data);





static void EchoP__uart0__putDone(void );
//# 51 "/opt/tinyos-2.x/tos/interfaces/Init.nc"
static error_t LedsP__Init__init(void );
//# 56 "/opt/tinyos-2.x/tos/interfaces/Leds.nc"
static void LedsP__Leds__led0Toggle(void );
//#line 72
static void LedsP__Leds__led1Toggle(void );
//#line 89
static void LedsP__Leds__led2Toggle(void );
//#line 45
static void LedsP__Leds__led0On(void );
//# 31 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
static void HplMcs51GeneralIOC__P13__toggle(void );



static void HplMcs51GeneralIOC__P13__makeOutput(void );
//#line 30
static void HplMcs51GeneralIOC__P13__clr(void );
static void HplMcs51GeneralIOC__P20__toggle(void );



static void HplMcs51GeneralIOC__P20__makeOutput(void );
//#line 29
static void HplMcs51GeneralIOC__P20__set(void );
static void HplMcs51GeneralIOC__P20__clr(void );
static void /*PlatformLedsC.Led0_rev*/ReverseGPIOP__0__Out__toggle(void );



static void /*PlatformLedsC.Led0_rev*/ReverseGPIOP__0__Out__makeOutput(void );
//#line 29
static void /*PlatformLedsC.Led0_rev*/ReverseGPIOP__0__Out__set(void );
static void /*PlatformLedsC.Led0_rev*/ReverseGPIOP__0__Out__clr(void );
static void /*PlatformLedsC.Led2_rev*/ReverseGPIOP__1__Out__toggle(void );



static void /*PlatformLedsC.Led2_rev*/ReverseGPIOP__1__Out__makeOutput(void );
//#line 29
static void /*PlatformLedsC.Led2_rev*/ReverseGPIOP__1__Out__set(void );

static void /*PlatformLedsC.NoPinC*/NoPinC__0__GeneralIO__toggle(void );



static void /*PlatformLedsC.NoPinC*/NoPinC__0__GeneralIO__makeOutput(void );
//#line 29
static void /*PlatformLedsC.NoPinC*/NoPinC__0__GeneralIO__set(void );
//# 51 "/opt/tinyos-2.x/tos/interfaces/Init.nc"
static error_t HalCC2430SimpleUartP__Init__init(void );
//# 41 "/opt/tinyos-2.x/tos/lib/serial/SerialByteComm.nc"
static error_t HalCC2430SimpleUartP__uart0__put(uint8_t _data);
//# 51 "/opt/tinyos-2.x/tos/interfaces/Init.nc"
static error_t PlatformP__LedsInit__init(void );
//# 48 "/opt/tinyos-2.x-contrib/diku/mcs51/tos/platforms/cc2430em/PlatformP.nc"
static  /*inline*/  error_t PlatformP__Init__init(void );
//# 51 "/opt/tinyos-2.x/tos/interfaces/Init.nc"
static error_t RealMainP__SoftwareInit__init(void );
//# 49 "/opt/tinyos-2.x/tos/interfaces/Boot.nc"
static void RealMainP__Boot__booted(void );
//# 51 "/opt/tinyos-2.x/tos/interfaces/Init.nc"
static error_t RealMainP__PlatformInit__init(void );
//# 46 "/opt/tinyos-2.x/tos/interfaces/Scheduler.nc"
static void RealMainP__Scheduler__init(void );
//#line 61
static void RealMainP__Scheduler__taskLoop(void );
//#line 54
static bool RealMainP__Scheduler__runNextTask(void );
//# 52 "/opt/tinyos-2.x/tos/system/RealMainP.nc"
int main(void )   ;
//# 64 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static void SchedulerBasicP__TaskBasic__runTask(
//# 45 "/opt/tinyos-2.x/tos/system/SchedulerBasicP.nc"
uint8_t arg_0x7ef72230);
//# 59 "/opt/tinyos-2.x/tos/interfaces/McuSleep.nc"
static void SchedulerBasicP__McuSleep__sleep(void );
//# 50 "/opt/tinyos-2.x/tos/system/SchedulerBasicP.nc"
enum SchedulerBasicP____nesc_unnamed4273 {

  SchedulerBasicP__NUM_TASKS = 1U, 
  SchedulerBasicP__NO_TASK = 255
};

uint8_t SchedulerBasicP__m_head;
uint8_t SchedulerBasicP__m_tail;
uint8_t SchedulerBasicP__m_next[SchedulerBasicP__NUM_TASKS];








static   /*inline*/   uint8_t SchedulerBasicP__popTask(void );
//#line 86
static  /*inline*/  bool SchedulerBasicP__isWaiting(uint8_t id);




static  /*inline*/  bool SchedulerBasicP__pushTask(uint8_t id);
//#line 113
static  /*inline*/  void SchedulerBasicP__Scheduler__init(void );









static bool SchedulerBasicP__Scheduler__runNextTask(void );
//#line 138
static  /*inline*/  void SchedulerBasicP__Scheduler__taskLoop(void );
//#line 159
static  /*inline*/  error_t SchedulerBasicP__TaskBasic__postTask(uint8_t id);




static  /*inline*/  void SchedulerBasicP__TaskBasic__default__runTask(uint8_t id);
//# 51 "/opt/tinyos-2.x-contrib/diku/mcs51/tos/chips/cc2430/McuSleepC.nc"
static  /*inline*/  void McuSleepC__McuSleep__sleep(void );
//# 56 "/opt/tinyos-2.x/tos/interfaces/Leds.nc"
static void EchoP__Leds__led0Toggle(void );
//#line 72
static void EchoP__Leds__led1Toggle(void );
//#line 89
static void EchoP__Leds__led2Toggle(void );
//#line 45
static void EchoP__Leds__led0On(void );
//# 56 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static error_t EchoP__print__postTask(void );
//# 41 "/opt/tinyos-2.x/tos/lib/serial/SerialByteComm.nc"
static error_t EchoP__uart0__put(uint8_t _data);
//# 64 "EchoP.nc"
enum EchoP____nesc_unnamed4274 {
//#line 64
  EchoP__print = 0U
};
//#line 64
typedef int EchoP____nesc_sillytask_print[];
//#line 50
char EchoP__buf = 'A';





static  /*inline*/  void EchoP__Boot__booted(void );




static  /*inline*/  void EchoP__uart0__putDone(void );


static  /*inline*/  void EchoP__print__runTask(void );



static  /*inline*/  void EchoP__uart0__get(uint8_t _data);
//# 31 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
static void LedsP__Led0__toggle(void );



static void LedsP__Led0__makeOutput(void );
//#line 29
static void LedsP__Led0__set(void );
static void LedsP__Led0__clr(void );
static void LedsP__Led1__toggle(void );



static void LedsP__Led1__makeOutput(void );
//#line 29
static void LedsP__Led1__set(void );

static void LedsP__Led2__toggle(void );



static void LedsP__Led2__makeOutput(void );
//#line 29
static void LedsP__Led2__set(void );
//# 45 "/opt/tinyos-2.x/tos/system/LedsP.nc"
static  /*inline*/  error_t LedsP__Init__init(void );
//#line 63
static  /*inline*/  void LedsP__Leds__led0On(void );









static  /*inline*/  void LedsP__Leds__led0Toggle(void );
//#line 88
static  /*inline*/  void LedsP__Leds__led1Toggle(void );
//#line 103
static  /*inline*/  void LedsP__Leds__led2Toggle(void );
//# 97 "/opt/tinyos-2.x-contrib/diku/mcs51/tos/chips/mcs51/pins/HplMcs51GeneralIOC.nc"
static   /*inline*/   void HplMcs51GeneralIOC__P13__clr(void );
//#line 97
static  /*inline*/  void HplMcs51GeneralIOC__P13__toggle(void );
//#line 97
static   /*inline*/   void HplMcs51GeneralIOC__P13__makeOutput(void );





static   /*inline*/   void HplMcs51GeneralIOC__P20__set(void );
//#line 103
static   /*inline*/   void HplMcs51GeneralIOC__P20__clr(void );
//#line 103
static  /*inline*/  void HplMcs51GeneralIOC__P20__toggle(void );
//#line 103
static   /*inline*/   void HplMcs51GeneralIOC__P20__makeOutput(void );
//# 31 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
static void /*PlatformLedsC.Led0_rev*/ReverseGPIOP__0__In__toggle(void );



static void /*PlatformLedsC.Led0_rev*/ReverseGPIOP__0__In__makeOutput(void );
//#line 29
static void /*PlatformLedsC.Led0_rev*/ReverseGPIOP__0__In__set(void );
static void /*PlatformLedsC.Led0_rev*/ReverseGPIOP__0__In__clr(void );
//# 59 "/opt/tinyos-2.x-contrib/diku/common/lib/ReverseGPIOP.nc"
static   /*inline*/   void /*PlatformLedsC.Led0_rev*/ReverseGPIOP__0__Out__set(void );
//#line 59
static   /*inline*/   void /*PlatformLedsC.Led0_rev*/ReverseGPIOP__0__Out__clr(void );
//#line 59
static  /*inline*/  void /*PlatformLedsC.Led0_rev*/ReverseGPIOP__0__Out__toggle(void );
//#line 59
static   /*inline*/   void /*PlatformLedsC.Led0_rev*/ReverseGPIOP__0__Out__makeOutput(void );
//# 31 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
static void /*PlatformLedsC.Led2_rev*/ReverseGPIOP__1__In__toggle(void );



static void /*PlatformLedsC.Led2_rev*/ReverseGPIOP__1__In__makeOutput(void );
//#line 30
static void /*PlatformLedsC.Led2_rev*/ReverseGPIOP__1__In__clr(void );
//# 59 "/opt/tinyos-2.x-contrib/diku/common/lib/ReverseGPIOP.nc"
static   /*inline*/   void /*PlatformLedsC.Led2_rev*/ReverseGPIOP__1__Out__set(void );
//#line 59
static  /*inline*/  void /*PlatformLedsC.Led2_rev*/ReverseGPIOP__1__Out__toggle(void );
//#line 59
static   /*inline*/   void /*PlatformLedsC.Led2_rev*/ReverseGPIOP__1__Out__makeOutput(void );
//# 23 "/opt/tinyos-2.x/tos/system/NoPinC.nc"
static  /*inline*/  void /*PlatformLedsC.NoPinC*/NoPinC__0__GeneralIO__set(void );

static  /*inline*/  void /*PlatformLedsC.NoPinC*/NoPinC__0__GeneralIO__toggle(void );

static  /*inline*/  void /*PlatformLedsC.NoPinC*/NoPinC__0__GeneralIO__makeOutput(void );
//# 47 "/opt/tinyos-2.x/tos/lib/serial/SerialByteComm.nc"
static void HalCC2430SimpleUartP__uart0__get(uint8_t _data);





static void HalCC2430SimpleUartP__uart0__putDone(void );
//# 50 "/opt/tinyos-2.x-contrib/diku/mcs51/tos/chips/cc2430/usart/HalCC2430SimpleUartP.nc"
static  /*inline*/  error_t HalCC2430SimpleUartP__Init__init(void );
//#line 130
static  /*inline*/  error_t HalCC2430SimpleUartP__uart0__put(uint8_t _data);




void __vector_2(void )   /*__attribute((interrupt))*/ ;
//#line 154
void __vector_7(void )   /*__attribute((interrupt))*/ ;
//# 107 "/opt/tinyos-2.x-contrib/diku/mcs51/tos/chips/mcs51/mcs51hardware.h"
  /*inline*/    __nesc_atomic_t __nesc_atomic_start(void )
//#line 107
{
  __nesc_atomic_t tmp = EA;

//#line 109
  EA = 0;
  return tmp;
}

  /*inline*/    void __nesc_atomic_end(__nesc_atomic_t oldSreg)
//#line 113
{
  EA = oldSreg;
}

//# 113 "/opt/tinyos-2.x/tos/system/SchedulerBasicP.nc"
static  /*inline*/  void SchedulerBasicP__Scheduler__init(void )
{
  /* atomic removed: atomic calls only */
  {
    memset((void *)SchedulerBasicP__m_next, SchedulerBasicP__NO_TASK, sizeof SchedulerBasicP__m_next);
    SchedulerBasicP__m_head = SchedulerBasicP__NO_TASK;
    SchedulerBasicP__m_tail = SchedulerBasicP__NO_TASK;
  }
}

//# 46 "/opt/tinyos-2.x/tos/interfaces/Scheduler.nc"
 /*inline*/  static void RealMainP__Scheduler__init(void ){
//#line 46
  SchedulerBasicP__Scheduler__init();
//#line 46
}
//#line 46
//# 97 "/opt/tinyos-2.x-contrib/diku/mcs51/tos/chips/mcs51/pins/HplMcs51GeneralIOC.nc"
static   /*inline*/   void HplMcs51GeneralIOC__P13__clr(void )
//#line 97
{
//#line 97
  P1_3 = 0;
}

//# 30 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
 /*inline*/  static void /*PlatformLedsC.Led2_rev*/ReverseGPIOP__1__In__clr(void ){
//#line 30
  HplMcs51GeneralIOC__P13__clr();
//#line 30
}
//#line 30
//# 59 "/opt/tinyos-2.x-contrib/diku/common/lib/ReverseGPIOP.nc"
static   /*inline*/   void /*PlatformLedsC.Led2_rev*/ReverseGPIOP__1__Out__set(void )
//#line 59
{
//#line 59
  /*PlatformLedsC.Led2_rev*/ReverseGPIOP__1__In__clr();
}

//# 29 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
 /*inline*/  static void LedsP__Led2__set(void ){
//#line 29
  /*PlatformLedsC.Led2_rev*/ReverseGPIOP__1__Out__set();
//#line 29
}
//#line 29
//# 23 "/opt/tinyos-2.x/tos/system/NoPinC.nc"
static  /*inline*/  void /*PlatformLedsC.NoPinC*/NoPinC__0__GeneralIO__set(void )
//#line 23
{
}

//# 29 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
 /*inline*/  static void LedsP__Led1__set(void ){
//#line 29
  /*PlatformLedsC.NoPinC*/NoPinC__0__GeneralIO__set();
//#line 29
}
//#line 29
//# 103 "/opt/tinyos-2.x-contrib/diku/mcs51/tos/chips/mcs51/pins/HplMcs51GeneralIOC.nc"
static   /*inline*/   void HplMcs51GeneralIOC__P20__clr(void )
//#line 103
{
//#line 103
  P2_0 = 0;
}

//# 30 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
 /*inline*/  static void /*PlatformLedsC.Led0_rev*/ReverseGPIOP__0__In__clr(void ){
//#line 30
  HplMcs51GeneralIOC__P20__clr();
//#line 30
}
//#line 30
//# 59 "/opt/tinyos-2.x-contrib/diku/common/lib/ReverseGPIOP.nc"
static   /*inline*/   void /*PlatformLedsC.Led0_rev*/ReverseGPIOP__0__Out__set(void )
//#line 59
{
//#line 59
  /*PlatformLedsC.Led0_rev*/ReverseGPIOP__0__In__clr();
}

//# 29 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
 /*inline*/  static void LedsP__Led0__set(void ){
//#line 29
  /*PlatformLedsC.Led0_rev*/ReverseGPIOP__0__Out__set();
//#line 29
}
//#line 29
//# 97 "/opt/tinyos-2.x-contrib/diku/mcs51/tos/chips/mcs51/pins/HplMcs51GeneralIOC.nc"
static   /*inline*/   void HplMcs51GeneralIOC__P13__makeOutput(void )
//#line 97
{
//#line 97
  P1_DIR |= 1 << 3;
}

//# 35 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
 /*inline*/  static void /*PlatformLedsC.Led2_rev*/ReverseGPIOP__1__In__makeOutput(void ){
//#line 35
  HplMcs51GeneralIOC__P13__makeOutput();
//#line 35
}
//#line 35
//# 59 "/opt/tinyos-2.x-contrib/diku/common/lib/ReverseGPIOP.nc"
static   /*inline*/   void /*PlatformLedsC.Led2_rev*/ReverseGPIOP__1__Out__makeOutput(void )
//#line 59
{
//#line 59
  /*PlatformLedsC.Led2_rev*/ReverseGPIOP__1__In__makeOutput();
}

//# 35 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
 /*inline*/  static void LedsP__Led2__makeOutput(void ){
//#line 35
  /*PlatformLedsC.Led2_rev*/ReverseGPIOP__1__Out__makeOutput();
//#line 35
}
//#line 35
//# 27 "/opt/tinyos-2.x/tos/system/NoPinC.nc"
static  /*inline*/  void /*PlatformLedsC.NoPinC*/NoPinC__0__GeneralIO__makeOutput(void )
//#line 27
{
}

//# 35 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
 /*inline*/  static void LedsP__Led1__makeOutput(void ){
//#line 35
  /*PlatformLedsC.NoPinC*/NoPinC__0__GeneralIO__makeOutput();
//#line 35
}
//#line 35
//# 103 "/opt/tinyos-2.x-contrib/diku/mcs51/tos/chips/mcs51/pins/HplMcs51GeneralIOC.nc"
static   /*inline*/   void HplMcs51GeneralIOC__P20__makeOutput(void )
//#line 103
{
//#line 103
  P2_DIR |= 1 << 0;
}

//# 35 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
 /*inline*/  static void /*PlatformLedsC.Led0_rev*/ReverseGPIOP__0__In__makeOutput(void ){
//#line 35
  HplMcs51GeneralIOC__P20__makeOutput();
//#line 35
}
//#line 35
//# 59 "/opt/tinyos-2.x-contrib/diku/common/lib/ReverseGPIOP.nc"
static   /*inline*/   void /*PlatformLedsC.Led0_rev*/ReverseGPIOP__0__Out__makeOutput(void )
//#line 59
{
//#line 59
  /*PlatformLedsC.Led0_rev*/ReverseGPIOP__0__In__makeOutput();
}

//# 35 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
 /*inline*/  static void LedsP__Led0__makeOutput(void ){
//#line 35
  /*PlatformLedsC.Led0_rev*/ReverseGPIOP__0__Out__makeOutput();
//#line 35
}
//#line 35
//# 45 "/opt/tinyos-2.x/tos/system/LedsP.nc"
static  /*inline*/  error_t LedsP__Init__init(void )
//#line 45
{
  /* atomic removed: atomic calls only */
//#line 46
  {
    ;
    LedsP__Led0__makeOutput();
    LedsP__Led1__makeOutput();
    LedsP__Led2__makeOutput();
    LedsP__Led0__set();
    LedsP__Led1__set();
    LedsP__Led2__set();
  }
  return SUCCESS;
}

//# 51 "/opt/tinyos-2.x/tos/interfaces/Init.nc"
 /*inline*/  static error_t PlatformP__LedsInit__init(void ){
//#line 51
  unsigned char result;
//#line 51

//#line 51
  result = LedsP__Init__init();
//#line 51

//#line 51
  return result;
//#line 51
}
//#line 51
//# 48 "/opt/tinyos-2.x-contrib/diku/mcs51/tos/platforms/cc2430em/PlatformP.nc"
static  /*inline*/  error_t PlatformP__Init__init(void )
//#line 48
{
  uint8_t new_clkcon;

//#line 88
  SLEEP = (SLEEP & ~CC2430_SLEEP_MODE_MASK) | CC2430_SLEEP_POWERMODE_0;



  SLEEP &= ~(1 << CC2430_SLEEP_OSC_PD);
  while (!(SLEEP & (1 << CC2430_SLEEP_XOSC_STB))) ;

  new_clkcon = 0x0;

  new_clkcon = (new_clkcon & ~CC2430_CLKCON_TICKSPD_MASK) | CC2430_TICKF_DIV_1;


  new_clkcon &= ~(1 << CC2430_CLKCON_OSC32K);






  new_clkcon &= ~(1 << CC2430_CLKCON_OSC);

  new_clkcon |= 1 << CC2430_CLKCON_TICKSPD;

  CLKCON = new_clkcon;
//#line 137
  PlatformP__LedsInit__init();
  return SUCCESS;
}

//# 51 "/opt/tinyos-2.x/tos/interfaces/Init.nc"
 /*inline*/  static error_t RealMainP__PlatformInit__init(void ){
//#line 51
  unsigned char result;
//#line 51

//#line 51
  result = PlatformP__Init__init();
//#line 51

//#line 51
  return result;
//#line 51
}
//#line 51
//# 54 "/opt/tinyos-2.x/tos/interfaces/Scheduler.nc"
 /*inline*/  static bool RealMainP__Scheduler__runNextTask(void ){
//#line 54
  unsigned char result;
//#line 54

//#line 54
  result = SchedulerBasicP__Scheduler__runNextTask();
//#line 54

//#line 54
  return result;
//#line 54
}
//#line 54
//# 50 "/opt/tinyos-2.x-contrib/diku/mcs51/tos/chips/cc2430/usart/HalCC2430SimpleUartP.nc"
static  /*inline*/  error_t HalCC2430SimpleUartP__Init__init(void )
//#line 50
{
//#line 92
  PERCFG &= ~0x1u;
//#line 105
  P0_ALT |= 0x0Cu;




  U0GCR = 57600 == 2400 ? 6 + ((CLKCON & (1 << CC2430_CLKCON_OSC)) >> CC2430_CLKCON_OSC) : 57600 == 4800 ? 7 + ((CLKCON & (1 << CC2430_CLKCON_OSC)) >> CC2430_CLKCON_OSC) : 57600 == 9600 ? 8 + ((CLKCON & (1 << CC2430_CLKCON_OSC)) >> CC2430_CLKCON_OSC) : 57600 == 14400 ? 8 + ((CLKCON & (1 << CC2430_CLKCON_OSC)) >> CC2430_CLKCON_OSC) : 57600 == 19200 ? 9 + ((CLKCON & (1 << CC2430_CLKCON_OSC)) >> CC2430_CLKCON_OSC) : 57600 == 28800 ? 9 + ((CLKCON & (1 << CC2430_CLKCON_OSC)) >> CC2430_CLKCON_OSC) : 57600 == 38400 ? 10 + ((CLKCON & (1 << CC2430_CLKCON_OSC)) >> CC2430_CLKCON_OSC) : 57600 == 57600 ? 10 + ((CLKCON & (1 << CC2430_CLKCON_OSC)) >> CC2430_CLKCON_OSC) : 57600 == 76800 ? 11 + ((CLKCON & (1 << CC2430_CLKCON_OSC)) >> CC2430_CLKCON_OSC) : 57600 == 115200 ? 11 + ((CLKCON & (1 << CC2430_CLKCON_OSC)) >> CC2430_CLKCON_OSC) : 57600 == 153600 ? 12 + ((CLKCON & (1 << CC2430_CLKCON_OSC)) >> CC2430_CLKCON_OSC) : 57600 == 230400 ? 12 + ((CLKCON & (1 << CC2430_CLKCON_OSC)) >> CC2430_CLKCON_OSC) : 57600 == 307200 ? 13 + ((CLKCON & (1 << CC2430_CLKCON_OSC)) >> CC2430_CLKCON_OSC) : 0;


  U0BAUD = 57600 == 2400 ? 59 : 57600 == 4800 ? 59 : 57600 == 9600 ? 59 : 57600 == 14400 ? 216 : 57600 == 19200 ? 59 : 57600 == 28800 ? 216 : 57600 == 38400 ? 59 : 57600 == 57600 ? 216 : 57600 == 76800 ? 59 : 57600 == 115200 ? 216 : 57600 == 153600 ? 59 : 57600 == 230400 ? 216 : 57600 == 307200 ? 59 : 0;

  U0CSR |= 0x80u | 0x40u;
  U0UCR |= 0x2u | 0x80u;


  UTX0IF = 0;
  URX0IF = 0;

  URX0IE = 1;


  IEN2 |= 1 << 2;

  return SUCCESS;
}

//# 51 "/opt/tinyos-2.x/tos/interfaces/Init.nc"
 /*inline*/  static error_t RealMainP__SoftwareInit__init(void ){
//#line 51
  unsigned char result;
//#line 51

//#line 51
  result = HalCC2430SimpleUartP__Init__init();
//#line 51

//#line 51
  return result;
//#line 51
}
//#line 51
//# 130 "/opt/tinyos-2.x-contrib/diku/mcs51/tos/chips/cc2430/usart/HalCC2430SimpleUartP.nc"
static  /*inline*/  error_t HalCC2430SimpleUartP__uart0__put(uint8_t _data)
//#line 130
{
  U0BUF = _data;
  return SUCCESS;
}

//# 41 "/opt/tinyos-2.x/tos/lib/serial/SerialByteComm.nc"
 /*inline*/  static error_t EchoP__uart0__put(uint8_t _data){
//#line 41
  unsigned char result;
//#line 41

//#line 41
  result = HalCC2430SimpleUartP__uart0__put(_data);
//#line 41

//#line 41
  return result;
//#line 41
}
//#line 41
//# 103 "/opt/tinyos-2.x-contrib/diku/mcs51/tos/chips/mcs51/pins/HplMcs51GeneralIOC.nc"
static   /*inline*/   void HplMcs51GeneralIOC__P20__set(void )
//#line 103
{
//#line 103
  P2_0 = 1;
}

//# 29 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
 /*inline*/  static void /*PlatformLedsC.Led0_rev*/ReverseGPIOP__0__In__set(void ){
//#line 29
  HplMcs51GeneralIOC__P20__set();
//#line 29
}
//#line 29
//# 59 "/opt/tinyos-2.x-contrib/diku/common/lib/ReverseGPIOP.nc"
static   /*inline*/   void /*PlatformLedsC.Led0_rev*/ReverseGPIOP__0__Out__clr(void )
//#line 59
{
//#line 59
  /*PlatformLedsC.Led0_rev*/ReverseGPIOP__0__In__set();
}

//# 30 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
 /*inline*/  static void LedsP__Led0__clr(void ){
//#line 30
  /*PlatformLedsC.Led0_rev*/ReverseGPIOP__0__Out__clr();
//#line 30
}
//#line 30
//# 63 "/opt/tinyos-2.x/tos/system/LedsP.nc"
static  /*inline*/  void LedsP__Leds__led0On(void )
//#line 63
{
  LedsP__Led0__clr();
  ;
//#line 65
  ;
}

//# 45 "/opt/tinyos-2.x/tos/interfaces/Leds.nc"
 /*inline*/  static void EchoP__Leds__led0On(void ){
//#line 45
  LedsP__Leds__led0On();
//#line 45
}
//#line 45
//# 56 "EchoP.nc"
static  /*inline*/  void EchoP__Boot__booted(void )
//#line 56
{
  EchoP__Leds__led0On();
  EchoP__uart0__put(EchoP__buf);
}

//# 49 "/opt/tinyos-2.x/tos/interfaces/Boot.nc"
 /*inline*/  static void RealMainP__Boot__booted(void ){
//#line 49
  EchoP__Boot__booted();
//#line 49
}
//#line 49
//# 64 "EchoP.nc"
static  /*inline*/  void EchoP__print__runTask(void )
//#line 64
{
  EchoP__uart0__put(EchoP__buf);
}

//# 164 "/opt/tinyos-2.x/tos/system/SchedulerBasicP.nc"
static  /*inline*/  void SchedulerBasicP__TaskBasic__default__runTask(uint8_t id)
{
}

//# 64 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
 /*inline*/  static void SchedulerBasicP__TaskBasic__runTask(uint8_t arg_0x7ef72230){
//#line 64
  switch (arg_0x7ef72230) {
//#line 64
    case EchoP__print:
//#line 64
      EchoP__print__runTask();
//#line 64
      break;
//#line 64
    default:
//#line 64
      SchedulerBasicP__TaskBasic__default__runTask(arg_0x7ef72230);
//#line 64
      break;
//#line 64
    }
//#line 64
}
//#line 64
//# 104 "/opt/tinyos-2.x-contrib/diku/mcs51/tos/chips/mcs51/mcs51hardware.h"
static   /*inline*/   void __nesc_disable_interrupt()
//#line 104
{
//#line 104
  EA = 0;
}

//#line 105
static   /*inline*/   void __nesc_enable_interrupt()
//#line 105
{
//#line 105
  EA = 1;
}

//# 51 "/opt/tinyos-2.x-contrib/diku/mcs51/tos/chips/cc2430/McuSleepC.nc"
static  /*inline*/  void McuSleepC__McuSleep__sleep(void )
//#line 51
{

  __nesc_enable_interrupt();






  SLEEP = (SLEEP & ~CC2430_SLEEP_MODE_MASK) | CC2430_SLEEP_POWERMODE_0;

  __nesc_disable_interrupt();
}

//# 59 "/opt/tinyos-2.x/tos/interfaces/McuSleep.nc"
 /*inline*/  static void SchedulerBasicP__McuSleep__sleep(void ){
//#line 59
  McuSleepC__McuSleep__sleep();
//#line 59
}
//#line 59
//# 67 "/opt/tinyos-2.x/tos/system/SchedulerBasicP.nc"
static   /*inline*/   uint8_t SchedulerBasicP__popTask(void )
{
  if (SchedulerBasicP__m_head != SchedulerBasicP__NO_TASK) 
    {
      uint8_t id = SchedulerBasicP__m_head;

//#line 72
      SchedulerBasicP__m_head = SchedulerBasicP__m_next[SchedulerBasicP__m_head];
      if (SchedulerBasicP__m_head == SchedulerBasicP__NO_TASK) 
        {
          SchedulerBasicP__m_tail = SchedulerBasicP__NO_TASK;
        }
      SchedulerBasicP__m_next[id] = SchedulerBasicP__NO_TASK;
      return id;
    }
  else 
    {
      return SchedulerBasicP__NO_TASK;
    }
}

//#line 138
static  /*inline*/  void SchedulerBasicP__Scheduler__taskLoop(void )
{
  for (; ; ) 
    {
      uint8_t nextTask;

      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
        {
          while ((nextTask = SchedulerBasicP__popTask()) == SchedulerBasicP__NO_TASK) 
            {
              SchedulerBasicP__McuSleep__sleep();
            }
        }
//#line 150
        __nesc_atomic_end(__nesc_atomic); }
      SchedulerBasicP__TaskBasic__runTask(nextTask);
    }
}

//# 61 "/opt/tinyos-2.x/tos/interfaces/Scheduler.nc"
 /*inline*/  static void RealMainP__Scheduler__taskLoop(void ){
//#line 61
  SchedulerBasicP__Scheduler__taskLoop();
//#line 61
}
//#line 61
//# 86 "/opt/tinyos-2.x/tos/system/SchedulerBasicP.nc"
static  /*inline*/  bool SchedulerBasicP__isWaiting(uint8_t id)
{
  return SchedulerBasicP__m_next[id] != SchedulerBasicP__NO_TASK || SchedulerBasicP__m_tail == id;
}

static  /*inline*/  bool SchedulerBasicP__pushTask(uint8_t id)
{
  if (!SchedulerBasicP__isWaiting(id)) 
    {
      if (SchedulerBasicP__m_head == SchedulerBasicP__NO_TASK) 
        {
          SchedulerBasicP__m_head = id;
          SchedulerBasicP__m_tail = id;
        }
      else 
        {
          SchedulerBasicP__m_next[SchedulerBasicP__m_tail] = id;
          SchedulerBasicP__m_tail = id;
        }
      return TRUE;
    }
  else 
    {
      return FALSE;
    }
}

//#line 159
static  /*inline*/  error_t SchedulerBasicP__TaskBasic__postTask(uint8_t id)
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
//#line 161
    {
//#line 161
      {
        unsigned char __nesc_temp = 
//#line 161
        SchedulerBasicP__pushTask(id) ? SUCCESS : EBUSY;

        {
//#line 161
          __nesc_atomic_end(__nesc_atomic); 
//#line 161
          return __nesc_temp;
        }
      }
    }
//#line 164
    __nesc_atomic_end(__nesc_atomic); }
}

//# 56 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
 /*inline*/  static error_t EchoP__print__postTask(void ){
//#line 56
  unsigned char result;
//#line 56

//#line 56
  result = SchedulerBasicP__TaskBasic__postTask(EchoP__print);
//#line 56

//#line 56
  return result;
//#line 56
}
//#line 56
//# 97 "/opt/tinyos-2.x-contrib/diku/mcs51/tos/chips/mcs51/pins/HplMcs51GeneralIOC.nc"
static  /*inline*/  void HplMcs51GeneralIOC__P13__toggle(void )
//#line 97
{
  /* atomic removed: atomic calls only */
//#line 97
  {
//#line 97
    P1_3 = ~P1_3;
  }
}

//# 31 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
 /*inline*/  static void /*PlatformLedsC.Led2_rev*/ReverseGPIOP__1__In__toggle(void ){
//#line 31
  HplMcs51GeneralIOC__P13__toggle();
//#line 31
}
//#line 31
//# 59 "/opt/tinyos-2.x-contrib/diku/common/lib/ReverseGPIOP.nc"
static  /*inline*/  void /*PlatformLedsC.Led2_rev*/ReverseGPIOP__1__Out__toggle(void )
//#line 59
{
//#line 59
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
//#line 59
    {
//#line 59
      /*PlatformLedsC.Led2_rev*/ReverseGPIOP__1__In__toggle();
    }
//#line 60
    __nesc_atomic_end(__nesc_atomic); }
}

//# 31 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
 /*inline*/  static void LedsP__Led2__toggle(void ){
//#line 31
  /*PlatformLedsC.Led2_rev*/ReverseGPIOP__1__Out__toggle();
//#line 31
}
//#line 31
//# 103 "/opt/tinyos-2.x/tos/system/LedsP.nc"
static  /*inline*/  void LedsP__Leds__led2Toggle(void )
//#line 103
{
  LedsP__Led2__toggle();
  ;
//#line 105
  ;
}

//# 89 "/opt/tinyos-2.x/tos/interfaces/Leds.nc"
 /*inline*/  static void EchoP__Leds__led2Toggle(void ){
//#line 89
  LedsP__Leds__led2Toggle();
//#line 89
}
//#line 89
//# 25 "/opt/tinyos-2.x/tos/system/NoPinC.nc"
static  /*inline*/  void /*PlatformLedsC.NoPinC*/NoPinC__0__GeneralIO__toggle(void )
//#line 25
{
}

//# 31 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
 /*inline*/  static void LedsP__Led1__toggle(void ){
//#line 31
  /*PlatformLedsC.NoPinC*/NoPinC__0__GeneralIO__toggle();
//#line 31
}
//#line 31
//# 88 "/opt/tinyos-2.x/tos/system/LedsP.nc"
static  /*inline*/  void LedsP__Leds__led1Toggle(void )
//#line 88
{
  LedsP__Led1__toggle();
  ;
//#line 90
  ;
}

//# 72 "/opt/tinyos-2.x/tos/interfaces/Leds.nc"
 /*inline*/  static void EchoP__Leds__led1Toggle(void ){
//#line 72
  LedsP__Leds__led1Toggle();
//#line 72
}
//#line 72
//# 103 "/opt/tinyos-2.x-contrib/diku/mcs51/tos/chips/mcs51/pins/HplMcs51GeneralIOC.nc"
static  /*inline*/  void HplMcs51GeneralIOC__P20__toggle(void )
//#line 103
{
  /* atomic removed: atomic calls only */
//#line 103
  {
//#line 103
    P2_0 = ~P2_0;
  }
}

//# 31 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
 /*inline*/  static void /*PlatformLedsC.Led0_rev*/ReverseGPIOP__0__In__toggle(void ){
//#line 31
  HplMcs51GeneralIOC__P20__toggle();
//#line 31
}
//#line 31
//# 59 "/opt/tinyos-2.x-contrib/diku/common/lib/ReverseGPIOP.nc"
static  /*inline*/  void /*PlatformLedsC.Led0_rev*/ReverseGPIOP__0__Out__toggle(void )
//#line 59
{
//#line 59
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
//#line 59
    {
//#line 59
      /*PlatformLedsC.Led0_rev*/ReverseGPIOP__0__In__toggle();
    }
//#line 60
    __nesc_atomic_end(__nesc_atomic); }
}

//# 31 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
 /*inline*/  static void LedsP__Led0__toggle(void ){
//#line 31
  /*PlatformLedsC.Led0_rev*/ReverseGPIOP__0__Out__toggle();
//#line 31
}
//#line 31
//# 73 "/opt/tinyos-2.x/tos/system/LedsP.nc"
static  /*inline*/  void LedsP__Leds__led0Toggle(void )
//#line 73
{
  LedsP__Led0__toggle();
  ;
//#line 75
  ;
}

//# 56 "/opt/tinyos-2.x/tos/interfaces/Leds.nc"
 /*inline*/  static void EchoP__Leds__led0Toggle(void ){
//#line 56
  LedsP__Leds__led0Toggle();
//#line 56
}
//#line 56
//# 68 "EchoP.nc"
static  /*inline*/  void EchoP__uart0__get(uint8_t _data)
//#line 68
{
  EchoP__buf = _data;
  EchoP__Leds__led0Toggle();
  EchoP__Leds__led1Toggle();
  EchoP__Leds__led2Toggle();
  EchoP__print__postTask();

  return;
}

//# 47 "/opt/tinyos-2.x/tos/lib/serial/SerialByteComm.nc"
 /*inline*/  static void HalCC2430SimpleUartP__uart0__get(uint8_t _data){
//#line 47
  EchoP__uart0__get(_data);
//#line 47
}
//#line 47
//# 61 "EchoP.nc"
static  /*inline*/  void EchoP__uart0__putDone(void )
//#line 61
{
}

//# 53 "/opt/tinyos-2.x/tos/lib/serial/SerialByteComm.nc"
 /*inline*/  static void HalCC2430SimpleUartP__uart0__putDone(void ){
//#line 53
  EchoP__uart0__putDone();
//#line 53
}
//#line 53
//# 52 "/opt/tinyos-2.x/tos/system/RealMainP.nc"
  int main(void )
//#line 52
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    {





      {
      }
//#line 60
      ;

      RealMainP__Scheduler__init();





      RealMainP__PlatformInit__init();
      while (RealMainP__Scheduler__runNextTask()) ;





      RealMainP__SoftwareInit__init();
      while (RealMainP__Scheduler__runNextTask()) ;
    }
//#line 77
    __nesc_atomic_end(__nesc_atomic); }


  __nesc_enable_interrupt();

  RealMainP__Boot__booted();


  RealMainP__Scheduler__taskLoop();




  return -1;
}

//# 123 "/opt/tinyos-2.x/tos/system/SchedulerBasicP.nc"
static bool SchedulerBasicP__Scheduler__runNextTask(void )
{
  uint8_t nextTask;

  /* atomic removed: atomic calls only */
//#line 127
  {
    nextTask = SchedulerBasicP__popTask();
    if (nextTask == SchedulerBasicP__NO_TASK) 
      {
        {
          unsigned char __nesc_temp = 
//#line 131
          FALSE;

//#line 131
          return __nesc_temp;
        }
      }
  }
//#line 134
  SchedulerBasicP__TaskBasic__runTask(nextTask);
  return TRUE;
}

//# 135 "/opt/tinyos-2.x-contrib/diku/mcs51/tos/chips/cc2430/usart/HalCC2430SimpleUartP.nc"
void __vector_2(void) interrupt 2
//#line 135
{
  URX0IF = 0;
  HalCC2430SimpleUartP__uart0__get(U0BUF);
}

//#line 154
void __vector_7(void) interrupt 7
//#line 154
{
  int done = 0;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
//#line 157
    {



      if (!(U0CSR & 0x1)) {

          UTX0IF = 0;
          done = 1;
        }
    }
//#line 166
    __nesc_atomic_end(__nesc_atomic); }
  if (done) {
      HalCC2430SimpleUartP__uart0__putDone();
    }
}

