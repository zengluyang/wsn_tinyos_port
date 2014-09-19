
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
//# 11 "/opt/tinyos-2.x-contrib/diku/mcs51/tos/chips/mcs51/string.h"
extern void *memcpy(void *s1, void *s2, int n);

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
uint16_t TOS_NODE_ID = 1;





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

static  /*inline*/  error_t ecombine(error_t r1, error_t r2)  ;
//# 160 "/opt/tinyos-2.x-contrib/diku/mcs51/tos/chips/mcs51/io8051.h"
sbit EA = 0xAF;
//#line 194
sbit P1_0 = 0x90;


sbit P1_3 = 0x93;
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

sbit RFERRIF = 0x89;




sfr RFIM = 0x91;









sfr IEN2 = 0x9A;
sfr S1CON = 0x9B;
//#line 217
sbit RFERRIE = 0xA8;
sbit ADCIE = 0xA9;
sbit URX0IE = 0xAA;




sfr FWT = 0xAB;
sfr FADDRL = 0xAC;
sfr FADDRH = 0xAD;
//#line 246
sfr ADCCON1 = 0xB4;

sfr ADCCON3 = 0xB6;



sfr ADCL = 0xBA;
sfr ADCH = 0xBB;


sfr SLEEP = 0xBE;


sfr U0BUF = 0xC1;
sfr U0BAUD = 0xC2;

sfr U0UCR = 0xC4;
sfr U0GCR = 0xC5;
sfr CLKCON = 0xC6;
sfr MEMCTR = 0xC7;
//#line 289
sfr RFD = 0xD9;
//#line 308
sfr RFST = 0xE1;

sfr T1CNTL = 0xE2;
sfr T1CNTH = 0xE3;
//#line 334
sbit UTX0IF = 0xE9;



sfr RFIF = 0xE9;








sfr PERCFG = 0xF1;
sfr ADCCFG = 0xF2;



sfr P1_DIR = 0xFE;



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
//# 26 "/opt/tinyos-2.x-contrib/diku/common/lib/simplemac/packet.h"
typedef uint16_t mac_addr_t;
typedef uint8_t ieee_mac_addr_t[8];



enum fcf_stuff {
  FCF_FT_BEACON = 0x0000, 
  FCF_FT_DATA = 0x0001, 
  FCF_FT_ACK = 0x0002, 
  FCF_FT_MAC_COMMAND = 0x0003, 
  FCF_FT_MASK = 0x0007, 



  FCF_SECENC = 0x0008, 
  FCF_FRAMEPENDING = 0x0010, 
  FCF_ACKREQ = 0x0020, 
  FCF_INTRAPAN = 0x0040, 


  FCF_DST_NO_ADDR = 0x0000, 
  FCF_DST_SHORT_ADDR = 0x0800, 
  FCF_DST_LONG_ADDR = 0x0C00, 
  FCF_DST_ADDR_MASK = 0x0C00, 

  FCF_SRC_NO_ADDR = 0x0000, 
  FCF_SRC_SHORT_ADDR = 0x8000, 
  FCF_SRC_LONG_ADDR = 0xC000, 
  FCF_SRC_ADDR_MASK = 0xC000, 

  FCS_CRC_OK_MASK = 0x80, 
  FCS_CORRELATION_MASK = 0x7F
};




//#line 60
typedef struct __nesc_unnamed4273 {
  int8_t rssi;
  uint8_t correlation;
} fsc_t;
/*YYYYY } fsc_t*/
struct packet {

  uint8_t length;

  uint16_t fcf;
  uint8_t _data_seq_no;
  mac_addr_t dest;
  mac_addr_t src;

  uint8_t _data[122 - 2 * sizeof(mac_addr_t )];

  fsc_t fcs;
} 
/*__attribute((packed))*/ ;




typedef struct packet packet_t;
//# 40 "/opt/tinyos-2.x-contrib/diku/mcs51/tos/chips/cc2430/cc2420/cc2420.h"
typedef uint8_t cc2420_status_t;









//#line 42
typedef nx_struct cc2420_header_t {
  nxle_uint8_t length;
  nxle_uint16_t fcf;
  nxle_uint8_t dsn;
  nxle_uint16_t destpan;
  nxle_uint16_t dest;
  nxle_uint16_t src;
  nxle_uint8_t type;
} /*__attribute__((packed))*/ cc2420_header_t;



//#line 52
typedef nx_struct cc2420_footer_t {

  nxle_uint8_t i;
} /*__attribute__((packed))*/ cc2420_footer_t;







//#line 57
typedef nx_struct cc2420_meta_data_t {
  nx_uint8_t tx_power;
  nx_uint8_t rssi;
  nx_uint8_t lqi;
  nx_bool crc;
  nx_bool ack;
  nx_uint16_t time;
} /*__attribute__((packed))*/ cc2420_meta_data_t;




//#line 66
typedef nx_struct cc2420_packet_t {
  cc2420_header_t packet;

  nx_uint8_t _data[42];
} /*__attribute__((packed))*/ cc2420_packet_t;
enum __nesc_unnamed4274 {

  MAC_HEADER_SIZE = sizeof(cc2420_header_t ) - 1, 

  MAC_FOOTER_SIZE = sizeof(uint16_t ), 

  MAC_PACKET_SIZE = MAC_HEADER_SIZE + 28 + MAC_FOOTER_SIZE
};

enum cc2420_enums {
  CC2420_TIME_ACK_TURNAROUND = 7, 
  CC2420_TIME_VREN = 20, 
  CC2420_TIME_SYMBOL = 2, 
  CC2420_BACKOFF_PERIOD = 20 / CC2420_TIME_SYMBOL, 
  CC2420_ACK_WAIT_DELAY = 128
};

enum cc2420_status_enums {
  CC2420_STATUS_RSSI_VALID = 1 << 1, 
  CC2420_STATUS_LOCK = 1 << 2, 
  CC2420_STATUS_TX_ACTIVE = 1 << 3, 
  CC2420_STATUS_ENC_BUSY = 1 << 4, 
  CC2420_STATUS_TX_UNDERFLOW = 1 << 5, 
  CC2420_STATUS_XOSC16M_STABLE = 1 << 6
};

enum cc2420_config_reg_enums {
  CC2420_SNOP = 0x00, 
  CC2420_SXOSCON = 0x01, 
  CC2420_STXCAL = 0x02, 
  CC2420_SRXON = 0x03, 
  CC2420_STXON = 0x04, 
  CC2420_STXONCCA = 0x05, 
  CC2420_SRFOFF = 0x06, 
  CC2420_SXOSCOFF = 0x07, 
  CC2420_SFLUSHRX = 0x08, 
  CC2420_SFLUSHTX = 0x09, 
  CC2420_SACK = 0x0a, 
  CC2420_SACKPEND = 0x0b, 
  CC2420_SRXDEC = 0x0c, 
  CC2420_SRXENC = 0x0d, 
  CC2420_SAES = 0x0e, 
  CC2420_MAIN = 0x10, 
  CC2420_MDMCTRL0 = 0x11, 
  CC2420_MDMCTRL1 = 0x12, 
  CC2420_RSSI = 0x13, 
  CC2420_SYNCWORD = 0x14, 
  CC2420_TXCTRL = 0x15, 
  CC2420_RXCTRL0 = 0x16, 
  CC2420_RXCTRL1 = 0x17, 
  CC2420_FSCTRL = 0x18, 
  CC2420_SECCTRL0 = 0x19, 
  CC2420_SECCTRL1 = 0x1a, 
  CC2420_BATTMON = 0x1b, 
  CC2420_IOCFG0 = 0x1c, 
  CC2420_IOCFG1 = 0x1d, 
  CC2420_MANFIDL = 0x1e, 
  CC2420_MANFIDH = 0x1f, 
  CC2420_FSMTC = 0x20, 
  CC2420_MANAND = 0x21, 
  CC2420_MANOR = 0x22, 
  CC2420_AGCCTRL = 0x23, 
  CC2420_AGCTST0 = 0x24, 
  CC2420_AGCTST1 = 0x25, 
  CC2420_AGCTST2 = 0x26, 
  CC2420_FSTST0 = 0x27, 
  CC2420_FSTST1 = 0x28, 
  CC2420_FSTST2 = 0x29, 
  CC2420_FSTST3 = 0x2a, 
  CC2420_RXBPFTST = 0x2b, 
  CC2420_FMSTATE = 0x2c, 
  CC2420_ADCTST = 0x2d, 
  CC2420_DACTST = 0x2e, 
  CC2420_TOPTST = 0x2f, 
  CC2420_TXFIFO = 0x3e, 
  CC2420_RXFIFO = 0x3f
};

enum cc2420_ram_addr_enums {
  CC2420_RAM_TXFIFO = 0x000, 
  CC2420_RAM_RXFIFO = 0x080, 
  CC2420_RAM_KEY0 = 0x100, 
  CC2420_RAM_RXNONCE = 0x110, 
  CC2420_RAM_SABUF = 0x120, 
  CC2420_RAM_KEY1 = 0x130, 
  CC2420_RAM_TXNONCE = 0x140, 
  CC2420_RAM_CBCSTATE = 0x150, 
  CC2420_RAM_IEEEADR = 0x160, 
  CC2420_RAM_PANID = 0x168, 
  CC2420_RAM_SHORTADR = 0x16a
};

enum cc2420_nonce_enums {
  CC2420_NONCE_BLOCK_COUNTER = 0, 
  CC2420_NONCE_KEY_SEQ_COUNTER = 2, 
  CC2420_NONCE_FRAME_COUNTER = 3, 
  CC2420_NONCE_SOURCE_ADDRESS = 7, 
  CC2420_NONCE_FLAGS = 15
};

enum cc2420_main_enums {
  CC2420_MAIN_RESETn = 15, 
  CC2420_MAIN_ENC_RESETn = 14, 
  CC2420_MAIN_DEMOD_RESETn = 13, 
  CC2420_MAIN_MOD_RESETn = 12, 
  CC2420_MAIN_FS_RESETn = 11, 
  CC2420_MAIN_XOSC16M_BYPASS = 0
};

enum cc2420_mdmctrl0_enums {
  CC2420_MDMCTRL0_RESERVED_FRAME_MODE = 13, 
  CC2420_MDMCTRL0_PAN_COORDINATOR = 12, 
  CC2420_MDMCTRL0_ADR_DECODE = 11, 
  CC2420_MDMCTRL0_CCA_HYST = 8, 
  CC2420_MDMCTRL0_CCA_MOD = 6, 
  CC2420_MDMCTRL0_AUTOCRC = 5, 
  CC2420_MDMCTRL0_AUTOACK = 4, 
  CC2420_MDMCTRL0_PREAMBLE_LENGTH = 0
};

enum cc2420_mdmctrl1_enums {
  CC2420_MDMCTRL1_CORR_THR = 6, 
  CC2420_MDMCTRL1_DEMOD_AVG_MODE = 5, 
  CC2420_MDMCTRL1_MODULATION_MODE = 4, 
  CC2420_MDMCTRL1_TX_MODE = 2, 
  CC2420_MDMCTRL1_RX_MODE = 0
};

enum cc2420_rssi_enums {
  CC2420_RSSI_CCA_THR = 8, 
  CC2420_RSSI_RSSI_VAL = 0
};

enum cc2420_syncword_enums {
  CC2420_SYNCWORD_SYNCWORD = 0
};

enum cc2420_txctrl_enums {
  CC2420_TXCTRL_TXMIXBUF_CUR = 14, 
  CC2420_TXCTRL_TX_TURNAROUND = 13, 
  CC2420_TXCTRL_TXMIX_CAP_ARRAY = 11, 
  CC2420_TXCTRL_TXMIX_CURRENT = 9, 
  CC2420_TXCTRL_PA_CURRENT = 6, 
  CC2420_TXCTRL_RESERVED = 5, 
  CC2420_TXCTRL_PA_LEVEL = 0
};

enum cc2420_rxctrl0_enums {
  CC2420_RXCTRL0_RXMIXBUF_CUR = 12, 
  CC2420_RXCTRL0_HIGH_LNA_GAIN = 10, 
  CC2420_RXCTRL0_MED_LNA_GAIN = 8, 
  CC2420_RXCTRL0_LOW_LNA_GAIN = 6, 
  CC2420_RXCTRL0_HIGH_LNA_CURRENT = 4, 
  CC2420_RXCTRL0_MED_LNA_CURRENT = 2, 
  CC2420_RXCTRL0_LOW_LNA_CURRENT = 0
};

enum cc2420_rxctrl1_enums {
  CC2420_RXCTRL1_RXBPF_LOCUR = 13, 
  CC2420_RXCTRL1_RXBPF_MIDCUR = 12, 
  CC2420_RXCTRL1_LOW_LOWGAIN = 11, 
  CC2420_RXCTRL1_MED_LOWGAIN = 10, 
  CC2420_RXCTRL1_HIGH_HGM = 9, 
  CC2420_RXCTRL1_MED_HGM = 8, 
  CC2420_RXCTRL1_LNA_CAP_ARRAY = 6, 
  CC2420_RXCTRL1_RXMIX_TAIL = 4, 
  CC2420_RXCTRL1_RXMIX_VCM = 2, 
  CC2420_RXCTRL1_RXMIX_CURRENT = 0
};

enum cc2420_rsctrl_enums {
  CC2420_FSCTRL_LOCK_THR = 14, 
  CC2420_FSCTRL_CAL_DONE = 13, 
  CC2420_FSCTRL_CAL_RUNNING = 12, 
  CC2420_FSCTRL_LOCK_LENGTH = 11, 
  CC2420_FSCTRL_LOCK_STATUS = 10, 
  CC2420_FSCTRL_FREQ = 0
};

enum cc2420_secctrl0_enums {
  CC2420_SECCTRL0_RXFIFO_PROTECTION = 9, 
  CC2420_SECCTRL0_SEC_CBC_HEAD = 8, 
  CC2420_SECCTRL0_SEC_SAKEYSEL = 7, 
  CC2420_SECCTRL0_SEC_TXKEYSEL = 6, 
  CC2420_SECCTRL0_SEC_RXKEYSEL = 5, 
  CC2420_SECCTRL0_SEC_M = 2, 
  CC2420_SECCTRL0_SEC_MODE = 0
};

enum cc2420_secctrl1_enums {
  CC2420_SECCTRL1_SEC_TXL = 8, 
  CC2420_SECCTRL1_SEC_RXL = 0
};

enum cc2420_battmon_enums {
  CC2420_BATTMON_BATT_OK = 6, 
  CC2420_BATTMON_BATTMON_EN = 5, 
  CC2420_BATTMON_BATTMON_VOLTAGE = 0
};

enum cc2420_iocfg0_enums {
  CC2420_IOCFG0_BCN_ACCEPT = 11, 
  CC2420_IOCFG0_FIFO_POLARITY = 10, 
  CC2420_IOCFG0_FIFOP_POLARITY = 9, 
  CC2420_IOCFG0_SFD_POLARITY = 8, 
  CC2420_IOCFG0_CCA_POLARITY = 7, 
  CC2420_IOCFG0_FIFOP_THR = 0
};

enum cc2420_iocfg1_enums {
  CC2420_IOCFG1_HSSD_SRC = 10, 
  CC2420_IOCFG1_SFDMUX = 5, 
  CC2420_IOCFG1_CCAMUX = 0
};

enum cc2420_manfidl_enums {
  CC2420_MANFIDL_PARTNUM = 12, 
  CC2420_MANFIDL_MANFID = 0
};

enum cc2420_manfidh_enums {
  CC2420_MANFIDH_VERSION = 12, 
  CC2420_MANFIDH_PARTNUM = 0
};

enum cc2420_fsmtc_enums {
  CC2420_FSMTC_TC_RXCHAIN2RX = 13, 
  CC2420_FSMTC_TC_SWITCH2TX = 10, 
  CC2420_FSMTC_TC_PAON2TX = 6, 
  CC2420_FSMTC_TC_TXEND2SWITCH = 3, 
  CC2420_FSMTC_TC_TXEND2PAOFF = 0
};

enum cc2420_sfdmux_enums {
  CC2420_SFDMUX_SFD = 0, 
  CC2420_SFDMUX_XOSC16M_STABLE = 24
};
typedef int16_t TestAppP__Read__val_t;
typedef int16_t AdcP__Read__val_t;
enum /*TestAppC.AdcC*/AdcC__0____nesc_unnamed4275 {
  AdcC__0__ID = 0U
};
//# 51 "/opt/tinyos-2.x/tos/interfaces/Init.nc"
static error_t PlatformP__Init__init(void );
//# 56 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static error_t SchedulerBasicP__TaskBasic__postTask(
//# 45 "/opt/tinyos-2.x/tos/system/SchedulerBasicP.nc"
uint8_t arg_0x7ef73230);
//# 64 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static void SchedulerBasicP__TaskBasic__default__runTask(
//# 45 "/opt/tinyos-2.x/tos/system/SchedulerBasicP.nc"
uint8_t arg_0x7ef73230);
//# 46 "/opt/tinyos-2.x/tos/interfaces/Scheduler.nc"
static void SchedulerBasicP__Scheduler__init(void );
//#line 61
static void SchedulerBasicP__Scheduler__taskLoop(void );
//#line 54
static bool SchedulerBasicP__Scheduler__runNextTask(void );
//# 59 "/opt/tinyos-2.x/tos/interfaces/McuSleep.nc"
static void McuSleepC__McuSleep__sleep(void );
//# 49 "/opt/tinyos-2.x/tos/interfaces/Boot.nc"
static void TestAppP__Boot__booted(void );
//# 64 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static void TestAppP__sendPacketTask__runTask(void );
//# 63 "/opt/tinyos-2.x/tos/interfaces/Read.nc"
static void TestAppP__Read__readDone(error_t result, TestAppP__Read__val_t val);
//# 42 "/opt/tinyos-2.x-contrib/diku/common/lib/simplemac/SimpleMac.nc"
static void TestAppP__SimpleMac__sendPacketDone(packet_t *packet, error_t result);









static packet_t *TestAppP__SimpleMac__receivedPacket(packet_t *packet);
//# 51 "/opt/tinyos-2.x/tos/interfaces/Init.nc"
static error_t TestAppP__Init__init(void );
//# 64 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static void TestAppP__consoleTask__runTask(void );
//# 108 "/opt/tinyos-2.x-contrib/diku/common/lib/stdout/StdOut.nc"
static void TestAppP__StdOut__get(uint8_t _data);
//# 51 "/opt/tinyos-2.x/tos/interfaces/Init.nc"
static error_t LedsP__Init__init(void );
//# 50 "/opt/tinyos-2.x/tos/interfaces/Leds.nc"
static void LedsP__Leds__led0Off(void );





static void LedsP__Leds__led0Toggle(void );




static void LedsP__Leds__led1On(void );




static void LedsP__Leds__led1Off(void );
//#line 83
static void LedsP__Leds__led2Off(void );
//#line 45
static void LedsP__Leds__led0On(void );
//#line 78
static void LedsP__Leds__led2On(void );
//# 35 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
static void HplMcs51GeneralIOC__P13__makeOutput(void );
//#line 29
static void HplMcs51GeneralIOC__P13__set(void );
static void HplMcs51GeneralIOC__P13__clr(void );
static void HplMcs51GeneralIOC__P10__toggle(void );



static void HplMcs51GeneralIOC__P10__makeOutput(void );
//#line 29
static void HplMcs51GeneralIOC__P10__set(void );
static void HplMcs51GeneralIOC__P10__clr(void );
static void /*PlatformLedsC.Led0_rev*/ReverseGPIOP__0__Out__toggle(void );



static void /*PlatformLedsC.Led0_rev*/ReverseGPIOP__0__Out__makeOutput(void );
//#line 29
static void /*PlatformLedsC.Led0_rev*/ReverseGPIOP__0__Out__set(void );
static void /*PlatformLedsC.Led0_rev*/ReverseGPIOP__0__Out__clr(void );




static void /*PlatformLedsC.Led2_rev*/ReverseGPIOP__1__Out__makeOutput(void );
//#line 29
static void /*PlatformLedsC.Led2_rev*/ReverseGPIOP__1__Out__set(void );
static void /*PlatformLedsC.Led2_rev*/ReverseGPIOP__1__Out__clr(void );




static void /*PlatformLedsC.NoPinC*/NoPinC__0__GeneralIO__makeOutput(void );
//#line 29
static void /*PlatformLedsC.NoPinC*/NoPinC__0__GeneralIO__set(void );
static void /*PlatformLedsC.NoPinC*/NoPinC__0__GeneralIO__clr(void );
//# 51 "/opt/tinyos-2.x/tos/interfaces/Init.nc"
static error_t StdOutM__Init__init(void );
//# 47 "/opt/tinyos-2.x/tos/lib/serial/SerialByteComm.nc"
static void StdOutM__UART__get(uint8_t _data);





static void StdOutM__UART__putDone(void );
//# 56 "/opt/tinyos-2.x-contrib/diku/common/lib/stdout/StdOut.nc"
static void StdOutM__StdOut__dumpHex(uint8_t ptr[], uint8_t count, char *sep);
//#line 71
static int StdOutM__StdOut__printBase10uint16(const uint16_t c);
//#line 44
static int StdOutM__StdOut__printHexword(uint16_t c);
//#line 61
static int StdOutM__StdOut__printBase10uint8(const uint8_t c);
//#line 34
static int StdOutM__StdOut__print(const char *str);




static int StdOutM__StdOut__printHex(uint8_t c);
//#line 76
static int StdOutM__StdOut__printBase10int16(const int16_t c);
//# 51 "/opt/tinyos-2.x/tos/interfaces/Init.nc"
static error_t HalCC2430SimpleUartP__Init__init(void );
//# 41 "/opt/tinyos-2.x/tos/lib/serial/SerialByteComm.nc"
static error_t HalCC2430SimpleUartP__uart0__put(uint8_t _data);
//# 62 "/opt/tinyos-2.x-contrib/diku/common/lib/simplemac/SimpleMac.nc"
static error_t SimpleMacM__SimpleMac__rxDisable(void );
//#line 54
static error_t SimpleMacM__SimpleMac__setChannel(uint8_t channel);
//#line 34
static error_t SimpleMacM__SimpleMac__sendPacket(packet_t *packet);
//#line 61
static error_t SimpleMacM__SimpleMac__rxEnable(void );
//# 51 "/opt/tinyos-2.x/tos/interfaces/Init.nc"
static error_t SimpleMacM__Init__init(void );
//# 64 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static void SimpleMacM__initTask__runTask(void );
//# 74 "/opt/tinyos-2.x/tos/interfaces/StdControl.nc"
static error_t SimpleMacM__SimpleMacControl__start(void );









static error_t SimpleMacM__SimpleMacControl__stop(void );
//# 43 "/opt/tinyos-2.x-contrib/diku/common/lib/simplemac/HALCC2420.nc"
static void SimpleMacM__HALCC2420__sendPacketDone(uint8_t *packet, error_t result);









static uint8_t *SimpleMacM__HALCC2420__receivedPacket(uint8_t *packet);
//# 64 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static void SimpleMacM__signalSendPacketDone__runTask(void );
//#line 64
static void HalCC2430RadioP__receivedPacketTask__runTask(void );
//#line 64
static void HalCC2430RadioP__sendPacketDoneTask__runTask(void );
//# 57 "/opt/tinyos-2.x/tos/interfaces/GpioInterrupt.nc"
static void HalCC2430RadioP__InterruptRFErr__fired(void );
//# 74 "/opt/tinyos-2.x/tos/interfaces/StdControl.nc"
static error_t HalCC2430RadioP__HALCC2420Control__start(void );









static error_t HalCC2430RadioP__HALCC2420Control__stop(void );
//# 57 "/opt/tinyos-2.x/tos/interfaces/GpioInterrupt.nc"
static void HalCC2430RadioP__InterruptTXDone__fired(void );
//# 51 "/opt/tinyos-2.x/tos/interfaces/Init.nc"
static error_t HalCC2430RadioP__Init__init(void );
//# 64 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static void HalCC2430RadioP__initTask__runTask(void );
//# 63 "/opt/tinyos-2.x-contrib/diku/common/lib/simplemac/HALCC2420.nc"
static error_t HalCC2430RadioP__HALCC2420__rxDisable(void );
//#line 59
static const mac_addr_t *HalCC2430RadioP__HALCC2420__getAddress(void );
//#line 55
static error_t HalCC2430RadioP__HALCC2420__setChannel(uint8_t channel);
//#line 35
static error_t HalCC2430RadioP__HALCC2420__sendPacket(uint8_t *packet);
//#line 62
static error_t HalCC2430RadioP__HALCC2420__rxEnable(void );
//# 64 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static void HalCC2430RadioP__flushBufferTask__runTask(void );
//# 108 "/opt/tinyos-2.x-contrib/diku/common/lib/stdout/StdOut.nc"
static void HalCC2430RadioP__StdOut__get(uint8_t _data);
//# 57 "/opt/tinyos-2.x/tos/interfaces/GpioInterrupt.nc"
static void HalCC2430RadioP__InterruptFIFOP__fired(void );
//# 64 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static void HalCC2430RadioP__setChannelTask__runTask(void );
//#line 64
static void HalCC2430RadioP__transmitTask__runTask(void );
//# 50 "/opt/tinyos-2.x/tos/interfaces/GpioCapture.nc"
static void HplCC2430InterruptsC__CaptureSFD__default__captured(uint16_t time);
//# 42 "/opt/tinyos-2.x/tos/interfaces/GpioInterrupt.nc"
static error_t HplCC2430InterruptsC__InterruptRFErr__enableRisingEdge(void );
//#line 42
static error_t HplCC2430InterruptsC__InterruptTXDone__enableRisingEdge(void );
//#line 57
static void HplCC2430InterruptsC__InterruptCCA__default__fired(void );
//#line 50
static error_t HplCC2430InterruptsC__InterruptFIFOP__disable(void );
//#line 42
static error_t HplCC2430InterruptsC__InterruptFIFOP__enableRisingEdge(void );
//# 22 "/opt/tinyos-2.x-contrib/diku/common/tos/interfaces/HalFlash.nc"
static error_t HalFlashP__HalFlash__read(uint8_t *destination, uint8_t *source, uint16_t length);

static error_t HalFlashP__HalFlash__erase(uint8_t *address);
//#line 23
static error_t HalFlashP__HalFlash__write(uint8_t *source, uint8_t *destination, uint16_t length);
//# 51 "/opt/tinyos-2.x/tos/interfaces/Init.nc"
static error_t HalFlashP__Init__init(void );
//# 66 "/opt/tinyos-2.x-contrib/diku/mcs51/tos/chips/cc2430/adc/AdcControl.nc"
static void AdcP__AdcControl__enable(
//# 42 "/opt/tinyos-2.x-contrib/diku/mcs51/tos/chips/cc2430/adc/AdcP.nc"
uint8_t arg_0x7e700818, 
//# 66 "/opt/tinyos-2.x-contrib/diku/mcs51/tos/chips/cc2430/adc/AdcControl.nc"
uint8_t reference, uint8_t resolution, uint8_t input);
//# 55 "/opt/tinyos-2.x/tos/interfaces/Read.nc"
static error_t AdcP__Read__read(
//# 43 "/opt/tinyos-2.x-contrib/diku/mcs51/tos/chips/cc2430/adc/AdcP.nc"
uint8_t arg_0x7e6f8030);
//# 63 "/opt/tinyos-2.x/tos/interfaces/Read.nc"
static void AdcP__Read__default__readDone(
//# 43 "/opt/tinyos-2.x-contrib/diku/mcs51/tos/chips/cc2430/adc/AdcP.nc"
uint8_t arg_0x7e6f8030, 
//# 63 "/opt/tinyos-2.x/tos/interfaces/Read.nc"
error_t result, AdcP__Read__val_t val);
//# 51 "/opt/tinyos-2.x/tos/interfaces/Init.nc"
static error_t AdcP__Init__init(void );
//# 64 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static void AdcP__signalReadDone__runTask(void );
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
uint8_t arg_0x7ef73230);
//# 59 "/opt/tinyos-2.x/tos/interfaces/McuSleep.nc"
static void SchedulerBasicP__McuSleep__sleep(void );
//# 50 "/opt/tinyos-2.x/tos/system/SchedulerBasicP.nc"
enum SchedulerBasicP____nesc_unnamed4276 {

  SchedulerBasicP__NUM_TASKS = 11U, 
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
static error_t SchedulerBasicP__TaskBasic__postTask(uint8_t id);




static void SchedulerBasicP__TaskBasic__default__runTask(uint8_t id);
//# 51 "/opt/tinyos-2.x-contrib/diku/mcs51/tos/chips/cc2430/McuSleepC.nc"
static  /*inline*/  void McuSleepC__McuSleep__sleep(void );
//# 22 "/opt/tinyos-2.x-contrib/diku/common/tos/interfaces/HalFlash.nc"
static error_t TestAppP__HalFlash__read(uint8_t *destination, uint8_t *source, uint16_t length);

static error_t TestAppP__HalFlash__erase(uint8_t *address);
//#line 23
static error_t TestAppP__HalFlash__write(uint8_t *source, uint8_t *destination, uint16_t length);
//# 66 "/opt/tinyos-2.x-contrib/diku/mcs51/tos/chips/cc2430/adc/AdcControl.nc"
static void TestAppP__AdcControl__enable(uint8_t reference, uint8_t resolution, uint8_t input);
//# 55 "/opt/tinyos-2.x/tos/interfaces/Read.nc"
static error_t TestAppP__Read__read(void );
//# 62 "/opt/tinyos-2.x-contrib/diku/common/lib/simplemac/SimpleMac.nc"
static error_t TestAppP__SimpleMac__rxDisable(void );
//#line 54
static error_t TestAppP__SimpleMac__setChannel(uint8_t channel);
//#line 34
static error_t TestAppP__SimpleMac__sendPacket(packet_t *packet);
//#line 61
static error_t TestAppP__SimpleMac__rxEnable(void );
//# 56 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static error_t TestAppP__consoleTask__postTask(void );
//# 74 "/opt/tinyos-2.x/tos/interfaces/StdControl.nc"
static error_t TestAppP__SimpleMacControl__start(void );









static error_t TestAppP__SimpleMacControl__stop(void );
//# 50 "/opt/tinyos-2.x/tos/interfaces/Leds.nc"
static void TestAppP__Leds__led0Off(void );





static void TestAppP__Leds__led0Toggle(void );




static void TestAppP__Leds__led1On(void );




static void TestAppP__Leds__led1Off(void );
//#line 83
static void TestAppP__Leds__led2Off(void );
//#line 45
static void TestAppP__Leds__led0On(void );
//#line 78
static void TestAppP__Leds__led2On(void );
//# 56 "/opt/tinyos-2.x-contrib/diku/common/lib/stdout/StdOut.nc"
static void TestAppP__StdOut__dumpHex(uint8_t ptr[], uint8_t count, char *sep);
//#line 44
static int TestAppP__StdOut__printHexword(uint16_t c);
//#line 61
static int TestAppP__StdOut__printBase10uint8(const uint8_t c);
//#line 34
static int TestAppP__StdOut__print(const char *str);




static int TestAppP__StdOut__printHex(uint8_t c);
//#line 76
static int TestAppP__StdOut__printBase10int16(const int16_t c);
//# 89 "TestAppP.nc"
enum TestAppP____nesc_unnamed4277 {
//#line 89
  TestAppP__sendPacketTask = 0U
};
//#line 89
typedef int TestAppP____nesc_sillytask_sendPacketTask[];
//#line 243
enum TestAppP____nesc_unnamed4278 {
//#line 243
  TestAppP__consoleTask = 1U
};
//#line 243
typedef int TestAppP____nesc_sillytask_consoleTask[TestAppP__consoleTask];
//#line 81
mac_addr_t TestAppP__shortAddress;
uint8_t TestAppP__transmitPacket[128];
packet_t *TestAppP__transmitPacketPtr;

bool TestAppP__radioOn = FALSE;
//#line 85
bool TestAppP__receiverOn = FALSE;
uint8_t TestAppP__channel;
uint8_t TestAppP__sequence = 1;



uint8_t TestAppP__source[256];
uint8_t TestAppP__destination[256];

bool TestAppP__ledOn = FALSE;




static  /*inline*/  error_t TestAppP__Init__init(void );
//#line 130
static  /*inline*/  void TestAppP__Boot__booted(void );
//#line 156
uint8_t TestAppP__counter = 0;


static  /*inline*/  void TestAppP__Read__readDone(error_t result, int16_t val);
//#line 186
static  /*inline*/  void TestAppP__sendPacketTask__runTask(void );






static  /*inline*/  void TestAppP__SimpleMac__sendPacketDone(packet_t *packet, error_t result);
//#line 207
static  /*inline*/  packet_t *TestAppP__SimpleMac__receivedPacket(packet_t *packet);
//#line 241
uint8_t TestAppP__keyBuffer;



static  /*inline*/  void TestAppP__StdOut__get(uint8_t _data);








static  /*inline*/  void TestAppP__consoleTask__runTask(void );
//# 31 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
static void LedsP__Led0__toggle(void );



static void LedsP__Led0__makeOutput(void );
//#line 29
static void LedsP__Led0__set(void );
static void LedsP__Led0__clr(void );




static void LedsP__Led1__makeOutput(void );
//#line 29
static void LedsP__Led1__set(void );
static void LedsP__Led1__clr(void );




static void LedsP__Led2__makeOutput(void );
//#line 29
static void LedsP__Led2__set(void );
static void LedsP__Led2__clr(void );
//# 45 "/opt/tinyos-2.x/tos/system/LedsP.nc"
static  /*inline*/  error_t LedsP__Init__init(void );
//#line 63
static  /*inline*/  void LedsP__Leds__led0On(void );




static  /*inline*/  void LedsP__Leds__led0Off(void );




static  /*inline*/  void LedsP__Leds__led0Toggle(void );




static  /*inline*/  void LedsP__Leds__led1On(void );




static  /*inline*/  void LedsP__Leds__led1Off(void );









static  /*inline*/  void LedsP__Leds__led2On(void );




static  /*inline*/  void LedsP__Leds__led2Off(void );
//# 92 "/opt/tinyos-2.x-contrib/diku/mcs51/tos/chips/mcs51/pins/HplMcs51GeneralIOC.nc"
static   /*inline*/   void HplMcs51GeneralIOC__P10__set(void );
//#line 92
static   /*inline*/   void HplMcs51GeneralIOC__P10__clr(void );
//#line 92
static  /*inline*/  void HplMcs51GeneralIOC__P10__toggle(void );
//#line 92
static   /*inline*/   void HplMcs51GeneralIOC__P10__makeOutput(void );


static   /*inline*/   void HplMcs51GeneralIOC__P13__set(void );
//#line 95
static   /*inline*/   void HplMcs51GeneralIOC__P13__clr(void );
//#line 95
static   /*inline*/   void HplMcs51GeneralIOC__P13__makeOutput(void );
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
//# 35 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
static void /*PlatformLedsC.Led2_rev*/ReverseGPIOP__1__In__makeOutput(void );
//#line 29
static void /*PlatformLedsC.Led2_rev*/ReverseGPIOP__1__In__set(void );
static void /*PlatformLedsC.Led2_rev*/ReverseGPIOP__1__In__clr(void );
//# 59 "/opt/tinyos-2.x-contrib/diku/common/lib/ReverseGPIOP.nc"
static   /*inline*/   void /*PlatformLedsC.Led2_rev*/ReverseGPIOP__1__Out__set(void );
//#line 59
static   /*inline*/   void /*PlatformLedsC.Led2_rev*/ReverseGPIOP__1__Out__clr(void );
//#line 59
static   /*inline*/   void /*PlatformLedsC.Led2_rev*/ReverseGPIOP__1__Out__makeOutput(void );
//# 23 "/opt/tinyos-2.x/tos/system/NoPinC.nc"
static  /*inline*/  void /*PlatformLedsC.NoPinC*/NoPinC__0__GeneralIO__set(void );
static  /*inline*/  void /*PlatformLedsC.NoPinC*/NoPinC__0__GeneralIO__clr(void );


static  /*inline*/  void /*PlatformLedsC.NoPinC*/NoPinC__0__GeneralIO__makeOutput(void );
//# 41 "/opt/tinyos-2.x/tos/lib/serial/SerialByteComm.nc"
static error_t StdOutM__UART__put(uint8_t _data);
//# 108 "/opt/tinyos-2.x-contrib/diku/common/lib/stdout/StdOut.nc"
static void StdOutM__StdOut__get(uint8_t _data);
//# 47 "/opt/tinyos-2.x-contrib/diku/common/lib/stdout/StdOutM.nc"
char StdOutM__buffer[1000];
char *StdOutM__bufferhead;
char *StdOutM__buffertail;
char *StdOutM__bufferend;
int StdOutM__isOutputting;

int StdOutM__count;


static  /*inline*/  error_t StdOutM__Init__init(void );
//#line 77
static int StdOutM__StdOut__print(const char *str);
//#line 155
static int StdOutM__StdOut__printHex(uint8_t c);
//#line 181
static int StdOutM__StdOut__printHexword(uint16_t c);
//#line 196
static int StdOutM__StdOut__printBase10uint8(const uint8_t c);
//#line 250
static  /*inline*/  int StdOutM__StdOut__printBase10uint16(const uint16_t c);
//#line 305
static  /*inline*/  int StdOutM__StdOut__printBase10int16(const int16_t c);
//#line 507
static  /*inline*/  void StdOutM__StdOut__dumpHex(uint8_t ptr[], uint8_t countar, char *sep);
//#line 520
static  /*inline*/  void StdOutM__UART__putDone(void );
//#line 545
static  /*inline*/  void StdOutM__UART__get(uint8_t _data);
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
//# 42 "/opt/tinyos-2.x-contrib/diku/common/lib/simplemac/SimpleMac.nc"
static void SimpleMacM__SimpleMac__sendPacketDone(packet_t *packet, error_t result);









static packet_t *SimpleMacM__SimpleMac__receivedPacket(packet_t *packet);
//# 74 "/opt/tinyos-2.x/tos/interfaces/StdControl.nc"
static error_t SimpleMacM__HALCC2420Control__start(void );









static error_t SimpleMacM__HALCC2420Control__stop(void );
//# 56 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static error_t SimpleMacM__initTask__postTask(void );
//# 63 "/opt/tinyos-2.x-contrib/diku/common/lib/simplemac/HALCC2420.nc"
static error_t SimpleMacM__HALCC2420__rxDisable(void );
//#line 59
static const mac_addr_t *SimpleMacM__HALCC2420__getAddress(void );
//#line 55
static error_t SimpleMacM__HALCC2420__setChannel(uint8_t channel);
//#line 35
static error_t SimpleMacM__HALCC2420__sendPacket(uint8_t *packet);
//#line 62
static error_t SimpleMacM__HALCC2420__rxEnable(void );
//# 56 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static error_t SimpleMacM__signalSendPacketDone__postTask(void );
//# 45 "/opt/tinyos-2.x-contrib/diku/common/lib/simplemac/SimpleMacM.nc"
enum SimpleMacM____nesc_unnamed4279 {
//#line 45
  SimpleMacM__initTask = 2U
};
//#line 45
typedef int SimpleMacM____nesc_sillytask_initTask[SimpleMacM__initTask];
//#line 123
enum SimpleMacM____nesc_unnamed4280 {
//#line 123
  SimpleMacM__signalSendPacketDone = 3U
};
//#line 123
typedef int SimpleMacM____nesc_sillytask_signalSendPacketDone[SimpleMacM__signalSendPacketDone];
//#line 36
packet_t SimpleMacM__receivedPacket;
packet_t *SimpleMacM__receivedPacketPtr;

const mac_addr_t *SimpleMacM__shortAddress;









static  /*inline*/  error_t SimpleMacM__Init__init(void );







static  /*inline*/  void SimpleMacM__initTask__runTask(void );







static  /*inline*/  error_t SimpleMacM__SimpleMacControl__start(void );




static  /*inline*/  error_t SimpleMacM__SimpleMacControl__stop(void );
//#line 82
uint8_t SimpleMacM__transmitbuffer[128];
packet_t *SimpleMacM__sendPacketPtr;

static error_t SimpleMacM__SimpleMac__sendPacket(packet_t *packet);
//#line 121
error_t SimpleMacM__sendPacketResult;

static  /*inline*/  void SimpleMacM__signalSendPacketDone__runTask(void );








static  /*inline*/  void SimpleMacM__HALCC2420__sendPacketDone(uint8_t *packet, error_t result);









static  /*inline*/  error_t SimpleMacM__SimpleMac__setChannel(uint8_t channel);
//#line 160
static  /*inline*/  error_t SimpleMacM__SimpleMac__rxEnable(void );








static  /*inline*/  error_t SimpleMacM__SimpleMac__rxDisable(void );







static  /*inline*/  uint8_t *SimpleMacM__HALCC2420__receivedPacket(uint8_t *packet);
//# 56 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static error_t HalCC2430RadioP__receivedPacketTask__postTask(void );
//#line 56
static error_t HalCC2430RadioP__sendPacketDoneTask__postTask(void );
//# 42 "/opt/tinyos-2.x/tos/interfaces/GpioInterrupt.nc"
static error_t HalCC2430RadioP__InterruptRFErr__enableRisingEdge(void );
//#line 42
static error_t HalCC2430RadioP__InterruptTXDone__enableRisingEdge(void );
//# 56 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static error_t HalCC2430RadioP__initTask__postTask(void );
//# 43 "/opt/tinyos-2.x-contrib/diku/common/lib/simplemac/HALCC2420.nc"
static void HalCC2430RadioP__HALCC2420__sendPacketDone(uint8_t *packet, error_t result);









static uint8_t *HalCC2430RadioP__HALCC2420__receivedPacket(uint8_t *packet);
//# 56 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static error_t HalCC2430RadioP__flushBufferTask__postTask(void );
//# 34 "/opt/tinyos-2.x-contrib/diku/common/lib/stdout/StdOut.nc"
static int HalCC2430RadioP__StdOut__print(const char *str);
//# 50 "/opt/tinyos-2.x/tos/interfaces/GpioInterrupt.nc"
static error_t HalCC2430RadioP__InterruptFIFOP__disable(void );
//#line 42
static error_t HalCC2430RadioP__InterruptFIFOP__enableRisingEdge(void );
//# 56 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static error_t HalCC2430RadioP__setChannelTask__postTask(void );
//#line 56
static error_t HalCC2430RadioP__transmitTask__postTask(void );
//# 81 "/opt/tinyos-2.x-contrib/diku/mcs51/tos/chips/cc2430/radio/HalCC2430RadioP.nc"
enum HalCC2430RadioP____nesc_unnamed4281 {
//#line 81
  HalCC2430RadioP__initTask = 4U
};
//#line 81
typedef int HalCC2430RadioP____nesc_sillytask_initTask[HalCC2430RadioP__initTask];
enum HalCC2430RadioP____nesc_unnamed4282 {
//#line 82
  HalCC2430RadioP__transmitTask = 5U
};
//#line 82
typedef int HalCC2430RadioP____nesc_sillytask_transmitTask[HalCC2430RadioP__transmitTask];
enum HalCC2430RadioP____nesc_unnamed4283 {
//#line 83
  HalCC2430RadioP__setChannelTask = 6U
};
//#line 83
typedef int HalCC2430RadioP____nesc_sillytask_setChannelTask[HalCC2430RadioP__setChannelTask];
//#line 472
enum HalCC2430RadioP____nesc_unnamed4284 {
//#line 472
  HalCC2430RadioP__receivedPacketTask = 7U
};
//#line 472
typedef int HalCC2430RadioP____nesc_sillytask_receivedPacketTask[HalCC2430RadioP__receivedPacketTask];
enum HalCC2430RadioP____nesc_unnamed4285 {
//#line 473
  HalCC2430RadioP__flushBufferTask = 8U
};
//#line 473
typedef int HalCC2430RadioP____nesc_sillytask_flushBufferTask[HalCC2430RadioP__flushBufferTask];
enum HalCC2430RadioP____nesc_unnamed4286 {
//#line 474
  HalCC2430RadioP__sendPacketDoneTask = 9U
};
//#line 474
typedef int HalCC2430RadioP____nesc_sillytask_sendPacketDoneTask[HalCC2430RadioP__sendPacketDoneTask];
//# 58 "/opt/tinyos-2.x-contrib/diku/common/lib/simplemac/hplcc2420.h"
//#line 4
typedef enum HalCC2430RadioP____nesc_unnamed4287 {

  HalCC2430RadioP__CC_REG_SNOP = 0x00, 
  HalCC2430RadioP__CC_REG_SXOSCON = 0x01, 
  HalCC2430RadioP__CC_REG_STXCAL = 0x02, 
  HalCC2430RadioP__CC_REG_SRXON = 0x03, 
  HalCC2430RadioP__CC_REG_STXON = 0x04, 
  HalCC2430RadioP__CC_REG_STXONCCA = 0x05, 
  HalCC2430RadioP__CC_REG_SRFOFF = 0x06, 
  HalCC2430RadioP__CC_REG_SXOSCOFF = 0x07, 
  HalCC2430RadioP__CC_REG_SFLUSHRX = 0x08, 
  HalCC2430RadioP__CC_REG_SFLUSHTX = 0x09, 
  HalCC2430RadioP__CC_REG_SACK = 0x0A, 
  HalCC2430RadioP__CC_REG_SACKPEND = 0x0B, 
  HalCC2430RadioP__CC_REG_SRXDEC = 0x0C, 
  HalCC2430RadioP__CC_REG_STXENC = 0x0D, 
  HalCC2430RadioP__CC_REG_SAES = 0x0E, 

  HalCC2430RadioP__CC_REG_MAIN = 0x10, 
  HalCC2430RadioP__CC_REG_MDMCTRL0 = 0x11, 
  HalCC2430RadioP__CC_REG_MDMCTRL1 = 0x12, 
  HalCC2430RadioP__CC_REG_RSSI = 0x13, 
  HalCC2430RadioP__CC_REG_SYNCWORD = 0x14, 
  HalCC2430RadioP__CC_REG_TXCTRL = 0x15, 
  HalCC2430RadioP__CC_REG_RXCTRL0 = 0x16, 
  HalCC2430RadioP__CC_REG_RXCTRL1 = 0x17, 
  HalCC2430RadioP__CC_REG_FSCTRL = 0x18, 
  HalCC2430RadioP__CC_REG_SECCTRL0 = 0x19, 
  HalCC2430RadioP__CC_REG_SECCTRL1 = 0x1A, 
  HalCC2430RadioP__CC_REG_BATTMON = 0x1B, 
  HalCC2430RadioP__CC_REG_IOCFG0 = 0x1C, 
  HalCC2430RadioP__CC_REG_IOCFG1 = 0x1D, 
  HalCC2430RadioP__CC_REG_MANFIDL = 0x1E, 
  HalCC2430RadioP__CC_REG_MANFIDH = 0x1F, 
  HalCC2430RadioP__CC_REG_FSMTC = 0x20, 
  HalCC2430RadioP__CC_REG_MANAND = 0x21, 
  HalCC2430RadioP__CC_REG_MANOR = 0x22, 
  HalCC2430RadioP__CC_REG_AGCCTRL = 0x23, 
  HalCC2430RadioP__CC_REG_AGCTST0 = 0x24, 
  HalCC2430RadioP__CC_REG_AGCTST1 = 0x25, 
  HalCC2430RadioP__CC_REG_AGCTST2 = 0x26, 
  HalCC2430RadioP__CC_REG_FSTST0 = 0x27, 
  HalCC2430RadioP__CC_REG_FSTST1 = 0x28, 
  HalCC2430RadioP__CC_REG_FSTST2 = 0x29, 
  HalCC2430RadioP__CC_REG_FSTST3 = 0x2A, 
  HalCC2430RadioP__CC_REG_RXBPFTST = 0x2B, 
  HalCC2430RadioP__CC_REG_FSMSTATE = 0x2C, 
  HalCC2430RadioP__CC_REG_ADCTST = 0x2D, 
  HalCC2430RadioP__CC_REG_DACTST = 0x2E, 
  HalCC2430RadioP__CC_REG_TOPTST = 0x2F, 
  HalCC2430RadioP__CC_REG_RESERVED = 0x30, 

  HalCC2430RadioP__CC_REG_TXFIFO = 0x3E, 
  HalCC2430RadioP__CC_REG_RXFIFO = 0x3F
} HalCC2430RadioP__cc2420_reg_t;
//#line 73
//#line 60
typedef enum HalCC2430RadioP____nesc_unnamed4288 {

  HalCC2430RadioP__CC_ADDR_TXFIFO = 0x000, 
  HalCC2430RadioP__CC_ADDR_RXFIFO = 0x080, 
  HalCC2430RadioP__CC_ADDR_KEY0 = 0x100, 
  HalCC2430RadioP__CC_ADDR_RXNONCE = 0x110, 
  HalCC2430RadioP__CC_ADDR_SABUF = 0x120, 
  HalCC2430RadioP__CC_ADDR_KEY1 = 0x130, 
  HalCC2430RadioP__CC_ADDR_TXNONCE = 0x140, 
  HalCC2430RadioP__CC_ADDR_CBCSTATE = 0x150, 
  HalCC2430RadioP__CC_ADDR_IEEEADDR = 0x160, 
  HalCC2430RadioP__CC_ADDR_PANID = 0x168, 
  HalCC2430RadioP__CC_ADDR_SHORTADDR = 0x16A
} HalCC2430RadioP__cc2420_addr_t;
//#line 125
//#line 115
typedef enum HalCC2430RadioP____nesc_unnamed4289 {

  HalCC2430RadioP__CCA_HYST_0DB = 0x00, 
  HalCC2430RadioP__CCA_HYST_1DB = 0x01, 
  HalCC2430RadioP__CCA_HYST_2DB = 0x02, 
  HalCC2430RadioP__CCA_HYST_3DB = 0x03, 
  HalCC2430RadioP__CCA_HYST_4DB = 0x04, 
  HalCC2430RadioP__CCA_HYST_5DB = 0x05, 
  HalCC2430RadioP__CCA_HYST_6DB = 0x06, 
  HalCC2430RadioP__CCA_HYST_7DB = 0x07
} HalCC2430RadioP__cca_hyst_db_t;
//#line 145
//#line 127
typedef enum HalCC2430RadioP____nesc_unnamed4290 {

  HalCC2430RadioP__LEADING_ZERO_BYTES_1 = 0x00, 
  HalCC2430RadioP__LEADING_ZERO_BYTES_2 = 0x01, 
  HalCC2430RadioP__LEADING_ZERO_BYTES_3 = 0x02, 
  HalCC2430RadioP__LEADING_ZERO_BYTES_4 = 0x03, 
  HalCC2430RadioP__LEADING_ZERO_BYTES_5 = 0x04, 
  HalCC2430RadioP__LEADING_ZERO_BYTES_6 = 0x05, 
  HalCC2430RadioP__LEADING_ZERO_BYTES_7 = 0x06, 
  HalCC2430RadioP__LEADING_ZERO_BYTES_8 = 0x07, 
  HalCC2430RadioP__LEADING_ZERO_BYTES_9 = 0x08, 
  HalCC2430RadioP__LEADING_ZERO_BYTES_10 = 0x09, 
  HalCC2430RadioP__LEADING_ZERO_BYTES_11 = 0x0A, 
  HalCC2430RadioP__LEADING_ZERO_BYTES_12 = 0x0B, 
  HalCC2430RadioP__LEADING_ZERO_BYTES_13 = 0x0C, 
  HalCC2430RadioP__LEADING_ZERO_BYTES_14 = 0x0D, 
  HalCC2430RadioP__LEADING_ZERO_BYTES_15 = 0x0E, 
  HalCC2430RadioP__LEADING_ZERO_BYTES_16 = 0x0F
} HalCC2430RadioP__preamble_length_t;
//#line 158
//#line 147
typedef struct HalCC2430RadioP____nesc_unnamed4291 {

  HalCC2430RadioP__preamble_length_t preamble_length : 4;
  bool autoack : 1;
  bool autocrc : 1;
  uint8_t cca_mode : 2;
  HalCC2430RadioP__cca_hyst_db_t cca_hyst : 3;
  bool adr_decode : 1;
  bool pan_coordinator : 1;
  bool reserved_frame_mode : 1;
  uint8_t reserved : 2;
} HalCC2430RadioP__MDMCTRL0_t;
/*YYYYY } HalCC2430RadioP__MDMCTRL0_t*///# 63 "/opt/tinyos-2.x-contrib/diku/mcs51/tos/chips/cc2430/radio/HalCC2430RadioP.nc"
sfr HalCC2430RadioP__RFIF = 0xE9;

static  /*inline*/  void HalCC2430RadioP__CC2430SetPanid(uint16_t id);
static  /*inline*/  void HalCC2430RadioP__CC2430SetShortaddr(uint16_t shortAddr);
static  /*inline*/  void HalCC2430RadioP__CC2420SetIEEEAddr(ieee_mac_addr_t extAddress);
static  /*inline*/  void HalCC2430RadioP__CC2430Reset(void );
static void HalCC2430RadioP__CC2430RFEnable(void );
static  /*inline*/  void HalCC2430RadioP__CC2430RFDisable(void );
static  /*inline*/  void HalCC2430RadioP__CC2430ExternalOscillator(void );
static void HalCC2430RadioP__CC2430RxEnable(void );
static  /*inline*/  void HalCC2430RadioP__CC2430RxDisable(void );
static void HalCC2430RadioP__CC2430ChannelSet(uint8_t channel);
static  /*inline*/  void HalCC2430RadioP__CC2430PALevelSet(uint8_t new_power);
static  /*inline*/  void HalCC2430RadioP__CC2430ControlSet(void );
static void HalCC2430RadioP__CC2430TxWait(void );
static void HalCC2430RadioP__wait(uint16_t u);
//#line 93
ieee_mac_addr_t HalCC2430RadioP__ieeeAddress;
mac_addr_t HalCC2430RadioP__shortAddress;
//#line 94
mac_addr_t HalCC2430RadioP__panid;

bool HalCC2430RadioP__rxEnabled = FALSE;

uint8_t HalCC2430RadioP__receivedPacket[128];
uint8_t *HalCC2430RadioP__receivedPacketPtr;





static  /*inline*/  error_t HalCC2430RadioP__Init__init(void );
//#line 127
static  /*inline*/  void HalCC2430RadioP__initTask__runTask(void );
//#line 186
static  /*inline*/  error_t HalCC2430RadioP__HALCC2420Control__start(void );
//#line 200
static  /*inline*/  error_t HalCC2430RadioP__HALCC2420Control__stop(void );
//#line 215
uint8_t *HalCC2430RadioP__transmitPacketPtr;
bool HalCC2430RadioP__transmitInProgress = FALSE;

static  /*inline*/  error_t HalCC2430RadioP__HALCC2420__sendPacket(uint8_t *packet);







static  /*inline*/  void HalCC2430RadioP__transmitTask__runTask(void );
//#line 311
uint8_t HalCC2430RadioP__currentChannel;

static error_t HalCC2430RadioP__HALCC2420__setChannel(uint8_t channel);
//#line 325
static  /*inline*/  void HalCC2430RadioP__setChannelTask__runTask(void );
//#line 357
static  /*inline*/  error_t HalCC2430RadioP__HALCC2420__rxEnable(void );










static  /*inline*/  error_t HalCC2430RadioP__HALCC2420__rxDisable(void );
//#line 397
static  /*inline*/  const mac_addr_t *HalCC2430RadioP__HALCC2420__getAddress(void );
//#line 446
static  /*inline*/  void HalCC2430RadioP__CC2430SetPanid(uint16_t id);




static  /*inline*/  void HalCC2430RadioP__CC2430SetShortaddr(uint16_t shortAddr);




static  /*inline*/  void HalCC2430RadioP__CC2420SetIEEEAddr(ieee_mac_addr_t extAddress);
//#line 475
bool HalCC2430RadioP__receivedPacketTaskPosted = FALSE;
//#line 475
bool HalCC2430RadioP__flushBufferTaskPosted = FALSE;




static  /*inline*/  void HalCC2430RadioP__InterruptTXDone__fired(void );
//#line 492
static  /*inline*/  void HalCC2430RadioP__sendPacketDoneTask__runTask(void );







static  /*inline*/  void HalCC2430RadioP__InterruptFIFOP__fired(void );
//#line 515
uint8_t HalCC2430RadioP__frameLength = 0;
//#line 515
uint8_t HalCC2430RadioP__receivedPacketTaskRetries = 0;

static  /*inline*/  void HalCC2430RadioP__receivedPacketTask__runTask(void );
//#line 627
static  /*inline*/  void HalCC2430RadioP__InterruptRFErr__fired(void );
//#line 644
static  /*inline*/  void HalCC2430RadioP__flushBufferTask__runTask(void );
//#line 682
static  /*inline*/  void HalCC2430RadioP__CC2430Reset(void );
//#line 697
static void HalCC2430RadioP__CC2430RFEnable(void );
//#line 710
static  /*inline*/  void HalCC2430RadioP__CC2430RFDisable(void );
//#line 731
static  /*inline*/  void HalCC2430RadioP__CC2430ExternalOscillator(void );










static void HalCC2430RadioP__CC2430RxEnable(void );
//#line 760
static  /*inline*/  void HalCC2430RadioP__CC2430RxDisable(void );
//#line 773
static void HalCC2430RadioP__CC2430ChannelSet(uint8_t channel);
//#line 789
static  /*inline*/  void HalCC2430RadioP__CC2430PALevelSet(uint8_t new_power);
//#line 804
static  /*inline*/  void HalCC2430RadioP__CC2430ControlSet(void );
//#line 828
static void HalCC2430RadioP__CC2430TxWait(void );
//#line 846
static   /*inline*/   void HalCC2430RadioP__wait(uint16_t u);
//#line 870
static  /*inline*/  void HalCC2430RadioP__StdOut__get(uint8_t _data);
//# 50 "/opt/tinyos-2.x/tos/interfaces/GpioCapture.nc"
static void HplCC2430InterruptsC__CaptureSFD__captured(uint16_t time);
//# 57 "/opt/tinyos-2.x/tos/interfaces/GpioInterrupt.nc"
static void HplCC2430InterruptsC__InterruptRFErr__fired(void );
//#line 57
static void HplCC2430InterruptsC__InterruptTXDone__fired(void );
//#line 57
static void HplCC2430InterruptsC__InterruptCCA__fired(void );
//#line 57
static void HplCC2430InterruptsC__InterruptFIFOP__fired(void );
//# 117 "/opt/tinyos-2.x-contrib/diku/mcs51/tos/chips/cc2430/radio/HplCC2430InterruptsC.nc"
static  /*inline*/  error_t HplCC2430InterruptsC__InterruptFIFOP__enableRisingEdge(void );
//#line 117
static error_t HplCC2430InterruptsC__InterruptFIFOP__disable(void );
static  /*inline*/  error_t HplCC2430InterruptsC__InterruptTXDone__enableRisingEdge(void );







static  /*inline*/  error_t HplCC2430InterruptsC__InterruptRFErr__enableRisingEdge(void );
//#line 142
void __vector_0(void )   /*__attribute((interrupt))*/ ;






void __vector_16(void )   /*__attribute((interrupt))*/ ;
//#line 200
static  /*inline*/  void HplCC2430InterruptsC__CaptureSFD__default__captured(uint16_t time);

static  /*inline*/  void HplCC2430InterruptsC__InterruptCCA__default__fired(void );
//# 53 "/opt/tinyos-2.x-contrib/diku/mcs51/tos/chips/cc2430/flash/HalFlashP.nc"
uint8_t xdata HalFlashP__eraseFunctionBuffer[11] = 
{ 0x75, 0xAE, 0x01, 0x00, 0xE5, 0xAE, 0x54, 0x80, 0x70, 0xFA, 0x22 };
uint8_t xdata HalFlashP__writeFunctionBuffer[105] = 
{ 0x7B, 0x01, 0x10, 0xAF, 0x02, 0x7B, 0x00, 0xC0, 0x83, 0xC0, 
0x82, 0xC0, 0x85, 0xC0, 0x84, 0xC0, 0x92, 0xEA, 0xF5, 0x83, 
0xE9, 0xF5, 0x82, 0x75, 0x92, 0x00, 0xE5, 0xAE, 0x54, 0x80, 
0x70, 0xFA, 0x75, 0xAE, 0x02, 0xEC, 0x60, 0x17, 0x7A, 0x40, 
0x79, 0x04, 0xE0, 0xA3, 0xF5, 0xAF, 0xD9, 0xFA, 0x75, 0xAF, 
0x00, 0xE5, 0xAE, 0x54, 0x40, 0x70, 0xFA, 0xDA, 0xED, 0xDC, 
0xE9, 0xED, 0x60, 0x19, 0xED, 0x54, 0xFC, 0x03, 0x03, 0xFD, 
0x79, 0x04, 0xE0, 0xA3, 0xF5, 0xAF, 0xD9, 0xFA, 0x75, 0xAF, 
0x00, 0xE5, 0xAE, 0x54, 0x40, 0x70, 0xFA, 0xDD, 0xED, 0xD0, 
0x92, 0xD0, 0x84, 0xD0, 0x85, 0xD0, 0x82, 0xD0, 0x83, 0xEB, 
0x60, 0x02, 0xD2, 0xAF, 0x22 };

void (*HalFlashP__eraseFunction)(void );
void (*HalFlashP__writeFunction)(uint8_t *_data, uint16_t length);

static  /*inline*/  error_t HalFlashP__Init__init(void );








static  /*inline*/  error_t HalFlashP__HalFlash__read(uint8_t *destination, uint8_t *source, uint16_t length);






static  /*inline*/  error_t HalFlashP__HalFlash__write(uint8_t *source, uint8_t *destination, uint16_t length);
//#line 109
static  /*inline*/  error_t HalFlashP__HalFlash__erase(uint8_t *address);
//# 63 "/opt/tinyos-2.x/tos/interfaces/Read.nc"
static void AdcP__Read__readDone(
//# 43 "/opt/tinyos-2.x-contrib/diku/mcs51/tos/chips/cc2430/adc/AdcP.nc"
uint8_t arg_0x7e6f8030, 
//# 63 "/opt/tinyos-2.x/tos/interfaces/Read.nc"
error_t result, AdcP__Read__val_t val);
//# 56 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static error_t AdcP__signalReadDone__postTask(void );
//# 140 "/opt/tinyos-2.x-contrib/diku/mcs51/tos/chips/cc2430/adc/AdcP.nc"
enum AdcP____nesc_unnamed4292 {
//#line 140
  AdcP__signalReadDone = 10U
};
//#line 140
typedef int AdcP____nesc_sillytask_signalReadDone[AdcP__signalReadDone];
//#line 52
uint8_t AdcP__references[1U];
uint8_t AdcP__resolutions[1U];
uint8_t AdcP__inputs[1U];
bool AdcP__inUse[1U];

uint8_t AdcP__counter;
uint8_t AdcP__lastId = 1U;


static  /*inline*/  error_t AdcP__Init__init(void );
//#line 74
static  /*inline*/  void AdcP__AdcControl__enable(uint8_t id, uint8_t reference, uint8_t resolution, uint8_t input);
//#line 118
static error_t AdcP__Read__read(uint8_t id);
//#line 141
int16_t AdcP__value;


void __vector_1(void )   /*__attribute((interrupt))*/ ;








static  /*inline*/  void AdcP__signalReadDone__runTask(void );
//#line 176
static  /*inline*/  void AdcP__Read__default__readDone(uint8_t id, error_t result, int16_t val);
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
//# 95 "/opt/tinyos-2.x-contrib/diku/mcs51/tos/chips/mcs51/pins/HplMcs51GeneralIOC.nc"
static   /*inline*/   void HplMcs51GeneralIOC__P13__clr(void )
//#line 95
{
//#line 95
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
//# 92 "/opt/tinyos-2.x-contrib/diku/mcs51/tos/chips/mcs51/pins/HplMcs51GeneralIOC.nc"
static   /*inline*/   void HplMcs51GeneralIOC__P10__clr(void )
//#line 92
{
//#line 92
  P1_0 = 0;
}

//# 30 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
 /*inline*/  static void /*PlatformLedsC.Led0_rev*/ReverseGPIOP__0__In__clr(void ){
//#line 30
  HplMcs51GeneralIOC__P10__clr();
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
//# 95 "/opt/tinyos-2.x-contrib/diku/mcs51/tos/chips/mcs51/pins/HplMcs51GeneralIOC.nc"
static   /*inline*/   void HplMcs51GeneralIOC__P13__makeOutput(void )
//#line 95
{
//#line 95
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
//# 92 "/opt/tinyos-2.x-contrib/diku/mcs51/tos/chips/mcs51/pins/HplMcs51GeneralIOC.nc"
static   /*inline*/   void HplMcs51GeneralIOC__P10__makeOutput(void )
//#line 92
{
//#line 92
  P1_DIR |= 1 << 0;
}

//# 35 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
 /*inline*/  static void /*PlatformLedsC.Led0_rev*/ReverseGPIOP__0__In__makeOutput(void ){
//#line 35
  HplMcs51GeneralIOC__P10__makeOutput();
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
//# 34 "/opt/tinyos-2.x-contrib/diku/common/lib/stdout/StdOut.nc"
 /*inline*/  static int TestAppP__StdOut__print(const char *str){
//#line 34
  int result;
//#line 34

//#line 34
  result = StdOutM__StdOut__print(str);
//#line 34

//#line 34
  return result;
//#line 34
}
//#line 34
//# 250 "/opt/tinyos-2.x-contrib/diku/common/lib/stdout/StdOutM.nc"
static  /*inline*/  int StdOutM__StdOut__printBase10uint16(const uint16_t c)
{
  bool print = 0;
  char str[6];
  uint8_t idx = 0;
//#line 254
  uint8_t tmp;
  uint32_t v;

  v = c;


  tmp = v / 10000;
  if (tmp != 0 || print) {
      str[idx] = tmp + 48;
      idx++;
      v = v % 10000;
      print = 1;
    }


  tmp = v / 1000;
  if (tmp != 0 || print) {
      str[idx] = tmp + 48;
      idx++;
      v = v % 1000;
      print = 1;
    }


  tmp = v / 100;
  if (tmp != 0 || print) {
      str[idx] = tmp + 48;
      idx++;
      v = v % 100;
      print = 1;
    }


  tmp = v / 10;
  if (tmp != 0 || print) {
      str[idx] = tmp + 48;
      idx++;
      v = v % 10;
      print = 1;
    }


  str[idx] = v + 48;
  idx++;

  str[idx] = 0;

  return StdOutM__StdOut__print(str);
}


static  /*inline*/  int StdOutM__StdOut__printBase10int16(const int16_t c)
{
  uint8_t counter = 0;
  uint16_t v;

  if (c < 0) {
      counter = StdOutM__StdOut__print("-");
      v = -1 * c;
    }
  else 
//#line 313
    {
      v = (uint16_t )c;
    }

  counter += StdOutM__StdOut__printBase10uint16(v);

  return counter;
}

//# 76 "/opt/tinyos-2.x-contrib/diku/common/lib/stdout/StdOut.nc"
 /*inline*/  static int TestAppP__StdOut__printBase10int16(const int16_t c){
//#line 76
  int result;
//#line 76

//#line 76
  result = StdOutM__StdOut__printBase10int16(c);
//#line 76

//#line 76
  return result;
//#line 76
}
//#line 76
//# 55 "/opt/tinyos-2.x/tos/interfaces/Read.nc"
 /*inline*/  static error_t TestAppP__Read__read(void ){
//#line 55
  unsigned char result;
//#line 55

//#line 55
  result = AdcP__Read__read(/*TestAppC.AdcC*/AdcC__0__ID);
//#line 55

//#line 55
  return result;
//#line 55
}
//#line 55
//# 159 "TestAppP.nc"
static  /*inline*/  void TestAppP__Read__readDone(error_t result, int16_t val)



{
  if (TestAppP__counter != 0) {
      TestAppP__Read__read();
      TestAppP__counter++;
    }
  else 
//#line 167
    {
      TestAppP__counter = 0;
    }

  TestAppP__StdOut__print("Val: ");

  TestAppP__StdOut__printBase10int16(val);



  TestAppP__StdOut__print("\n\r");
}

//# 176 "/opt/tinyos-2.x-contrib/diku/mcs51/tos/chips/cc2430/adc/AdcP.nc"
static  /*inline*/  void AdcP__Read__default__readDone(uint8_t id, error_t result, int16_t val)
//#line 176
{
}

//# 63 "/opt/tinyos-2.x/tos/interfaces/Read.nc"
 /*inline*/  static void AdcP__Read__readDone(uint8_t arg_0x7e6f8030, error_t result, AdcP__Read__val_t val){
//#line 63
  switch (arg_0x7e6f8030) {
//#line 63
    case /*TestAppC.AdcC*/AdcC__0__ID:
//#line 63
      TestAppP__Read__readDone(result, val);
//#line 63
      break;
//#line 63
    default:
//#line 63
      AdcP__Read__default__readDone(arg_0x7e6f8030, result, val);
//#line 63
      break;
//#line 63
    }
//#line 63
}
//#line 63
//# 153 "/opt/tinyos-2.x-contrib/diku/mcs51/tos/chips/cc2430/adc/AdcP.nc"
static  /*inline*/  void AdcP__signalReadDone__runTask(void )
//#line 153
{
  uint8_t tmp;


  tmp = AdcP__lastId;
  AdcP__lastId = 1U;


  AdcP__value >>= 8 - (AdcP__resolutions[tmp] >> 3);
//#line 173
  AdcP__Read__readDone(tmp, SUCCESS, AdcP__value);
}

//# 56 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
 /*inline*/  static error_t SimpleMacM__signalSendPacketDone__postTask(void ){
//#line 56
  unsigned char result;
//#line 56

//#line 56
  result = SchedulerBasicP__TaskBasic__postTask(SimpleMacM__signalSendPacketDone);
//#line 56

//#line 56
  return result;
//#line 56
}
//#line 56
//# 132 "/opt/tinyos-2.x-contrib/diku/common/lib/simplemac/SimpleMacM.nc"
static  /*inline*/  void SimpleMacM__HALCC2420__sendPacketDone(uint8_t *packet, error_t result)
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
//#line 134
    SimpleMacM__sendPacketResult = result;
//#line 134
    __nesc_atomic_end(__nesc_atomic); }

  SimpleMacM__signalSendPacketDone__postTask();
}

//# 43 "/opt/tinyos-2.x-contrib/diku/common/lib/simplemac/HALCC2420.nc"
 /*inline*/  static void HalCC2430RadioP__HALCC2420__sendPacketDone(uint8_t *packet, error_t result){
//#line 43
  SimpleMacM__HALCC2420__sendPacketDone(packet, result);
//#line 43
}
//#line 43
//# 492 "/opt/tinyos-2.x-contrib/diku/mcs51/tos/chips/cc2430/radio/HalCC2430RadioP.nc"
static  /*inline*/  void HalCC2430RadioP__sendPacketDoneTask__runTask(void )
//#line 492
{
  HalCC2430RadioP__HALCC2420__sendPacketDone(HalCC2430RadioP__transmitPacketPtr, SUCCESS);
}

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

//# 34 "/opt/tinyos-2.x-contrib/diku/common/lib/stdout/StdOut.nc"
 /*inline*/  static int HalCC2430RadioP__StdOut__print(const char *str){
//#line 34
  int result;
//#line 34

//#line 34
  result = StdOutM__StdOut__print(str);
//#line 34

//#line 34
  return result;
//#line 34
}
//#line 34
//# 50 "/opt/tinyos-2.x/tos/interfaces/GpioInterrupt.nc"
 /*inline*/  static error_t HalCC2430RadioP__InterruptFIFOP__disable(void ){
//#line 50
  unsigned char result;
//#line 50

//#line 50
  result = HplCC2430InterruptsC__InterruptFIFOP__disable();
//#line 50

//#line 50
  return result;
//#line 50
}
//#line 50
//# 760 "/opt/tinyos-2.x-contrib/diku/mcs51/tos/chips/cc2430/radio/HalCC2430RadioP.nc"
static  /*inline*/  void HalCC2430RadioP__CC2430RxDisable(void )
{
  RFST = 0xE5;


  HalCC2430RadioP__InterruptFIFOP__disable();

  HalCC2430RadioP__rxEnabled = FALSE;
}

//#line 644
static  /*inline*/  void HalCC2430RadioP__flushBufferTask__runTask(void )
{
  if (HalCC2430RadioP__rxEnabled) 
    {

      HalCC2430RadioP__CC2430RxDisable();


      RFST = 0xE6;
      RFST = 0xE6;


      HalCC2430RadioP__CC2430RxEnable();
    }
  else {

      RFST = 0xE6;
      RFST = 0xE6;
    }

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
//#line 664
    HalCC2430RadioP__flushBufferTaskPosted = FALSE;
//#line 664
    __nesc_atomic_end(__nesc_atomic); }

  HalCC2430RadioP__StdOut__print("MAC: Buffer flushed\r\n");
}

//# 117 "/opt/tinyos-2.x-contrib/diku/mcs51/tos/chips/cc2430/radio/HplCC2430InterruptsC.nc"
static  /*inline*/  error_t HplCC2430InterruptsC__InterruptFIFOP__enableRisingEdge(void )
//#line 117
{
//#line 117
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
//#line 117
    {
//#line 117
      RFIM |= 1 << CC2430_RFIM_FIFOP;
//#line 117
      RFIF &= ~(1 << CC2430_RFIF_FIFOP);
//#line 117
      IEN2 |= 1 << CC2430_IEN2_RFIE;
    }
//#line 118
    __nesc_atomic_end(__nesc_atomic); }
//#line 117
  return SUCCESS;
}

//# 42 "/opt/tinyos-2.x/tos/interfaces/GpioInterrupt.nc"
 /*inline*/  static error_t HalCC2430RadioP__InterruptFIFOP__enableRisingEdge(void ){
//#line 42
  unsigned char result;
//#line 42

//#line 42
  result = HplCC2430InterruptsC__InterruptFIFOP__enableRisingEdge();
//#line 42

//#line 42
  return result;
//#line 42
}
//#line 42
//# 56 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
 /*inline*/  static error_t HalCC2430RadioP__flushBufferTask__postTask(void ){
//#line 56
  unsigned char result;
//#line 56

//#line 56
  result = SchedulerBasicP__TaskBasic__postTask(HalCC2430RadioP__flushBufferTask);
//#line 56

//#line 56
  return result;
//#line 56
}
//#line 56
 /*inline*/  static error_t HalCC2430RadioP__receivedPacketTask__postTask(void ){
//#line 56
  unsigned char result;
//#line 56

//#line 56
  result = SchedulerBasicP__TaskBasic__postTask(HalCC2430RadioP__receivedPacketTask);
//#line 56

//#line 56
  return result;
//#line 56
}
//#line 56
//# 39 "/opt/tinyos-2.x-contrib/diku/common/lib/stdout/StdOut.nc"
 /*inline*/  static int TestAppP__StdOut__printHex(uint8_t c){
//#line 39
  int result;
//#line 39

//#line 39
  result = StdOutM__StdOut__printHex(c);
//#line 39

//#line 39
  return result;
//#line 39
}
//#line 39





 /*inline*/  static int TestAppP__StdOut__printHexword(uint16_t c){
//#line 44
  int result;
//#line 44

//#line 44
  result = StdOutM__StdOut__printHexword(c);
//#line 44

//#line 44
  return result;
//#line 44
}
//#line 44
//# 207 "TestAppP.nc"
static  /*inline*/  packet_t *TestAppP__SimpleMac__receivedPacket(packet_t *packet)
{
  uint8_t i;

  TestAppP__StdOut__print("Received packet: ");
  TestAppP__StdOut__printHex(packet->length);
  TestAppP__StdOut__print(" ");
  TestAppP__StdOut__printHexword(packet->fcf);
  TestAppP__StdOut__print(" ");
  TestAppP__StdOut__printHex(packet->_data_seq_no);
  TestAppP__StdOut__print(" ");
  TestAppP__StdOut__printHexword(packet->dest);
  TestAppP__StdOut__print(" ");
  TestAppP__StdOut__printHexword(packet->src);
  TestAppP__StdOut__print(" ");

  for (i = 0; i < packet->length - 9; i++) 
    {
      TestAppP__StdOut__printHex(packet->_data[i]);
      TestAppP__StdOut__print(" ");
    }

  TestAppP__StdOut__printHex(packet->fcs.rssi);
  TestAppP__StdOut__print(" ");
  TestAppP__StdOut__printHex(packet->fcs.correlation);
  TestAppP__StdOut__print("\r\n");


  return packet;
}

//# 52 "/opt/tinyos-2.x-contrib/diku/common/lib/simplemac/SimpleMac.nc"
 /*inline*/  static packet_t *SimpleMacM__SimpleMac__receivedPacket(packet_t *packet){
//#line 52
  struct packet *result;
//#line 52

//#line 52
  result = TestAppP__SimpleMac__receivedPacket(packet);
//#line 52

//#line 52
  return result;
//#line 52
}
//#line 52
//# 177 "/opt/tinyos-2.x-contrib/diku/common/lib/simplemac/SimpleMacM.nc"
static  /*inline*/  uint8_t *SimpleMacM__HALCC2420__receivedPacket(uint8_t *packet)
{
  uint8_t length;
//#line 179
  uint8_t i;
  uint16_t tmp;


  length = packet[0];

  SimpleMacM__receivedPacketPtr->length = length;

  tmp = packet[2];
  tmp = (tmp << 8) + packet[1];
  SimpleMacM__receivedPacketPtr->fcf = tmp;

  SimpleMacM__receivedPacketPtr->_data_seq_no = packet[3];

  tmp = packet[5];
  tmp = (tmp << 8) + packet[4];
  SimpleMacM__receivedPacketPtr->dest = tmp;

  tmp = packet[7];
  tmp = (tmp << 8) + packet[6];
  SimpleMacM__receivedPacketPtr->src = tmp;

  for (i = 8; i < length - 1; i++) 
    {
      SimpleMacM__receivedPacketPtr->_data[i - 8] = packet[i];
    }

  SimpleMacM__receivedPacketPtr->fcs.rssi = packet[length - 1];
  SimpleMacM__receivedPacketPtr->fcs.correlation = packet[length];

  SimpleMacM__receivedPacketPtr = SimpleMacM__SimpleMac__receivedPacket(SimpleMacM__receivedPacketPtr);

  return packet;
}

//# 53 "/opt/tinyos-2.x-contrib/diku/common/lib/simplemac/HALCC2420.nc"
 /*inline*/  static uint8_t *HalCC2430RadioP__HALCC2420__receivedPacket(uint8_t *packet){
//#line 53
  unsigned char *result;
//#line 53

//#line 53
  result = SimpleMacM__HALCC2420__receivedPacket(packet);
//#line 53

//#line 53
  return result;
//#line 53
}
//#line 53
//# 517 "/opt/tinyos-2.x-contrib/diku/mcs51/tos/chips/cc2430/radio/HalCC2430RadioP.nc"
static  /*inline*/  void HalCC2430RadioP__receivedPacketTask__runTask(void )
{
  uint8_t correlation;
//#line 519
  uint8_t i;




  if (HalCC2430RadioP__frameLength == 0) 
    {
      HalCC2430RadioP__frameLength = RFD;

      if (HalCC2430RadioP__frameLength < 127) {
        HalCC2430RadioP__receivedPacketPtr[0] = HalCC2430RadioP__frameLength;
        }
      else 
//#line 530
        {


          HalCC2430RadioP__flushBufferTask__postTask();
          { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
//#line 534
            HalCC2430RadioP__receivedPacketTaskPosted = FALSE;
//#line 534
            __nesc_atomic_end(__nesc_atomic); }
          HalCC2430RadioP__frameLength = 0;
          HalCC2430RadioP__receivedPacketTaskRetries = 0;
          return;
        }
    }


  if (HalCC2430RadioP__frameLength > * (uint8_t xdata *)0xDF53) 
    {



      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
//#line 547
        {
          if (HalCC2430RadioP__flushBufferTaskPosted || HalCC2430RadioP__receivedPacketTaskRetries > 10) 
            {
              HalCC2430RadioP__flushBufferTask__postTask();
              HalCC2430RadioP__receivedPacketTaskPosted = FALSE;
              HalCC2430RadioP__frameLength = 0;
              HalCC2430RadioP__receivedPacketTaskRetries = 0;
            }
          else 
//#line 554
            {
              HalCC2430RadioP__receivedPacketTask__postTask();
              HalCC2430RadioP__receivedPacketTaskRetries++;
            }
        }
//#line 558
        __nesc_atomic_end(__nesc_atomic); }

      return;
    }


  HalCC2430RadioP__receivedPacketTaskRetries = 0;


  for (i = 0; i < HalCC2430RadioP__frameLength; i++) {
      if (* (uint8_t xdata *)0xDF53 > 0 && * (uint8_t xdata *)0xDF53 < 129) {
        HalCC2430RadioP__receivedPacketPtr[i + 1] = RFD;
        }
      else 
//#line 570
        {
          HalCC2430RadioP__receivedPacketPtr[HalCC2430RadioP__frameLength] = 0;
          break;
        }
    }


  correlation = HalCC2430RadioP__receivedPacketPtr[HalCC2430RadioP__frameLength];


  HalCC2430RadioP__frameLength = 0;






  if (correlation & FCS_CRC_OK_MASK) 
    {
      HalCC2430RadioP__receivedPacketPtr = HalCC2430RadioP__HALCC2420__receivedPacket(HalCC2430RadioP__receivedPacketPtr);
    }
  else {

      HalCC2430RadioP__StdOut__print("MAC: CRC Failed\n\r");


      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
//#line 596
        HalCC2430RadioP__receivedPacketTaskPosted = FALSE;
//#line 596
        __nesc_atomic_end(__nesc_atomic); }
      HalCC2430RadioP__flushBufferTask__postTask();
    }




  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
//#line 603
    {


      if (* (uint8_t xdata *)0xDF53 > 0 && * (uint8_t xdata *)0xDF53 < 129) {
        HalCC2430RadioP__receivedPacketTask__postTask();
        }
      else {
          HalCC2430RadioP__receivedPacketTaskPosted = FALSE;



          if (HalCC2430RadioP__flushBufferTaskPosted) {
            HalCC2430RadioP__flushBufferTask__postTask();
            }
        }
    }
//#line 618
    __nesc_atomic_end(__nesc_atomic); }

  HalCC2430RadioP__RFIF &= ~(1 << CC2430_RFIF_FIFOP);
}

//#line 325
static  /*inline*/  void HalCC2430RadioP__setChannelTask__runTask(void )
{

  HalCC2430RadioP__CC2430TxWait();
  if (HalCC2430RadioP__rxEnabled) 
    {
      HalCC2430RadioP__CC2430RxDisable();

      HalCC2430RadioP__CC2430ChannelSet(HalCC2430RadioP__currentChannel);
      HalCC2430RadioP__CC2430RxEnable();
    }
  else {
    HalCC2430RadioP__CC2430ChannelSet(HalCC2430RadioP__currentChannel);
    }
}

//#line 846
static   /*inline*/   void HalCC2430RadioP__wait(uint16_t u)
{
  uint8_t j;
  uint16_t i;

  u = u >> 3;



  for (i = 0; i < u; i++) {
      for (j = 0; j < 32; ) {
          j++;
        }
    }
}

//#line 226
static  /*inline*/  void HalCC2430RadioP__transmitTask__runTask(void )
{
  bool oldRxEnabled;
  uint8_t i;
//#line 229
  uint8_t status;
//#line 229
  uint8_t counter;





  oldRxEnabled = HalCC2430RadioP__rxEnabled;
  if (!HalCC2430RadioP__rxEnabled) 
    {

      HalCC2430RadioP__CC2430RxEnable();
    }


  HalCC2430RadioP__wait(128);


  if (!(* (uint8_t xdata *)0xDF62 & (1 << CC2430_RFSTATUS_CCA))) {


      if (!HalCC2430RadioP__rxEnabled) 
        {

          HalCC2430RadioP__CC2430RxDisable();
        }

      HalCC2430RadioP__HALCC2420__sendPacketDone(HalCC2430RadioP__transmitPacketPtr, 2);

      return;
    }







  RFST = 0xE7;


  RFD = HalCC2430RadioP__transmitPacketPtr[0];

  for (i = 0; i < HalCC2430RadioP__transmitPacketPtr[0]; i++) {
      RFD = HalCC2430RadioP__transmitPacketPtr[i + 1];
    }


  i = 0;
  while (i++ < 3) 
    {
      RFST = 0xE3;
      counter = 0;

      do {
          status = * (uint8_t xdata *)0xDF62;
        }
      while (
//#line 284
      !(status & (1 << CC2430_RFSTATUS_TX_ACTIVE)) && counter++ < 200);

      if (status & (1 << CC2430_RFSTATUS_TX_ACTIVE)) {
        break;
        }
    }



  if (!(status & (1 << CC2430_RFSTATUS_TX_ACTIVE))) {


      RFST = 0xE7;

      HalCC2430RadioP__HALCC2420__sendPacketDone(HalCC2430RadioP__transmitPacketPtr, 3);

      return;
    }

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
//#line 303
    HalCC2430RadioP__transmitInProgress = TRUE;
//#line 303
    __nesc_atomic_end(__nesc_atomic); }

  return;
}

//# 126 "/opt/tinyos-2.x-contrib/diku/mcs51/tos/chips/cc2430/radio/HplCC2430InterruptsC.nc"
static  /*inline*/  error_t HplCC2430InterruptsC__InterruptRFErr__enableRisingEdge(void )
//#line 126
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
//#line 127
    {
      RFERRIF = 0;
      RFERRIE = 1;
    }
//#line 130
    __nesc_atomic_end(__nesc_atomic); }
  return SUCCESS;
}

//# 42 "/opt/tinyos-2.x/tos/interfaces/GpioInterrupt.nc"
 /*inline*/  static error_t HalCC2430RadioP__InterruptRFErr__enableRisingEdge(void ){
//#line 42
  unsigned char result;
//#line 42

//#line 42
  result = HplCC2430InterruptsC__InterruptRFErr__enableRisingEdge();
//#line 42

//#line 42
  return result;
//#line 42
}
//#line 42
//# 118 "/opt/tinyos-2.x-contrib/diku/mcs51/tos/chips/cc2430/radio/HplCC2430InterruptsC.nc"
static  /*inline*/  error_t HplCC2430InterruptsC__InterruptTXDone__enableRisingEdge(void )
//#line 118
{
//#line 118
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
//#line 118
    {
//#line 118
      RFIM |= 1 << CC2430_RFIM_TXDONE;
//#line 118
      RFIF &= ~(1 << CC2430_RFIF_TXDONE);
//#line 118
      IEN2 |= 1 << CC2430_IEN2_RFIE;
    }
//#line 119
    __nesc_atomic_end(__nesc_atomic); }
//#line 118
  return SUCCESS;
}

//# 42 "/opt/tinyos-2.x/tos/interfaces/GpioInterrupt.nc"
 /*inline*/  static error_t HalCC2430RadioP__InterruptTXDone__enableRisingEdge(void ){
//#line 42
  unsigned char result;
//#line 42

//#line 42
  result = HplCC2430InterruptsC__InterruptTXDone__enableRisingEdge();
//#line 42

//#line 42
  return result;
//#line 42
}
//#line 42
//# 710 "/opt/tinyos-2.x-contrib/diku/mcs51/tos/chips/cc2430/radio/HalCC2430RadioP.nc"
static  /*inline*/  void HalCC2430RadioP__CC2430RFDisable(void )
{
  * (uint8_t xdata *)0xDF17 |= 1 << CC2430_RFPWR_RREG_RADIO_PD;
}

//#line 789
static  /*inline*/  void HalCC2430RadioP__CC2430PALevelSet(uint8_t new_power)
{
  uint16_t power;

  power = new_power * 0x1F;
  power /= 100;

  * (uint16_t xdata *)0xDF0A = (* (uint16_t xdata *)0xDF0A & ~0x1F) | (power & 0x1F);
}






static  /*inline*/  void HalCC2430RadioP__CC2430ControlSet(void )
{

  * (uint8_t xdata *)0xDF02 &= ~(1 << CC2430_MDMCTRL0H_ADDR_DECODE);



  * (uint8_t xdata *)0xDF03 |= 1 << CC2430_MDMCTRL0L_AUTOCRC;





  * (uint8_t xdata *)0xDF4F = 0x7F;


  * (uint16_t xdata *)0xDF0A = 0x050F;

  return;
}

//#line 446
static  /*inline*/  void HalCC2430RadioP__CC2430SetPanid(uint16_t id)
{
  * (uint16_t xdata *)0xDF4B = id;
}

static  /*inline*/  void HalCC2430RadioP__CC2430SetShortaddr(uint16_t shortAddr)
{
  * (uint16_t xdata *)0xDF4D = shortAddr;
}

static  /*inline*/  void HalCC2430RadioP__CC2420SetIEEEAddr(ieee_mac_addr_t extAddress)
{
  * (uint8_t xdata *)0xDF4A = extAddress[7];
  * (uint8_t xdata *)0xDF49 = extAddress[6];
  * (uint8_t xdata *)0xDF48 = extAddress[5];
  * (uint8_t xdata *)0xDF47 = extAddress[4];
  * (uint8_t xdata *)0xDF46 = extAddress[3];
  * (uint8_t xdata *)0xDF45 = extAddress[2];
  * (uint8_t xdata *)0xDF44 = extAddress[1];
  * (uint8_t xdata *)0xDF43 = extAddress[0];
}

//#line 731
static  /*inline*/  void HalCC2430RadioP__CC2430ExternalOscillator(void )
{
  SLEEP &= ~(1 << CC2430_SLEEP_OSC_PD);
  CLKCON &= ~(1 << CC2430_CLKCON_CLKSPD);

  while (!(SLEEP & (1 << CC2430_SLEEP_XOSC_STB))) ;
}

//#line 682
static  /*inline*/  void HalCC2430RadioP__CC2430Reset(void )
{

  * (uint8_t xdata *)0xDF17 &= ~(1 << CC2430_RFPWR_RREG_RADIO_PD);
  * (uint8_t xdata *)0xDF17 |= 1 << CC2430_RFPWR_RREG_RADIO_PD;



  while (HalCC2430RadioP__RFIF & (1 << CC2430_RFIF_RREG_ON)) {
    }
}

//#line 127
static  /*inline*/  void HalCC2430RadioP__initTask__runTask(void )
{

  HalCC2430RadioP__ieeeAddress[0] = 0x10;
  HalCC2430RadioP__ieeeAddress[1] = 0x3d;
  HalCC2430RadioP__ieeeAddress[2] = 0x23;


  HalCC2430RadioP__ieeeAddress[3] = 0;
  HalCC2430RadioP__ieeeAddress[4] = 0;
  HalCC2430RadioP__ieeeAddress[5] = 0;
  HalCC2430RadioP__ieeeAddress[6] = TOS_NODE_ID >> 8;
  HalCC2430RadioP__ieeeAddress[7] = TOS_NODE_ID;


  HalCC2430RadioP__CC2430Reset();


  HalCC2430RadioP__CC2430ExternalOscillator();






  HalCC2430RadioP__CC2420SetIEEEAddr(HalCC2430RadioP__ieeeAddress);


  HalCC2430RadioP__shortAddress = ((uint16_t )HalCC2430RadioP__ieeeAddress[6] << 8) | HalCC2430RadioP__ieeeAddress[7];
  HalCC2430RadioP__CC2430SetShortaddr(HalCC2430RadioP__shortAddress);


  HalCC2430RadioP__panid = HalCC2430RadioP__shortAddress;
  HalCC2430RadioP__CC2430SetPanid(HalCC2430RadioP__panid);





  HalCC2430RadioP__CC2430ControlSet();


  HalCC2430RadioP__CC2430ChannelSet(20);


  HalCC2430RadioP__CC2430PALevelSet(1);




  HalCC2430RadioP__CC2430RFDisable();

  HalCC2430RadioP__InterruptTXDone__enableRisingEdge();
  HalCC2430RadioP__InterruptRFErr__enableRisingEdge();
}

//# 193 "TestAppP.nc"
static  /*inline*/  void TestAppP__SimpleMac__sendPacketDone(packet_t *packet, error_t result)
{

  if (result == SUCCESS) {
      TestAppP__StdOut__print("Transmission done\r\n");
    }
  else 
//#line 198
    {
      TestAppP__StdOut__print("Transmission failed: ");
      TestAppP__StdOut__printHex(result);
      TestAppP__StdOut__print("\r\n");
    }

  return;
}

//# 42 "/opt/tinyos-2.x-contrib/diku/common/lib/simplemac/SimpleMac.nc"
 /*inline*/  static void SimpleMacM__SimpleMac__sendPacketDone(packet_t *packet, error_t result){
//#line 42
  TestAppP__SimpleMac__sendPacketDone(packet, result);
//#line 42
}
//#line 42
//# 123 "/opt/tinyos-2.x-contrib/diku/common/lib/simplemac/SimpleMacM.nc"
static  /*inline*/  void SimpleMacM__signalSendPacketDone__runTask(void )
{
  error_t tmp;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
//#line 127
    tmp = SimpleMacM__sendPacketResult;
//#line 127
    __nesc_atomic_end(__nesc_atomic); }

  SimpleMacM__SimpleMac__sendPacketDone(SimpleMacM__sendPacketPtr, tmp);
}

//# 397 "/opt/tinyos-2.x-contrib/diku/mcs51/tos/chips/cc2430/radio/HalCC2430RadioP.nc"
static  /*inline*/  const mac_addr_t *HalCC2430RadioP__HALCC2420__getAddress(void )
{
  return &HalCC2430RadioP__shortAddress;
}

//# 59 "/opt/tinyos-2.x-contrib/diku/common/lib/simplemac/HALCC2420.nc"
 /*inline*/  static const mac_addr_t *SimpleMacM__HALCC2420__getAddress(void ){
//#line 59
  unsigned short const *result;
//#line 59

//#line 59
  result = HalCC2430RadioP__HALCC2420__getAddress();
//#line 59

//#line 59
  return result;
//#line 59
}
//#line 59
//# 57 "/opt/tinyos-2.x-contrib/diku/common/lib/simplemac/SimpleMacM.nc"
static  /*inline*/  void SimpleMacM__initTask__runTask(void )
{
  SimpleMacM__shortAddress = SimpleMacM__HALCC2420__getAddress();
}

//# 61 "/opt/tinyos-2.x-contrib/diku/common/lib/stdout/StdOut.nc"
 /*inline*/  static int TestAppP__StdOut__printBase10uint8(const uint8_t c){
//#line 61
  int result;
//#line 61

//#line 61
  result = StdOutM__StdOut__printBase10uint8(c);
//#line 61

//#line 61
  return result;
//#line 61
}
//#line 61
//# 109 "/opt/tinyos-2.x-contrib/diku/mcs51/tos/chips/cc2430/flash/HalFlashP.nc"
static  /*inline*/  error_t HalFlashP__HalFlash__erase(uint8_t *address)
//#line 109
{
  uint8_t page;
  uint8_t old;

  page = (uint32_t )address / 0x0800;

  FADDRH = page << 1;

  old = MEMCTR;
  MEMCTR |= 0x40;

  HalFlashP__eraseFunction();

  MEMCTR = old;

  return SUCCESS;
}

//# 24 "/opt/tinyos-2.x-contrib/diku/common/tos/interfaces/HalFlash.nc"
 /*inline*/  static error_t TestAppP__HalFlash__erase(uint8_t *address){
//#line 24
  unsigned char result;
//#line 24

//#line 24
  result = HalFlashP__HalFlash__erase(address);
//#line 24

//#line 24
  return result;
//#line 24
}
//#line 24
//# 87 "/opt/tinyos-2.x-contrib/diku/mcs51/tos/chips/cc2430/flash/HalFlashP.nc"
static  /*inline*/  error_t HalFlashP__HalFlash__write(uint8_t *source, uint8_t *destination, uint16_t length)
//#line 87
{
  uint8_t page;
//#line 88
  uint8_t row;
//#line 88
  uint8_t location;
  uint8_t old;

  page = (uint32_t )destination / 0x0800;
  row = (uint32_t )destination % 0x0800 >> 8;
  location = ((uint32_t )destination % 0x0800 & 0xFF) >> 2;


  FADDRH = (page << 1) | (row >> 2);
  FADDRL = (row << 6) | location;

  FWT = 0x24;

  old = MEMCTR;
  MEMCTR |= 0x40;
  HalFlashP__writeFunction(source, length);
  MEMCTR = old;

  return SUCCESS;
}

//# 23 "/opt/tinyos-2.x-contrib/diku/common/tos/interfaces/HalFlash.nc"
 /*inline*/  static error_t TestAppP__HalFlash__write(uint8_t *source, uint8_t *destination, uint16_t length){
//#line 23
  unsigned char result;
//#line 23

//#line 23
  result = HalFlashP__HalFlash__write(source, destination, length);
//#line 23

//#line 23
  return result;
//#line 23
}
//#line 23
//# 80 "/opt/tinyos-2.x-contrib/diku/mcs51/tos/chips/cc2430/flash/HalFlashP.nc"
static  /*inline*/  error_t HalFlashP__HalFlash__read(uint8_t *destination, uint8_t *source, uint16_t length)
//#line 80
{

  memcpy((uint8_t xdata *)destination, (uint8_t xdata *)source, length);

  return SUCCESS;
}

//# 22 "/opt/tinyos-2.x-contrib/diku/common/tos/interfaces/HalFlash.nc"
 /*inline*/  static error_t TestAppP__HalFlash__read(uint8_t *destination, uint8_t *source, uint16_t length){
//#line 22
  unsigned char result;
//#line 22

//#line 22
  result = HalFlashP__HalFlash__read(destination, source, length);
//#line 22

//#line 22
  return result;
//#line 22
}
//#line 22
//# 55 "/opt/tinyos-2.x-contrib/diku/common/lib/simplemac/HALCC2420.nc"
 /*inline*/  static error_t SimpleMacM__HALCC2420__setChannel(uint8_t channel){
//#line 55
  unsigned char result;
//#line 55

//#line 55
  result = HalCC2430RadioP__HALCC2420__setChannel(channel);
//#line 55

//#line 55
  return result;
//#line 55
}
//#line 55
//# 142 "/opt/tinyos-2.x-contrib/diku/common/lib/simplemac/SimpleMacM.nc"
static  /*inline*/  error_t SimpleMacM__SimpleMac__setChannel(uint8_t channel)
{
  return SimpleMacM__HALCC2420__setChannel(channel);
}

//# 54 "/opt/tinyos-2.x-contrib/diku/common/lib/simplemac/SimpleMac.nc"
 /*inline*/  static error_t TestAppP__SimpleMac__setChannel(uint8_t channel){
//#line 54
  unsigned char result;
//#line 54

//#line 54
  result = SimpleMacM__SimpleMac__setChannel(channel);
//#line 54

//#line 54
  return result;
//#line 54
}
//#line 54
//# 357 "/opt/tinyos-2.x-contrib/diku/mcs51/tos/chips/cc2430/radio/HalCC2430RadioP.nc"
static  /*inline*/  error_t HalCC2430RadioP__HALCC2420__rxEnable(void )
{

  HalCC2430RadioP__CC2430RxEnable();

  return SUCCESS;
}

//# 62 "/opt/tinyos-2.x-contrib/diku/common/lib/simplemac/HALCC2420.nc"
 /*inline*/  static error_t SimpleMacM__HALCC2420__rxEnable(void ){
//#line 62
  unsigned char result;
//#line 62

//#line 62
  result = HalCC2430RadioP__HALCC2420__rxEnable();
//#line 62

//#line 62
  return result;
//#line 62
}
//#line 62
//# 160 "/opt/tinyos-2.x-contrib/diku/common/lib/simplemac/SimpleMacM.nc"
static  /*inline*/  error_t SimpleMacM__SimpleMac__rxEnable(void )
{
  return SimpleMacM__HALCC2420__rxEnable();
}

//# 61 "/opt/tinyos-2.x-contrib/diku/common/lib/simplemac/SimpleMac.nc"
 /*inline*/  static error_t TestAppP__SimpleMac__rxEnable(void ){
//#line 61
  unsigned char result;
//#line 61

//#line 61
  result = SimpleMacM__SimpleMac__rxEnable();
//#line 61

//#line 61
  return result;
//#line 61
}
//#line 61
//# 368 "/opt/tinyos-2.x-contrib/diku/mcs51/tos/chips/cc2430/radio/HalCC2430RadioP.nc"
static  /*inline*/  error_t HalCC2430RadioP__HALCC2420__rxDisable(void )
{

  if (HalCC2430RadioP__rxEnabled) 
    {
      HalCC2430RadioP__CC2430RxDisable();
    }

  return SUCCESS;
}

//# 63 "/opt/tinyos-2.x-contrib/diku/common/lib/simplemac/HALCC2420.nc"
 /*inline*/  static error_t SimpleMacM__HALCC2420__rxDisable(void ){
//#line 63
  unsigned char result;
//#line 63

//#line 63
  result = HalCC2430RadioP__HALCC2420__rxDisable();
//#line 63

//#line 63
  return result;
//#line 63
}
//#line 63
//# 169 "/opt/tinyos-2.x-contrib/diku/common/lib/simplemac/SimpleMacM.nc"
static  /*inline*/  error_t SimpleMacM__SimpleMac__rxDisable(void )
{
  return SimpleMacM__HALCC2420__rxDisable();
}

//# 62 "/opt/tinyos-2.x-contrib/diku/common/lib/simplemac/SimpleMac.nc"
 /*inline*/  static error_t TestAppP__SimpleMac__rxDisable(void ){
//#line 62
  unsigned char result;
//#line 62

//#line 62
  result = SimpleMacM__SimpleMac__rxDisable();
//#line 62

//#line 62
  return result;
//#line 62
}
//#line 62
//# 186 "/opt/tinyos-2.x-contrib/diku/mcs51/tos/chips/cc2430/radio/HalCC2430RadioP.nc"
static  /*inline*/  error_t HalCC2430RadioP__HALCC2420Control__start(void )
{

  HalCC2430RadioP__CC2430RFEnable();


  HalCC2430RadioP__CC2430RxEnable();

  return SUCCESS;
}

//# 74 "/opt/tinyos-2.x/tos/interfaces/StdControl.nc"
 /*inline*/  static error_t SimpleMacM__HALCC2420Control__start(void ){
//#line 74
  unsigned char result;
//#line 74

//#line 74
  result = HalCC2430RadioP__HALCC2420Control__start();
//#line 74

//#line 74
  return result;
//#line 74
}
//#line 74
//# 65 "/opt/tinyos-2.x-contrib/diku/common/lib/simplemac/SimpleMacM.nc"
static  /*inline*/  error_t SimpleMacM__SimpleMacControl__start(void )
{
  return SimpleMacM__HALCC2420Control__start();
}

//# 74 "/opt/tinyos-2.x/tos/interfaces/StdControl.nc"
 /*inline*/  static error_t TestAppP__SimpleMacControl__start(void ){
//#line 74
  unsigned char result;
//#line 74

//#line 74
  result = SimpleMacM__SimpleMacControl__start();
//#line 74

//#line 74
  return result;
//#line 74
}
//#line 74
//# 200 "/opt/tinyos-2.x-contrib/diku/mcs51/tos/chips/cc2430/radio/HalCC2430RadioP.nc"
static  /*inline*/  error_t HalCC2430RadioP__HALCC2420Control__stop(void )
{

  HalCC2430RadioP__CC2430TxWait();
  HalCC2430RadioP__CC2430RFDisable();
  return SUCCESS;
}

//# 84 "/opt/tinyos-2.x/tos/interfaces/StdControl.nc"
 /*inline*/  static error_t SimpleMacM__HALCC2420Control__stop(void ){
//#line 84
  unsigned char result;
//#line 84

//#line 84
  result = HalCC2430RadioP__HALCC2420Control__stop();
//#line 84

//#line 84
  return result;
//#line 84
}
//#line 84
//# 70 "/opt/tinyos-2.x-contrib/diku/common/lib/simplemac/SimpleMacM.nc"
static  /*inline*/  error_t SimpleMacM__SimpleMacControl__stop(void )
{
  return SimpleMacM__HALCC2420Control__stop();
}

//# 84 "/opt/tinyos-2.x/tos/interfaces/StdControl.nc"
 /*inline*/  static error_t TestAppP__SimpleMacControl__stop(void ){
//#line 84
  unsigned char result;
//#line 84

//#line 84
  result = SimpleMacM__SimpleMacControl__stop();
//#line 84

//#line 84
  return result;
//#line 84
}
//#line 84
//# 34 "/opt/tinyos-2.x-contrib/diku/common/lib/simplemac/SimpleMac.nc"
 /*inline*/  static error_t TestAppP__SimpleMac__sendPacket(packet_t *packet){
//#line 34
  unsigned char result;
//#line 34

//#line 34
  result = SimpleMacM__SimpleMac__sendPacket(packet);
//#line 34

//#line 34
  return result;
//#line 34
}
//#line 34
//# 507 "/opt/tinyos-2.x-contrib/diku/common/lib/stdout/StdOutM.nc"
static  /*inline*/  void StdOutM__StdOut__dumpHex(uint8_t ptr[], uint8_t countar, char *sep)
//#line 507
{
  int i;

//#line 509
  for (i = 0; i < countar; i++) {
      if (i != 0) {
          StdOutM__StdOut__print(sep);
        }
      StdOutM__StdOut__printHex(ptr[i]);
    }
}

//# 56 "/opt/tinyos-2.x-contrib/diku/common/lib/stdout/StdOut.nc"
 /*inline*/  static void TestAppP__StdOut__dumpHex(uint8_t ptr[], uint8_t count, char *sep){
//#line 56
  StdOutM__StdOut__dumpHex(ptr, count, sep);
//#line 56
}
//#line 56
//# 95 "/opt/tinyos-2.x-contrib/diku/mcs51/tos/chips/mcs51/pins/HplMcs51GeneralIOC.nc"
static   /*inline*/   void HplMcs51GeneralIOC__P13__set(void )
//#line 95
{
//#line 95
  P1_3 = 1;
}

//# 29 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
 /*inline*/  static void /*PlatformLedsC.Led2_rev*/ReverseGPIOP__1__In__set(void ){
//#line 29
  HplMcs51GeneralIOC__P13__set();
//#line 29
}
//#line 29
//# 59 "/opt/tinyos-2.x-contrib/diku/common/lib/ReverseGPIOP.nc"
static   /*inline*/   void /*PlatformLedsC.Led2_rev*/ReverseGPIOP__1__Out__clr(void )
//#line 59
{
//#line 59
  /*PlatformLedsC.Led2_rev*/ReverseGPIOP__1__In__set();
}

//# 30 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
 /*inline*/  static void LedsP__Led2__clr(void ){
//#line 30
  /*PlatformLedsC.Led2_rev*/ReverseGPIOP__1__Out__clr();
//#line 30
}
//#line 30
//# 93 "/opt/tinyos-2.x/tos/system/LedsP.nc"
static  /*inline*/  void LedsP__Leds__led2On(void )
//#line 93
{
  LedsP__Led2__clr();
  ;
//#line 95
  ;
}

//# 78 "/opt/tinyos-2.x/tos/interfaces/Leds.nc"
 /*inline*/  static void TestAppP__Leds__led2On(void ){
//#line 78
  LedsP__Leds__led2On();
//#line 78
}
//#line 78
//# 24 "/opt/tinyos-2.x/tos/system/NoPinC.nc"
static  /*inline*/  void /*PlatformLedsC.NoPinC*/NoPinC__0__GeneralIO__clr(void )
//#line 24
{
}

//# 30 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
 /*inline*/  static void LedsP__Led1__clr(void ){
//#line 30
  /*PlatformLedsC.NoPinC*/NoPinC__0__GeneralIO__clr();
//#line 30
}
//#line 30
//# 78 "/opt/tinyos-2.x/tos/system/LedsP.nc"
static  /*inline*/  void LedsP__Leds__led1On(void )
//#line 78
{
  LedsP__Led1__clr();
  ;
//#line 80
  ;
}

//# 61 "/opt/tinyos-2.x/tos/interfaces/Leds.nc"
 /*inline*/  static void TestAppP__Leds__led1On(void ){
//#line 61
  LedsP__Leds__led1On();
//#line 61
}
//#line 61
//# 92 "/opt/tinyos-2.x-contrib/diku/mcs51/tos/chips/mcs51/pins/HplMcs51GeneralIOC.nc"
static   /*inline*/   void HplMcs51GeneralIOC__P10__set(void )
//#line 92
{
//#line 92
  P1_0 = 1;
}

//# 29 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
 /*inline*/  static void /*PlatformLedsC.Led0_rev*/ReverseGPIOP__0__In__set(void ){
//#line 29
  HplMcs51GeneralIOC__P10__set();
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
 /*inline*/  static void TestAppP__Leds__led0On(void ){
//#line 45
  LedsP__Leds__led0On();
//#line 45
}
//#line 45
//# 98 "/opt/tinyos-2.x/tos/system/LedsP.nc"
static  /*inline*/  void LedsP__Leds__led2Off(void )
//#line 98
{
  LedsP__Led2__set();
  ;
//#line 100
  ;
}

//# 83 "/opt/tinyos-2.x/tos/interfaces/Leds.nc"
 /*inline*/  static void TestAppP__Leds__led2Off(void ){
//#line 83
  LedsP__Leds__led2Off();
//#line 83
}
//#line 83
//# 83 "/opt/tinyos-2.x/tos/system/LedsP.nc"
static  /*inline*/  void LedsP__Leds__led1Off(void )
//#line 83
{
  LedsP__Led1__set();
  ;
//#line 85
  ;
}

//# 66 "/opt/tinyos-2.x/tos/interfaces/Leds.nc"
 /*inline*/  static void TestAppP__Leds__led1Off(void ){
//#line 66
  LedsP__Leds__led1Off();
//#line 66
}
//#line 66
//# 68 "/opt/tinyos-2.x/tos/system/LedsP.nc"
static  /*inline*/  void LedsP__Leds__led0Off(void )
//#line 68
{
  LedsP__Led0__set();
  ;
//#line 70
  ;
}

//# 50 "/opt/tinyos-2.x/tos/interfaces/Leds.nc"
 /*inline*/  static void TestAppP__Leds__led0Off(void ){
//#line 50
  LedsP__Leds__led0Off();
//#line 50
}
//#line 50
//# 254 "TestAppP.nc"
static  /*inline*/  void TestAppP__consoleTask__runTask(void )
{
  uint16_t j;
  uint8_t *ptr;
  uint8_t _data[2];
//#line 258
  uint8_t tmp;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
//#line 260
    _data[0] = TestAppP__keyBuffer;
//#line 260
    __nesc_atomic_end(__nesc_atomic); }

  switch (_data[0]) {
      case '\r': 
        TestAppP__StdOut__print("\r\n");
      break;

      case 'l': 
        if (TestAppP__ledOn) {
            TestAppP__ledOn = FALSE;
            TestAppP__StdOut__print("Led off\n\r");
            TestAppP__Leds__led0Off();
            TestAppP__Leds__led1Off();
            TestAppP__Leds__led2Off();
          }
        else 
//#line 274
          {
            TestAppP__ledOn = TRUE;
            TestAppP__StdOut__print("Led on\n\r");
            TestAppP__Leds__led0On();
            TestAppP__Leds__led1On();
            TestAppP__Leds__led2On();
          }
      break;


      case 'q': 
        TestAppP__transmitPacketPtr->_data_seq_no = TestAppP__sequence++;

      TestAppP__StdOut__print("Transmitting packet: ");
      TestAppP__StdOut__dumpHex(TestAppP__transmitPacket, 18, " ");
      TestAppP__StdOut__print("\r\n");

      TestAppP__SimpleMac__sendPacket(TestAppP__transmitPacketPtr);
      break;

      case 'w': 
        if (TestAppP__radioOn) {
            TestAppP__radioOn = FALSE;
            TestAppP__receiverOn = FALSE;
            TestAppP__StdOut__print("Radio turned off\r\n");
            TestAppP__SimpleMacControl__stop();
          }
        else 
//#line 300
          {
            TestAppP__radioOn = TRUE;
            TestAppP__receiverOn = TRUE;
            TestAppP__StdOut__print("Radio turned on\r\n");
            TestAppP__SimpleMacControl__start();
          }
      break;

      case 'e': 
        if (!TestAppP__radioOn) {

            TestAppP__StdOut__print("Radio is off\r\n");
          }
        else {
//#line 313
          if (TestAppP__receiverOn) {
              TestAppP__receiverOn = FALSE;
              TestAppP__StdOut__print("Receiver turned off\r\n");
              TestAppP__SimpleMac__rxDisable();
            }
          else 
//#line 317
            {
              TestAppP__receiverOn = TRUE;
              TestAppP__StdOut__print("Receiver turned on\r\n");
              TestAppP__SimpleMac__rxEnable();
            }
          }
//#line 322
      break;

      case 'a': 
        TestAppP__channel = TestAppP__channel == 26 ? 11 : TestAppP__channel + 1;

      TestAppP__StdOut__print("Channel : ");
      TestAppP__StdOut__printBase10uint8(TestAppP__channel);
      TestAppP__StdOut__print("\n\r");

      TestAppP__SimpleMac__setChannel(TestAppP__channel);
      break;

      case 's': 
        TestAppP__channel = TestAppP__channel == 11 ? 26 : TestAppP__channel - 1;

      TestAppP__StdOut__print("Channel : ");
      TestAppP__StdOut__printBase10uint8(TestAppP__channel);
      TestAppP__StdOut__print("\n\r");

      TestAppP__SimpleMac__setChannel(TestAppP__channel);
      break;


      case 'r': 
        TestAppP__HalFlash__read(TestAppP__destination, (uint8_t xdata *)0x7B00, 256);

      for (j = 0; j < 0x0100; j++) {
          TestAppP__StdOut__printHex(TestAppP__destination[j]);
        }
      TestAppP__StdOut__print("\n\r");

      break;

      case 't': 

        TestAppP__StdOut__print("Writing segment\n\r");

      TestAppP__HalFlash__write(TestAppP__source, (uint8_t xdata *)0x7B00, 256);
      break;

      case 'y': 

        TestAppP__StdOut__print("Erasing segment\n\r");

      TestAppP__HalFlash__erase((uint8_t xdata *)0x7B00);
      break;

      case 'u': 


        TestAppP__StdOut__print("Reading adc\n\r");

      TestAppP__Read__read();
      break;

      case 'p': 
        TestAppP__transmitPacketPtr->length = TestAppP__transmitPacketPtr->length == 127 ? 13 : 127;
      TestAppP__StdOut__print("Length now: ");
      TestAppP__StdOut__printBase10uint8(TestAppP__transmitPacketPtr->length);
      TestAppP__StdOut__print("\n\r");
      break;

      default: 
        _data[1] = '\0';
      TestAppP__StdOut__print(_data);
      break;
    }
}

//# 56 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
 /*inline*/  static error_t HalCC2430RadioP__transmitTask__postTask(void ){
//#line 56
  unsigned char result;
//#line 56

//#line 56
  result = SchedulerBasicP__TaskBasic__postTask(HalCC2430RadioP__transmitTask);
//#line 56

//#line 56
  return result;
//#line 56
}
//#line 56
//# 218 "/opt/tinyos-2.x-contrib/diku/mcs51/tos/chips/cc2430/radio/HalCC2430RadioP.nc"
static  /*inline*/  error_t HalCC2430RadioP__HALCC2420__sendPacket(uint8_t *packet)
{
  HalCC2430RadioP__transmitPacketPtr = packet;
  HalCC2430RadioP__transmitTask__postTask();

  return SUCCESS;
}

//# 35 "/opt/tinyos-2.x-contrib/diku/common/lib/simplemac/HALCC2420.nc"
 /*inline*/  static error_t SimpleMacM__HALCC2420__sendPacket(uint8_t *packet){
//#line 35
  unsigned char result;
//#line 35

//#line 35
  result = HalCC2430RadioP__HALCC2420__sendPacket(packet);
//#line 35

//#line 35
  return result;
//#line 35
}
//#line 35
//# 56 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
 /*inline*/  static error_t HalCC2430RadioP__setChannelTask__postTask(void ){
//#line 56
  unsigned char result;
//#line 56

//#line 56
  result = SchedulerBasicP__TaskBasic__postTask(HalCC2430RadioP__setChannelTask);
//#line 56

//#line 56
  return result;
//#line 56
}
//#line 56
//# 186 "TestAppP.nc"
static  /*inline*/  void TestAppP__sendPacketTask__runTask(void )
{
  TestAppP__SimpleMacControl__start();
  TestAppP__SimpleMac__sendPacket(TestAppP__transmitPacketPtr);
  TestAppP__SimpleMacControl__stop();
}

//# 58 "/opt/tinyos-2.x/tos/types/TinyError.h"
static  /*inline*/   error_t ecombine(error_t r1, error_t r2)




{
  return r1 == r2 ? r1 : FAIL;
}

//# 99 "TestAppP.nc"
static  /*inline*/  error_t TestAppP__Init__init(void )
//#line 99
{
  uint16_t i;

  TestAppP__shortAddress = TOS_NODE_ID;
  TestAppP__transmitPacketPtr = (packet_t *)TestAppP__transmitPacket;


  TestAppP__transmitPacketPtr->length = 127;
  TestAppP__transmitPacketPtr->fcf = 0x0000;
  TestAppP__transmitPacketPtr->_data_seq_no = TestAppP__sequence++;
  TestAppP__transmitPacketPtr->dest = 0xFFFF;
  TestAppP__transmitPacketPtr->src = 0;








  TestAppP__transmitPacketPtr->fcs.rssi = 0;
  TestAppP__transmitPacketPtr->fcs.correlation = 0;


  for (i = 0; i < 256; i++) {
      TestAppP__source[i] = i;
    }

  return SUCCESS;
}

//# 56 "/opt/tinyos-2.x-contrib/diku/common/lib/stdout/StdOutM.nc"
static  /*inline*/  error_t StdOutM__Init__init(void )
//#line 56
{
  ;
  /* atomic removed: atomic calls only */
  {
    StdOutM__bufferhead = StdOutM__buffer;
    StdOutM__buffertail = StdOutM__buffer;
    StdOutM__bufferend = StdOutM__buffer + 1000;
    StdOutM__isOutputting = FALSE;
    StdOutM__count = 0;
  }
  return SUCCESS;
}

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

//# 56 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
 /*inline*/  static error_t SimpleMacM__initTask__postTask(void ){
//#line 56
  unsigned char result;
//#line 56

//#line 56
  result = SchedulerBasicP__TaskBasic__postTask(SimpleMacM__initTask);
//#line 56

//#line 56
  return result;
//#line 56
}
//#line 56
//# 49 "/opt/tinyos-2.x-contrib/diku/common/lib/simplemac/SimpleMacM.nc"
static  /*inline*/  error_t SimpleMacM__Init__init(void )
{
  SimpleMacM__initTask__postTask();
  SimpleMacM__receivedPacketPtr = &SimpleMacM__receivedPacket;

  return SUCCESS;
}

//# 56 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
 /*inline*/  static error_t HalCC2430RadioP__initTask__postTask(void ){
//#line 56
  unsigned char result;
//#line 56

//#line 56
  result = SchedulerBasicP__TaskBasic__postTask(HalCC2430RadioP__initTask);
//#line 56

//#line 56
  return result;
//#line 56
}
//#line 56
//# 105 "/opt/tinyos-2.x-contrib/diku/mcs51/tos/chips/cc2430/radio/HalCC2430RadioP.nc"
static  /*inline*/  error_t HalCC2430RadioP__Init__init(void )
{
  HalCC2430RadioP__receivedPacketPtr = HalCC2430RadioP__receivedPacket;
//#line 122
  HalCC2430RadioP__initTask__postTask();

  return SUCCESS;
}

//# 71 "/opt/tinyos-2.x-contrib/diku/mcs51/tos/chips/cc2430/flash/HalFlashP.nc"
static  /*inline*/  error_t HalFlashP__Init__init(void )
//#line 71
{

  HalFlashP__eraseFunction = (void (*)(void ))HalFlashP__eraseFunctionBuffer;
  HalFlashP__writeFunction = (void (*)(uint8_t *arg_0x7e7117e8, uint16_t arg_0x7e711998))HalFlashP__writeFunctionBuffer;

  return SUCCESS;
}

//# 61 "/opt/tinyos-2.x-contrib/diku/mcs51/tos/chips/cc2430/adc/AdcP.nc"
static  /*inline*/  error_t AdcP__Init__init(void )
//#line 61
{
  uint8_t i;

  for (i = 0; i < 1U; i++) {
      AdcP__inUse[i] = FALSE;
    }

  AdcP__counter = 0;

  return SUCCESS;
}

//# 51 "/opt/tinyos-2.x/tos/interfaces/Init.nc"
 /*inline*/  static error_t RealMainP__SoftwareInit__init(void ){
//#line 51
  unsigned char result;
//#line 51

//#line 51
  result = AdcP__Init__init();
//#line 51
  result = ecombine(result, HalFlashP__Init__init());
//#line 51
  result = ecombine(result, HalCC2430RadioP__Init__init());
//#line 51
  result = ecombine(result, SimpleMacM__Init__init());
//#line 51
  result = ecombine(result, HalCC2430SimpleUartP__Init__init());
//#line 51
  result = ecombine(result, StdOutM__Init__init());
//#line 51
  result = ecombine(result, TestAppP__Init__init());
//#line 51

//#line 51
  return result;
//#line 51
}
//#line 51
//# 74 "/opt/tinyos-2.x-contrib/diku/mcs51/tos/chips/cc2430/adc/AdcP.nc"
static  /*inline*/  void AdcP__AdcControl__enable(uint8_t id, uint8_t reference, uint8_t resolution, uint8_t input)
//#line 74
{


  if (AdcP__counter == 0) {
      ADCIE = 1;
      do {
//#line 79
          ADCCON1 |= 0x30;
        }
      while (
//#line 79
      0);
    }


  if (!AdcP__inUse[id]) {
      AdcP__inUse[id] = TRUE;
      AdcP__counter++;

      ADCCFG |= 0x01 << AdcP__inputs[id];
    }


  AdcP__references[id] = reference;
  AdcP__resolutions[id] = resolution;
  AdcP__inputs[id] = input;
}

//# 66 "/opt/tinyos-2.x-contrib/diku/mcs51/tos/chips/cc2430/adc/AdcControl.nc"
 /*inline*/  static void TestAppP__AdcControl__enable(uint8_t reference, uint8_t resolution, uint8_t input){
//#line 66
  AdcP__AdcControl__enable(/*TestAppC.AdcC*/AdcC__0__ID, reference, resolution, input);
//#line 66
}
//#line 66
//# 130 "TestAppP.nc"
static  /*inline*/  void TestAppP__Boot__booted(void )
//#line 130
{
  TestAppP__Leds__led0Off();
  TestAppP__Leds__led1Off();

  TestAppP__StdOut__print("Program initialized\n\r");

  TestAppP__channel = 11;
  TestAppP__SimpleMac__setChannel(TestAppP__channel);

  TestAppP__StdOut__print("Channel : ");
  TestAppP__StdOut__printBase10uint8(TestAppP__channel);
  TestAppP__StdOut__print("\n\r");

  TestAppP__radioOn = TRUE;
  TestAppP__receiverOn = TRUE;
  TestAppP__SimpleMacControl__start();


  TestAppP__AdcControl__enable(0x80, 0x30, 0x01);
}

//# 49 "/opt/tinyos-2.x/tos/interfaces/Boot.nc"
 /*inline*/  static void RealMainP__Boot__booted(void ){
//#line 49
  TestAppP__Boot__booted();
//#line 49
}
//#line 49
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
//# 56 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
 /*inline*/  static error_t TestAppP__consoleTask__postTask(void ){
//#line 56
  unsigned char result;
//#line 56

//#line 56
  result = SchedulerBasicP__TaskBasic__postTask(TestAppP__consoleTask);
//#line 56

//#line 56
  return result;
//#line 56
}
//#line 56
//# 92 "/opt/tinyos-2.x-contrib/diku/mcs51/tos/chips/mcs51/pins/HplMcs51GeneralIOC.nc"
static  /*inline*/  void HplMcs51GeneralIOC__P10__toggle(void )
//#line 92
{
  /* atomic removed: atomic calls only */
//#line 92
  {
//#line 92
    P1_0 = ~P1_0;
  }
}

//# 31 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
 /*inline*/  static void /*PlatformLedsC.Led0_rev*/ReverseGPIOP__0__In__toggle(void ){
//#line 31
  HplMcs51GeneralIOC__P10__toggle();
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
 /*inline*/  static void TestAppP__Leds__led0Toggle(void ){
//#line 56
  LedsP__Leds__led0Toggle();
//#line 56
}
//#line 56
//# 245 "TestAppP.nc"
static  /*inline*/  void TestAppP__StdOut__get(uint8_t _data)
//#line 245
{

  TestAppP__keyBuffer = _data;

  TestAppP__Leds__led0Toggle();

  TestAppP__consoleTask__postTask();
}

//# 870 "/opt/tinyos-2.x-contrib/diku/mcs51/tos/chips/cc2430/radio/HalCC2430RadioP.nc"
static  /*inline*/  void HalCC2430RadioP__StdOut__get(uint8_t _data)
{
}

//# 108 "/opt/tinyos-2.x-contrib/diku/common/lib/stdout/StdOut.nc"
 /*inline*/  static void StdOutM__StdOut__get(uint8_t _data){
//#line 108
  HalCC2430RadioP__StdOut__get(_data);
//#line 108
  TestAppP__StdOut__get(_data);
//#line 108
}
//#line 108
//# 545 "/opt/tinyos-2.x-contrib/diku/common/lib/stdout/StdOutM.nc"
static  /*inline*/  void StdOutM__UART__get(uint8_t _data)
//#line 545
{
  StdOutM__StdOut__get(_data);
  return;
}

//# 47 "/opt/tinyos-2.x/tos/lib/serial/SerialByteComm.nc"
 /*inline*/  static void HalCC2430SimpleUartP__uart0__get(uint8_t _data){
//#line 47
  StdOutM__UART__get(_data);
//#line 47
}
//#line 47
//# 130 "/opt/tinyos-2.x-contrib/diku/mcs51/tos/chips/cc2430/usart/HalCC2430SimpleUartP.nc"
static  /*inline*/  error_t HalCC2430SimpleUartP__uart0__put(uint8_t _data)
//#line 130
{
  U0BUF = _data;
  return SUCCESS;
}

//# 41 "/opt/tinyos-2.x/tos/lib/serial/SerialByteComm.nc"
 /*inline*/  static error_t StdOutM__UART__put(uint8_t _data){
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
//# 520 "/opt/tinyos-2.x-contrib/diku/common/lib/stdout/StdOutM.nc"
static  /*inline*/  void StdOutM__UART__putDone(void )
//#line 520
{

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
//#line 522
    {

      ++StdOutM__bufferhead;
      ++StdOutM__count;
      if (StdOutM__bufferhead == StdOutM__bufferend) {
          StdOutM__bufferhead = StdOutM__buffer;
        }

      if (StdOutM__bufferhead != StdOutM__buffertail) {

          StdOutM__UART__put(*StdOutM__bufferhead);
          StdOutM__isOutputting = TRUE;
        }
      else 
//#line 534
        {
          StdOutM__isOutputting = FALSE;
        }
    }
//#line 537
    __nesc_atomic_end(__nesc_atomic); }
  return;
}

//# 53 "/opt/tinyos-2.x/tos/lib/serial/SerialByteComm.nc"
 /*inline*/  static void HalCC2430SimpleUartP__uart0__putDone(void ){
//#line 53
  StdOutM__UART__putDone();
//#line 53
}
//#line 53
//# 627 "/opt/tinyos-2.x-contrib/diku/mcs51/tos/chips/cc2430/radio/HalCC2430RadioP.nc"
static  /*inline*/  void HalCC2430RadioP__InterruptRFErr__fired(void )
//#line 627
{

  HalCC2430RadioP__StdOut__print("MAC: Buffer overrun\r\n");





  if (!HalCC2430RadioP__receivedPacketTaskPosted && !HalCC2430RadioP__flushBufferTaskPosted) 
    {
      HalCC2430RadioP__flushBufferTask__postTask();
    }

  HalCC2430RadioP__flushBufferTaskPosted = TRUE;
}

//# 57 "/opt/tinyos-2.x/tos/interfaces/GpioInterrupt.nc"
 /*inline*/  static void HplCC2430InterruptsC__InterruptRFErr__fired(void ){
//#line 57
  HalCC2430RadioP__InterruptRFErr__fired();
//#line 57
}
//#line 57
//# 202 "/opt/tinyos-2.x-contrib/diku/mcs51/tos/chips/cc2430/radio/HplCC2430InterruptsC.nc"
static  /*inline*/  void HplCC2430InterruptsC__InterruptCCA__default__fired(void )
//#line 202
{
}

//# 57 "/opt/tinyos-2.x/tos/interfaces/GpioInterrupt.nc"
 /*inline*/  static void HplCC2430InterruptsC__InterruptCCA__fired(void ){
//#line 57
  HplCC2430InterruptsC__InterruptCCA__default__fired();
//#line 57
}
//#line 57
//# 500 "/opt/tinyos-2.x-contrib/diku/mcs51/tos/chips/cc2430/radio/HalCC2430RadioP.nc"
static  /*inline*/  void HalCC2430RadioP__InterruptFIFOP__fired(void )
//#line 500
{





  if (!HalCC2430RadioP__receivedPacketTaskPosted && * (uint8_t xdata *)0xDF53 > 0) {

      HalCC2430RadioP__receivedPacketTaskPosted = TRUE;
      HalCC2430RadioP__receivedPacketTask__postTask();
    }
}

//# 57 "/opt/tinyos-2.x/tos/interfaces/GpioInterrupt.nc"
 /*inline*/  static void HplCC2430InterruptsC__InterruptFIFOP__fired(void ){
//#line 57
  HalCC2430RadioP__InterruptFIFOP__fired();
//#line 57
}
//#line 57
//# 56 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
 /*inline*/  static error_t HalCC2430RadioP__sendPacketDoneTask__postTask(void ){
//#line 56
  unsigned char result;
//#line 56

//#line 56
  result = SchedulerBasicP__TaskBasic__postTask(HalCC2430RadioP__sendPacketDoneTask);
//#line 56

//#line 56
  return result;
//#line 56
}
//#line 56
//# 480 "/opt/tinyos-2.x-contrib/diku/mcs51/tos/chips/cc2430/radio/HalCC2430RadioP.nc"
static  /*inline*/  void HalCC2430RadioP__InterruptTXDone__fired(void )
{

  if (HalCC2430RadioP__transmitInProgress) {
      HalCC2430RadioP__transmitInProgress = FALSE;
      HalCC2430RadioP__sendPacketDoneTask__postTask();
    }
  else 
//#line 486
    {
      HalCC2430RadioP__StdOut__print("MAC: Unscheduled transmit\n");
    }
}

//# 57 "/opt/tinyos-2.x/tos/interfaces/GpioInterrupt.nc"
 /*inline*/  static void HplCC2430InterruptsC__InterruptTXDone__fired(void ){
//#line 57
  HalCC2430RadioP__InterruptTXDone__fired();
//#line 57
}
//#line 57
//# 200 "/opt/tinyos-2.x-contrib/diku/mcs51/tos/chips/cc2430/radio/HplCC2430InterruptsC.nc"
static  /*inline*/  void HplCC2430InterruptsC__CaptureSFD__default__captured(uint16_t time)
//#line 200
{
}

//# 50 "/opt/tinyos-2.x/tos/interfaces/GpioCapture.nc"
 /*inline*/  static void HplCC2430InterruptsC__CaptureSFD__captured(uint16_t time){
//#line 50
  HplCC2430InterruptsC__CaptureSFD__default__captured(time);
//#line 50
}
//#line 50
//# 56 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
 /*inline*/  static error_t AdcP__signalReadDone__postTask(void ){
//#line 56
  unsigned char result;
//#line 56

//#line 56
  result = SchedulerBasicP__TaskBasic__postTask(AdcP__signalReadDone);
//#line 56

//#line 56
  return result;
//#line 56
}
//#line 56
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

//#line 164
static void SchedulerBasicP__TaskBasic__default__runTask(uint8_t id)
{
}

//# 64 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static void SchedulerBasicP__TaskBasic__runTask(uint8_t arg_0x7ef73230){
//#line 64
  switch (arg_0x7ef73230) {
//#line 64
    case TestAppP__sendPacketTask:
//#line 64
      TestAppP__sendPacketTask__runTask();
//#line 64
      break;
//#line 64
    case TestAppP__consoleTask:
//#line 64
      TestAppP__consoleTask__runTask();
//#line 64
      break;
//#line 64
    case SimpleMacM__initTask:
//#line 64
      SimpleMacM__initTask__runTask();
//#line 64
      break;
//#line 64
    case SimpleMacM__signalSendPacketDone:
//#line 64
      SimpleMacM__signalSendPacketDone__runTask();
//#line 64
      break;
//#line 64
    case HalCC2430RadioP__initTask:
//#line 64
      HalCC2430RadioP__initTask__runTask();
//#line 64
      break;
//#line 64
    case HalCC2430RadioP__transmitTask:
//#line 64
      HalCC2430RadioP__transmitTask__runTask();
//#line 64
      break;
//#line 64
    case HalCC2430RadioP__setChannelTask:
//#line 64
      HalCC2430RadioP__setChannelTask__runTask();
//#line 64
      break;
//#line 64
    case HalCC2430RadioP__receivedPacketTask:
//#line 64
      HalCC2430RadioP__receivedPacketTask__runTask();
//#line 64
      break;
//#line 64
    case HalCC2430RadioP__flushBufferTask:
//#line 64
      HalCC2430RadioP__flushBufferTask__runTask();
//#line 64
      break;
//#line 64
    case HalCC2430RadioP__sendPacketDoneTask:
//#line 64
      HalCC2430RadioP__sendPacketDoneTask__runTask();
//#line 64
      break;
//#line 64
    case AdcP__signalReadDone:
//#line 64
      AdcP__signalReadDone__runTask();
//#line 64
      break;
//#line 64
    default:
//#line 64
      SchedulerBasicP__TaskBasic__default__runTask(arg_0x7ef73230);
//#line 64
      break;
//#line 64
    }
//#line 64
}
//#line 64
//# 118 "/opt/tinyos-2.x-contrib/diku/mcs51/tos/chips/cc2430/adc/AdcP.nc"
static error_t AdcP__Read__read(uint8_t id)
//#line 118
{


  if (AdcP__lastId < 1U) {
      return FAIL;
    }
  else 
//#line 123
    {
      uint8_t temp;


      AdcP__lastId = id;


      temp = ADCH;
      temp = ADCL;


      do {
//#line 134
          ADCCON3 = (AdcP__references[id] | AdcP__resolutions[id]) | AdcP__inputs[id];
        }
      while (
//#line 134
      0);

      return SUCCESS;
    }
}

//# 77 "/opt/tinyos-2.x-contrib/diku/common/lib/stdout/StdOutM.nc"
static int StdOutM__StdOut__print(const char *str)
//#line 77
{

  int na_countret;

//#line 80
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
//#line 80
    {
      bool return_flag = FALSE;
      int countret = 0;

//#line 83
      ;


      if (StdOutM__buffertail >= StdOutM__bufferhead) {
          while (StdOutM__buffertail < StdOutM__bufferend && *str != 0) {

              *StdOutM__buffertail = *str;
              ++StdOutM__buffertail;

              ++str;
              ++countret;
            }
//#line 94
          ;

          if (StdOutM__buffertail == StdOutM__bufferend) {
              StdOutM__buffertail = StdOutM__buffer;
            }
          else 
//#line 98
            {

              if (!StdOutM__isOutputting) {

                  StdOutM__UART__put(*StdOutM__bufferhead);
                  StdOutM__isOutputting = TRUE;
                }
              return_flag = TRUE;
            }
        }



      if (!return_flag) {


          while (StdOutM__buffertail < StdOutM__bufferhead && *str != 0) {
              *StdOutM__buffertail = *str;
              ++StdOutM__buffertail;
              ++str;
              ++countret;
            }
//#line 119
          ;

          if (!StdOutM__isOutputting) {
              StdOutM__UART__put(*StdOutM__bufferhead);
              StdOutM__isOutputting = TRUE;
            }

          if (StdOutM__buffertail == StdOutM__bufferhead) {
              if (!StdOutM__isOutputting) {

                  StdOutM__UART__put(*StdOutM__bufferhead);
                  StdOutM__isOutputting = TRUE;
                }
              return_flag = TRUE;
            }
        }


      if (!return_flag) {

          if (!StdOutM__isOutputting) {

              StdOutM__UART__put(*StdOutM__bufferhead);
              StdOutM__isOutputting = TRUE;
            }
          return_flag = TRUE;
        }

      na_countret = countret;
    }
//#line 148
    __nesc_atomic_end(__nesc_atomic); }

  return na_countret;
}

//# 159 "/opt/tinyos-2.x/tos/system/SchedulerBasicP.nc"
static error_t SchedulerBasicP__TaskBasic__postTask(uint8_t id)
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

//# 117 "/opt/tinyos-2.x-contrib/diku/mcs51/tos/chips/cc2430/radio/HplCC2430InterruptsC.nc"
static error_t HplCC2430InterruptsC__InterruptFIFOP__disable(void )
//#line 117
{
//#line 117
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
//#line 117
    {
//#line 117
      RFIM &= ~(1 << CC2430_RFIM_FIFOP);
    }
//#line 118
    __nesc_atomic_end(__nesc_atomic); }
//#line 117
  return SUCCESS;
}

//# 742 "/opt/tinyos-2.x-contrib/diku/mcs51/tos/chips/cc2430/radio/HalCC2430RadioP.nc"
static void HalCC2430RadioP__CC2430RxEnable(void )
{

  RFST = 0xE6;
  RFST = 0xE6;


  HalCC2430RadioP__InterruptFIFOP__enableRisingEdge();


  RFST = 0xE2;

  HalCC2430RadioP__rxEnabled = TRUE;
}

//# 155 "/opt/tinyos-2.x-contrib/diku/common/lib/stdout/StdOutM.nc"
static int StdOutM__StdOut__printHex(uint8_t c)
//#line 155
{
  char str[3];
  uint8_t v;


  v = (0xF0U & c) >> 4;
  if (v < 0xAU) {
      str[0] = v + '0';
    }
  else 
//#line 163
    {
      str[0] = v - 0xAU + 'A';
    }


  v = 0xFU & c;
  if (v < 0xAU) {
      str[1] = v + '0';
    }
  else 
//#line 171
    {
      str[1] = v - 0xAU + 'A';
    }
  str[2] = 0;

  return StdOutM__StdOut__print(str);
}



static int StdOutM__StdOut__printHexword(uint16_t c)
//#line 181
{
  return StdOutM__StdOut__printHex((0xFF00U & c) >> 8)
   + StdOutM__StdOut__printHex(0xFFU & c);
}

//# 828 "/opt/tinyos-2.x-contrib/diku/mcs51/tos/chips/cc2430/radio/HalCC2430RadioP.nc"
static void HalCC2430RadioP__CC2430TxWait(void )
{
  uint8_t i = 0;

  while (* (uint8_t xdata *)0xDF62 & (1 << CC2430_RFSTATUS_TX_ACTIVE) && i < 50) 
    {
      HalCC2430RadioP__wait(100);
      i++;
    }

  return;
}

//#line 773
static void HalCC2430RadioP__CC2430ChannelSet(uint8_t channel)
{
  uint16_t freq;


  freq = (uint16_t )channel - 11;
  freq *= 5;
  freq += 357;
  freq |= 0x4000;

  * (uint16_t xdata *)0xDF10 = freq;
}

//# 85 "/opt/tinyos-2.x-contrib/diku/common/lib/simplemac/SimpleMacM.nc"
static error_t SimpleMacM__SimpleMac__sendPacket(packet_t *packet)
{
  uint8_t i;
//#line 87
  uint8_t length;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
//#line 89
    SimpleMacM__sendPacketPtr = packet;
//#line 89
    __nesc_atomic_end(__nesc_atomic); }


  length = packet->length;

  SimpleMacM__transmitbuffer[0] = length;




  SimpleMacM__transmitbuffer[1] = packet->fcf & 0x00F8;
  SimpleMacM__transmitbuffer[2] = packet->fcf >> 8;

  SimpleMacM__transmitbuffer[3] = packet->_data_seq_no;

  SimpleMacM__transmitbuffer[4] = packet->dest & 0x00FF;
  SimpleMacM__transmitbuffer[5] = packet->dest >> 8;

  SimpleMacM__transmitbuffer[6] = *SimpleMacM__shortAddress & 0x00FF;
  SimpleMacM__transmitbuffer[7] = *SimpleMacM__shortAddress >> 8;

  for (i = 8; i < length - 1; i++) 
    {
      SimpleMacM__transmitbuffer[i] = packet->_data[i - 8];
    }

  SimpleMacM__transmitbuffer[length - 1] = 0;
  SimpleMacM__transmitbuffer[length] = 0;

  return SimpleMacM__HALCC2420__sendPacket(SimpleMacM__transmitbuffer);
}

//# 697 "/opt/tinyos-2.x-contrib/diku/mcs51/tos/chips/cc2430/radio/HalCC2430RadioP.nc"
static void HalCC2430RadioP__CC2430RFEnable(void )
{

  * (uint8_t xdata *)0xDF17 = 0x04;




  while (* (uint8_t xdata *)0xDF17 & 0x10) {
    }
}

//# 196 "/opt/tinyos-2.x-contrib/diku/common/lib/stdout/StdOutM.nc"
static int StdOutM__StdOut__printBase10uint8(const uint8_t c)
{
  bool print = 0;
  char str[4];
  uint8_t idx = 0;
//#line 200
  uint8_t tmp;
  uint32_t v;

  v = c;


  tmp = v / 100;
  if (tmp != 0 || print) {
      str[idx] = tmp + 48;
      idx++;
      v = v % 100;
      print = 1;
    }


  tmp = v / 10;
  if (tmp != 0 || print) {
      str[idx] = tmp + 48;
      idx++;
      v = v % 10;
      print = 1;
    }


  str[idx] = v + 48;
  idx++;

  str[idx] = 0;

  return StdOutM__StdOut__print(str);
}

//# 313 "/opt/tinyos-2.x-contrib/diku/mcs51/tos/chips/cc2430/radio/HalCC2430RadioP.nc"
static error_t HalCC2430RadioP__HALCC2420__setChannel(uint8_t channel)
{
  if (channel < 11 || channel > 26) {
    return FAIL;
    }
  else 
//#line 317
    {
      HalCC2430RadioP__currentChannel = channel;
      HalCC2430RadioP__setChannelTask__postTask();
    }

  return SUCCESS;
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

//# 142 "/opt/tinyos-2.x-contrib/diku/mcs51/tos/chips/cc2430/radio/HplCC2430InterruptsC.nc"
void __vector_0(void) interrupt 0
//#line 142
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
//#line 143
    {
      RFERRIF = 0;
      HplCC2430InterruptsC__InterruptRFErr__fired();
    }
//#line 146
    __nesc_atomic_end(__nesc_atomic); }
}

void __vector_16(void) interrupt 16
//#line 149
{





  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
//#line 155
    {
      uint8_t RFIF_RFIM = RFIF & RFIM;










      RFIF &= ~ (uint8_t )0x01;
      RFIF &= ~ (uint8_t )0x02;
      RFIF &= ~ (uint8_t )0x04;
      RFIF &= ~ (uint8_t )0x08;
      RFIF &= ~ (uint8_t )0x10;
      RFIF &= ~ (uint8_t )0x20;
      RFIF &= ~ (uint8_t )0x40;
      RFIF &= ~ (uint8_t )0x80;

      if (RFIF_RFIM & (1 << CC2430_RFIF_CCA)) {
          HplCC2430InterruptsC__InterruptCCA__fired();
        }

      if (RFIF_RFIM & (1 << CC2430_RFIF_FIFOP)) {
          HplCC2430InterruptsC__InterruptFIFOP__fired();
        }

      if (RFIF_RFIM & (1 << CC2430_RFIF_TXDONE)) {
          HplCC2430InterruptsC__InterruptTXDone__fired();
        }

      if (RFIF_RFIM & (1 << CC2430_RFIF_SFD)) {
          uint16_t now;

          (
//#line 190
          (uint8_t *)&now)[1] = T1CNTL;
          (
//#line 190
          (uint8_t *)&now)[0] = T1CNTH;
          HplCC2430InterruptsC__CaptureSFD__captured(now);
        }


      S1CON &= ~0x03;
    }
//#line 196
    __nesc_atomic_end(__nesc_atomic); }
}

//# 144 "/opt/tinyos-2.x-contrib/diku/mcs51/tos/chips/cc2430/adc/AdcP.nc"
void __vector_1(void) interrupt 1
//#line 144
{


  AdcP__value = (uint16_t )ADCH << 8;
  AdcP__value |= ADCL;

  AdcP__signalReadDone__postTask();
}

