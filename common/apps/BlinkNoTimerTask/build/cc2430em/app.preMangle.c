#define nx_struct struct
#define nx_union union
#define dbg(mode, format, ...) ((void)0)
#define dbg_clear(mode, format, ...) ((void)0)
#define dbg_active(mode) 0
# 151 "/usr/lib/gcc/i686-pc-cygwin/3.4.4/include/stddef.h" 3
typedef int ptrdiff_t;
#line 213
typedef unsigned int size_t;
#line 325
typedef short unsigned int wchar_t;
# 8 "/usr/lib/ncc/deputy_nodeputy.h"
struct __nesc_attr_nonnull {
}  ;
#line 9
struct __nesc_attr_bnd {
#line 9
  void *lo, *hi;
}  ;
#line 10
struct __nesc_attr_bnd_nok {
#line 10
  void *lo, *hi;
}  ;
#line 11
struct __nesc_attr_count {
#line 11
  int n;
}  ;
#line 12
struct __nesc_attr_count_nok {
#line 12
  int n;
}  ;
#line 13
struct __nesc_attr_one {
}  ;
#line 14
struct __nesc_attr_one_nok {
}  ;
#line 15
struct __nesc_attr_dmemset {
#line 15
  int a1, a2, a3;
}  ;
#line 16
struct __nesc_attr_dmemcpy {
#line 16
  int a1, a2, a3;
}  ;
#line 17
struct __nesc_attr_nts {
}  ;
# 47 "/opt/tinyos-2.x-contrib/diku/mcs51/tos/chips/mcs51/keil_stdint.h" 3
typedef signed char int8_t;
typedef short int16_t;



typedef long int32_t;


typedef long long int int64_t;




typedef unsigned char uint8_t;
typedef unsigned short uint16_t;



typedef unsigned long uint32_t;


typedef unsigned long long int uint64_t;
# 42 "/opt/tinyos-2.x-contrib/diku/mcs51/tos/chips/mcs51/stdint.h" 3
typedef signed char int_least8_t;
typedef short int int_least16_t;
typedef long int_least32_t;









typedef unsigned char uint_least8_t;
typedef unsigned short uint_least16_t;
typedef unsigned long uint_least32_t;
#line 70
typedef signed char int_fast8_t;





typedef int int_fast16_t;
typedef int int_fast32_t;





typedef unsigned char uint_fast8_t;





typedef unsigned int uint_fast16_t;
typedef unsigned int uint_fast32_t;
#line 105
typedef int intptr_t;


typedef unsigned int uintptr_t;








__extension__ 
typedef long long int intmax_t;
__extension__ 
typedef unsigned long long int uintmax_t;
# 231 "/usr/include/inttypes.h" 3
#line 228
typedef struct __nesc_unnamed4242 {
  intmax_t quot;
  intmax_t rem;
} imaxdiv_t;
# 385 "/usr/lib/ncc/nesc_nx.h"
typedef struct { unsigned char data[1]; } __attribute__((packed)) nx_int8_t;typedef int8_t __nesc_nxbase_nx_int8_t  ;
typedef struct { unsigned char data[2]; } __attribute__((packed)) nx_int16_t;typedef int16_t __nesc_nxbase_nx_int16_t  ;
typedef struct { unsigned char data[4]; } __attribute__((packed)) nx_int32_t;typedef int32_t __nesc_nxbase_nx_int32_t  ;
typedef struct { unsigned char data[8]; } __attribute__((packed)) nx_int64_t;typedef int64_t __nesc_nxbase_nx_int64_t  ;
typedef struct { unsigned char data[1]; } __attribute__((packed)) nx_uint8_t;typedef uint8_t __nesc_nxbase_nx_uint8_t  ;
typedef struct { unsigned char data[2]; } __attribute__((packed)) nx_uint16_t;typedef uint16_t __nesc_nxbase_nx_uint16_t  ;
typedef struct { unsigned char data[4]; } __attribute__((packed)) nx_uint32_t;typedef uint32_t __nesc_nxbase_nx_uint32_t  ;
typedef struct { unsigned char data[8]; } __attribute__((packed)) nx_uint64_t;typedef uint64_t __nesc_nxbase_nx_uint64_t  ;


typedef struct { unsigned char data[1]; } __attribute__((packed)) nxle_int8_t;typedef int8_t __nesc_nxbase_nxle_int8_t  ;
typedef struct { unsigned char data[2]; } __attribute__((packed)) nxle_int16_t;typedef int16_t __nesc_nxbase_nxle_int16_t  ;
typedef struct { unsigned char data[4]; } __attribute__((packed)) nxle_int32_t;typedef int32_t __nesc_nxbase_nxle_int32_t  ;
typedef struct { unsigned char data[8]; } __attribute__((packed)) nxle_int64_t;typedef int64_t __nesc_nxbase_nxle_int64_t  ;
typedef struct { unsigned char data[1]; } __attribute__((packed)) nxle_uint8_t;typedef uint8_t __nesc_nxbase_nxle_uint8_t  ;
typedef struct { unsigned char data[2]; } __attribute__((packed)) nxle_uint16_t;typedef uint16_t __nesc_nxbase_nxle_uint16_t  ;
typedef struct { unsigned char data[4]; } __attribute__((packed)) nxle_uint32_t;typedef uint32_t __nesc_nxbase_nxle_uint32_t  ;
typedef struct { unsigned char data[8]; } __attribute__((packed)) nxle_uint64_t;typedef uint64_t __nesc_nxbase_nxle_uint64_t  ;
# 13 "/opt/tinyos-2.x-contrib/diku/mcs51/tos/chips/mcs51/string.h"
extern void *memset(void *s, char val, int n);
# 14 "/usr/include/sys/lock.h" 3
typedef void *_LOCK_T;
# 14 "/usr/include/sys/_types.h" 3
typedef long _off_t;
__extension__ 
#line 15
typedef long long _off64_t;


typedef int _ssize_t;
# 354 "/usr/lib/gcc/i686-pc-cygwin/3.4.4/include/stddef.h" 3
typedef unsigned int wint_t;
# 35 "/usr/include/sys/_types.h" 3
#line 27
typedef struct __nesc_unnamed4243 {

  int __count;
  union __nesc_unnamed4244 {

    wint_t __wch;
    unsigned char __wchb[4];
  } __value;
} _mbstate_t;

typedef _LOCK_T _flock_t;


typedef void *_iconv_t;
# 19 "/usr/include/sys/reent.h" 3
typedef unsigned long __ULong;
#line 40
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
#line 166
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


  struct _reent *_data;



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

  struct _reent *_data;


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
#line 290
struct _rand48 {
  unsigned short _seed[3];
  unsigned short _mult[3];
  unsigned short _add;
};
#line 565
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
      __extension__ unsigned long long _rand_next;
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
#line 799
struct _reent;
struct _reent;









struct _reent;
# 32 "/usr/include/stdlib.h" 3
#line 28
typedef struct __nesc_unnamed4248 {

  int quot;
  int rem;
} div_t;





#line 34
typedef struct __nesc_unnamed4249 {

  long quot;
  long rem;
} ldiv_t;






#line 41
typedef struct __nesc_unnamed4250 {

  long long int quot;
  long long int rem;
} lldiv_t;
# 24 "/opt/tinyos-2.x-contrib/diku/common/tos/system/tos.h"
typedef uint8_t bool  ;
enum __nesc_unnamed4251 {
#line 25
  FALSE = 0, TRUE = 1
};









typedef nx_int8_t nx_bool;






struct __nesc_attr_atmostonce {
};
#line 44
struct __nesc_attr_atleastonce {
};
#line 45
struct __nesc_attr_exactlyonce {
};
# 40 "/opt/tinyos-2.x/tos/types/TinyError.h"
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
# 160 "/opt/tinyos-2.x-contrib/diku/mcs51/tos/chips/mcs51/io8051.h"
uint8_t volatile EA __attribute((sbitAT0xAF)) ;
#line 194
uint8_t volatile P1_0 __attribute((sbitAT0x90)) ;


uint8_t volatile P1_3 __attribute((sbitAT0x93)) ;
# 67 "/opt/tinyos-2.x-contrib/diku/mcs51/tos/chips/cc2430/ioCC2430.h"
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
#line 256
uint8_t volatile SLEEP __attribute((sfrAT0xBE)) ;







uint8_t volatile CLKCON __attribute((sfrAT0xC6)) ;
#line 352
uint8_t volatile P1_DIR __attribute((sfrAT0xFE)) ;
#line 393
typedef uint16_t uint16_t_xdata;
typedef uint8_t uint8_t_xdata;

typedef uint16_t uint16_t_code;
typedef uint8_t uint8_t_code;
#line 479
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
# 102 "/opt/tinyos-2.x-contrib/diku/mcs51/tos/chips/mcs51/mcs51hardware.h"
typedef uint8_t __nesc_atomic_t;

static __inline void __nesc_disable_interrupt();
static __inline void __nesc_enable_interrupt();

__inline __nesc_atomic_t __nesc_atomic_start(void )  ;





__inline void __nesc_atomic_end(__nesc_atomic_t oldSreg)  ;
# 71 "/opt/tinyos-2.x-contrib/diku/mcs51/tos/chips/cc2430/timer/CC2430Timer.h"
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
#line 240
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
# 42 "/opt/tinyos-2.x-contrib/diku/mcs51/tos/chips/mcs51/Timer.h"
typedef uint8_t TMilli;
typedef uint8_t T32khz;
typedef uint8_t TMicro;
# 32 "/opt/tinyos-2.x/tos/types/Leds.h"
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
# 51 "/opt/tinyos-2.x/tos/interfaces/Init.nc"
static error_t PlatformP$Init$init(void );
#line 51
static error_t RealMainP$SoftwareInit$default$init(void );
# 56 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static error_t SchedulerBasicP$TaskBasic$postTask(
# 45 "/opt/tinyos-2.x/tos/system/SchedulerBasicP.nc"
uint8_t arg_0x7ef72230);
# 64 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static void SchedulerBasicP$TaskBasic$default$runTask(
# 45 "/opt/tinyos-2.x/tos/system/SchedulerBasicP.nc"
uint8_t arg_0x7ef72230);
# 46 "/opt/tinyos-2.x/tos/interfaces/Scheduler.nc"
static void SchedulerBasicP$Scheduler$init(void );
#line 61
static void SchedulerBasicP$Scheduler$taskLoop(void );
#line 54
static bool SchedulerBasicP$Scheduler$runNextTask(void );
# 59 "/opt/tinyos-2.x/tos/interfaces/McuSleep.nc"
static void McuSleepC$McuSleep$sleep(void );
# 64 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static void BlinkNoTimerTaskC$delay$runTask(void );
# 49 "/opt/tinyos-2.x/tos/interfaces/Boot.nc"
static void BlinkNoTimerTaskC$Boot$booted(void );
# 64 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static void BlinkNoTimerTaskC$toggle$runTask(void );
# 51 "/opt/tinyos-2.x/tos/interfaces/Init.nc"
static error_t LedsP$Init$init(void );
# 56 "/opt/tinyos-2.x/tos/interfaces/Leds.nc"
static void LedsP$Leds$led0Toggle(void );




static void LedsP$Leds$led1On(void );










static void LedsP$Leds$led1Toggle(void );
#line 89
static void LedsP$Leds$led2Toggle(void );
#line 45
static void LedsP$Leds$led0On(void );
#line 78
static void LedsP$Leds$led2On(void );
# 31 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
static void HplMcs51GeneralIOC$P13$toggle(void );



static void HplMcs51GeneralIOC$P13$makeOutput(void );
#line 29
static void HplMcs51GeneralIOC$P13$set(void );
static void HplMcs51GeneralIOC$P13$clr(void );
static void HplMcs51GeneralIOC$P10$toggle(void );



static void HplMcs51GeneralIOC$P10$makeOutput(void );
#line 29
static void HplMcs51GeneralIOC$P10$set(void );
static void HplMcs51GeneralIOC$P10$clr(void );
static void /*PlatformLedsC.Led0_rev*/ReverseGPIOP$0$Out$toggle(void );



static void /*PlatformLedsC.Led0_rev*/ReverseGPIOP$0$Out$makeOutput(void );
#line 29
static void /*PlatformLedsC.Led0_rev*/ReverseGPIOP$0$Out$set(void );
static void /*PlatformLedsC.Led0_rev*/ReverseGPIOP$0$Out$clr(void );
static void /*PlatformLedsC.Led2_rev*/ReverseGPIOP$1$Out$toggle(void );



static void /*PlatformLedsC.Led2_rev*/ReverseGPIOP$1$Out$makeOutput(void );
#line 29
static void /*PlatformLedsC.Led2_rev*/ReverseGPIOP$1$Out$set(void );
static void /*PlatformLedsC.Led2_rev*/ReverseGPIOP$1$Out$clr(void );
static void /*PlatformLedsC.NoPinC*/NoPinC$0$GeneralIO$toggle(void );



static void /*PlatformLedsC.NoPinC*/NoPinC$0$GeneralIO$makeOutput(void );
#line 29
static void /*PlatformLedsC.NoPinC*/NoPinC$0$GeneralIO$set(void );
static void /*PlatformLedsC.NoPinC*/NoPinC$0$GeneralIO$clr(void );
# 51 "/opt/tinyos-2.x/tos/interfaces/Init.nc"
static error_t PlatformP$LedsInit$init(void );
# 48 "/opt/tinyos-2.x-contrib/diku/mcs51/tos/platforms/cc2430em/PlatformP.nc"
static inline error_t PlatformP$Init$init(void );
# 51 "/opt/tinyos-2.x/tos/interfaces/Init.nc"
static error_t RealMainP$SoftwareInit$init(void );
# 49 "/opt/tinyos-2.x/tos/interfaces/Boot.nc"
static void RealMainP$Boot$booted(void );
# 51 "/opt/tinyos-2.x/tos/interfaces/Init.nc"
static error_t RealMainP$PlatformInit$init(void );
# 46 "/opt/tinyos-2.x/tos/interfaces/Scheduler.nc"
static void RealMainP$Scheduler$init(void );
#line 61
static void RealMainP$Scheduler$taskLoop(void );
#line 54
static bool RealMainP$Scheduler$runNextTask(void );
# 52 "/opt/tinyos-2.x/tos/system/RealMainP.nc"
int main(void )   ;
#line 94
static inline error_t RealMainP$SoftwareInit$default$init(void );
# 64 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static void SchedulerBasicP$TaskBasic$runTask(
# 45 "/opt/tinyos-2.x/tos/system/SchedulerBasicP.nc"
uint8_t arg_0x7ef72230);
# 59 "/opt/tinyos-2.x/tos/interfaces/McuSleep.nc"
static void SchedulerBasicP$McuSleep$sleep(void );
# 50 "/opt/tinyos-2.x/tos/system/SchedulerBasicP.nc"
enum SchedulerBasicP$__nesc_unnamed4273 {

  SchedulerBasicP$NUM_TASKS = 2U, 
  SchedulerBasicP$NO_TASK = 255
};

uint8_t SchedulerBasicP$m_head;
uint8_t SchedulerBasicP$m_tail;
uint8_t SchedulerBasicP$m_next[SchedulerBasicP$NUM_TASKS];








static __inline uint8_t SchedulerBasicP$popTask(void );
#line 86
static inline bool SchedulerBasicP$isWaiting(uint8_t id);




static inline bool SchedulerBasicP$pushTask(uint8_t id);
#line 113
static inline void SchedulerBasicP$Scheduler$init(void );









static bool SchedulerBasicP$Scheduler$runNextTask(void );
#line 138
static inline void SchedulerBasicP$Scheduler$taskLoop(void );
#line 159
static error_t SchedulerBasicP$TaskBasic$postTask(uint8_t id);




static inline void SchedulerBasicP$TaskBasic$default$runTask(uint8_t id);
# 51 "/opt/tinyos-2.x-contrib/diku/mcs51/tos/chips/cc2430/McuSleepC.nc"
static inline void McuSleepC$McuSleep$sleep(void );
# 56 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static error_t BlinkNoTimerTaskC$delay$postTask(void );
#line 56
static error_t BlinkNoTimerTaskC$toggle$postTask(void );
# 56 "/opt/tinyos-2.x/tos/interfaces/Leds.nc"
static void BlinkNoTimerTaskC$Leds$led0Toggle(void );




static void BlinkNoTimerTaskC$Leds$led1On(void );










static void BlinkNoTimerTaskC$Leds$led1Toggle(void );
#line 89
static void BlinkNoTimerTaskC$Leds$led2Toggle(void );
#line 45
static void BlinkNoTimerTaskC$Leds$led0On(void );
#line 78
static void BlinkNoTimerTaskC$Leds$led2On(void );
# 48 "BlinkNoTimerTaskC.nc"
enum BlinkNoTimerTaskC$__nesc_unnamed4274 {
#line 48
  BlinkNoTimerTaskC$toggle = 0U
};
#line 48
typedef int BlinkNoTimerTaskC$__nesc_sillytask_toggle[BlinkNoTimerTaskC$toggle];
enum BlinkNoTimerTaskC$__nesc_unnamed4275 {
#line 49
  BlinkNoTimerTaskC$delay = 1U
};
#line 49
typedef int BlinkNoTimerTaskC$__nesc_sillytask_delay[BlinkNoTimerTaskC$delay];

static inline void BlinkNoTimerTaskC$Boot$booted(void );







static void BlinkNoTimerTaskC$delay$runTask(void );









static inline void BlinkNoTimerTaskC$toggle$runTask(void );
# 31 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
static void LedsP$Led0$toggle(void );



static void LedsP$Led0$makeOutput(void );
#line 29
static void LedsP$Led0$set(void );
static void LedsP$Led0$clr(void );
static void LedsP$Led1$toggle(void );



static void LedsP$Led1$makeOutput(void );
#line 29
static void LedsP$Led1$set(void );
static void LedsP$Led1$clr(void );
static void LedsP$Led2$toggle(void );



static void LedsP$Led2$makeOutput(void );
#line 29
static void LedsP$Led2$set(void );
static void LedsP$Led2$clr(void );
# 45 "/opt/tinyos-2.x/tos/system/LedsP.nc"
static inline error_t LedsP$Init$init(void );
#line 63
static inline void LedsP$Leds$led0On(void );









static inline void LedsP$Leds$led0Toggle(void );




static inline void LedsP$Leds$led1On(void );









static inline void LedsP$Leds$led1Toggle(void );




static inline void LedsP$Leds$led2On(void );









static inline void LedsP$Leds$led2Toggle(void );
# 92 "/opt/tinyos-2.x-contrib/diku/mcs51/tos/chips/mcs51/pins/HplMcs51GeneralIOC.nc"
static __inline void HplMcs51GeneralIOC$P10$set(void );
#line 92
static __inline void HplMcs51GeneralIOC$P10$clr(void );
#line 92
static inline void HplMcs51GeneralIOC$P10$toggle(void );
#line 92
static __inline void HplMcs51GeneralIOC$P10$makeOutput(void );


static __inline void HplMcs51GeneralIOC$P13$set(void );
#line 95
static __inline void HplMcs51GeneralIOC$P13$clr(void );
#line 95
static inline void HplMcs51GeneralIOC$P13$toggle(void );
#line 95
static __inline void HplMcs51GeneralIOC$P13$makeOutput(void );
# 31 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
static void /*PlatformLedsC.Led0_rev*/ReverseGPIOP$0$In$toggle(void );



static void /*PlatformLedsC.Led0_rev*/ReverseGPIOP$0$In$makeOutput(void );
#line 29
static void /*PlatformLedsC.Led0_rev*/ReverseGPIOP$0$In$set(void );
static void /*PlatformLedsC.Led0_rev*/ReverseGPIOP$0$In$clr(void );
# 59 "/opt/tinyos-2.x-contrib/diku/common/lib/ReverseGPIOP.nc"
static __inline void /*PlatformLedsC.Led0_rev*/ReverseGPIOP$0$Out$set(void );
#line 59
static __inline void /*PlatformLedsC.Led0_rev*/ReverseGPIOP$0$Out$clr(void );
#line 59
static inline void /*PlatformLedsC.Led0_rev*/ReverseGPIOP$0$Out$toggle(void );
#line 59
static __inline void /*PlatformLedsC.Led0_rev*/ReverseGPIOP$0$Out$makeOutput(void );
# 31 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
static void /*PlatformLedsC.Led2_rev*/ReverseGPIOP$1$In$toggle(void );



static void /*PlatformLedsC.Led2_rev*/ReverseGPIOP$1$In$makeOutput(void );
#line 29
static void /*PlatformLedsC.Led2_rev*/ReverseGPIOP$1$In$set(void );
static void /*PlatformLedsC.Led2_rev*/ReverseGPIOP$1$In$clr(void );
# 59 "/opt/tinyos-2.x-contrib/diku/common/lib/ReverseGPIOP.nc"
static __inline void /*PlatformLedsC.Led2_rev*/ReverseGPIOP$1$Out$set(void );
#line 59
static __inline void /*PlatformLedsC.Led2_rev*/ReverseGPIOP$1$Out$clr(void );
#line 59
static inline void /*PlatformLedsC.Led2_rev*/ReverseGPIOP$1$Out$toggle(void );
#line 59
static __inline void /*PlatformLedsC.Led2_rev*/ReverseGPIOP$1$Out$makeOutput(void );
# 23 "/opt/tinyos-2.x/tos/system/NoPinC.nc"
static inline void /*PlatformLedsC.NoPinC*/NoPinC$0$GeneralIO$set(void );
static inline void /*PlatformLedsC.NoPinC*/NoPinC$0$GeneralIO$clr(void );
static inline void /*PlatformLedsC.NoPinC*/NoPinC$0$GeneralIO$toggle(void );

static inline void /*PlatformLedsC.NoPinC*/NoPinC$0$GeneralIO$makeOutput(void );
# 107 "/opt/tinyos-2.x-contrib/diku/mcs51/tos/chips/mcs51/mcs51hardware.h"
__inline  __nesc_atomic_t __nesc_atomic_start(void )
#line 107
{
  __nesc_atomic_t tmp = EA;

#line 109
  EA = 0;
  return tmp;
}

__inline  void __nesc_atomic_end(__nesc_atomic_t oldSreg)
#line 113
{
  EA = oldSreg;
}

# 113 "/opt/tinyos-2.x/tos/system/SchedulerBasicP.nc"
static inline void SchedulerBasicP$Scheduler$init(void )
{
  /* atomic removed: atomic calls only */
  {
    memset((void *)SchedulerBasicP$m_next, SchedulerBasicP$NO_TASK, sizeof SchedulerBasicP$m_next);
    SchedulerBasicP$m_head = SchedulerBasicP$NO_TASK;
    SchedulerBasicP$m_tail = SchedulerBasicP$NO_TASK;
  }
}

# 46 "/opt/tinyos-2.x/tos/interfaces/Scheduler.nc"
inline static void RealMainP$Scheduler$init(void ){
#line 46
  SchedulerBasicP$Scheduler$init();
#line 46
}
#line 46
# 95 "/opt/tinyos-2.x-contrib/diku/mcs51/tos/chips/mcs51/pins/HplMcs51GeneralIOC.nc"
static __inline void HplMcs51GeneralIOC$P13$clr(void )
#line 95
{
#line 95
  P1_3 = 0;
}

# 30 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static void /*PlatformLedsC.Led2_rev*/ReverseGPIOP$1$In$clr(void ){
#line 30
  HplMcs51GeneralIOC$P13$clr();
#line 30
}
#line 30
# 59 "/opt/tinyos-2.x-contrib/diku/common/lib/ReverseGPIOP.nc"
static __inline void /*PlatformLedsC.Led2_rev*/ReverseGPIOP$1$Out$set(void )
#line 59
{
#line 59
  /*PlatformLedsC.Led2_rev*/ReverseGPIOP$1$In$clr();
}

# 29 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static void LedsP$Led2$set(void ){
#line 29
  /*PlatformLedsC.Led2_rev*/ReverseGPIOP$1$Out$set();
#line 29
}
#line 29
# 23 "/opt/tinyos-2.x/tos/system/NoPinC.nc"
static inline void /*PlatformLedsC.NoPinC*/NoPinC$0$GeneralIO$set(void )
#line 23
{
}

# 29 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static void LedsP$Led1$set(void ){
#line 29
  /*PlatformLedsC.NoPinC*/NoPinC$0$GeneralIO$set();
#line 29
}
#line 29
# 92 "/opt/tinyos-2.x-contrib/diku/mcs51/tos/chips/mcs51/pins/HplMcs51GeneralIOC.nc"
static __inline void HplMcs51GeneralIOC$P10$clr(void )
#line 92
{
#line 92
  P1_0 = 0;
}

# 30 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static void /*PlatformLedsC.Led0_rev*/ReverseGPIOP$0$In$clr(void ){
#line 30
  HplMcs51GeneralIOC$P10$clr();
#line 30
}
#line 30
# 59 "/opt/tinyos-2.x-contrib/diku/common/lib/ReverseGPIOP.nc"
static __inline void /*PlatformLedsC.Led0_rev*/ReverseGPIOP$0$Out$set(void )
#line 59
{
#line 59
  /*PlatformLedsC.Led0_rev*/ReverseGPIOP$0$In$clr();
}

# 29 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static void LedsP$Led0$set(void ){
#line 29
  /*PlatformLedsC.Led0_rev*/ReverseGPIOP$0$Out$set();
#line 29
}
#line 29
# 95 "/opt/tinyos-2.x-contrib/diku/mcs51/tos/chips/mcs51/pins/HplMcs51GeneralIOC.nc"
static __inline void HplMcs51GeneralIOC$P13$makeOutput(void )
#line 95
{
#line 95
  P1_DIR |= 1 << 3;
}

# 35 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static void /*PlatformLedsC.Led2_rev*/ReverseGPIOP$1$In$makeOutput(void ){
#line 35
  HplMcs51GeneralIOC$P13$makeOutput();
#line 35
}
#line 35
# 59 "/opt/tinyos-2.x-contrib/diku/common/lib/ReverseGPIOP.nc"
static __inline void /*PlatformLedsC.Led2_rev*/ReverseGPIOP$1$Out$makeOutput(void )
#line 59
{
#line 59
  /*PlatformLedsC.Led2_rev*/ReverseGPIOP$1$In$makeOutput();
}

# 35 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static void LedsP$Led2$makeOutput(void ){
#line 35
  /*PlatformLedsC.Led2_rev*/ReverseGPIOP$1$Out$makeOutput();
#line 35
}
#line 35
# 27 "/opt/tinyos-2.x/tos/system/NoPinC.nc"
static inline void /*PlatformLedsC.NoPinC*/NoPinC$0$GeneralIO$makeOutput(void )
#line 27
{
}

# 35 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static void LedsP$Led1$makeOutput(void ){
#line 35
  /*PlatformLedsC.NoPinC*/NoPinC$0$GeneralIO$makeOutput();
#line 35
}
#line 35
# 92 "/opt/tinyos-2.x-contrib/diku/mcs51/tos/chips/mcs51/pins/HplMcs51GeneralIOC.nc"
static __inline void HplMcs51GeneralIOC$P10$makeOutput(void )
#line 92
{
#line 92
  P1_DIR |= 1 << 0;
}

# 35 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static void /*PlatformLedsC.Led0_rev*/ReverseGPIOP$0$In$makeOutput(void ){
#line 35
  HplMcs51GeneralIOC$P10$makeOutput();
#line 35
}
#line 35
# 59 "/opt/tinyos-2.x-contrib/diku/common/lib/ReverseGPIOP.nc"
static __inline void /*PlatformLedsC.Led0_rev*/ReverseGPIOP$0$Out$makeOutput(void )
#line 59
{
#line 59
  /*PlatformLedsC.Led0_rev*/ReverseGPIOP$0$In$makeOutput();
}

# 35 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static void LedsP$Led0$makeOutput(void ){
#line 35
  /*PlatformLedsC.Led0_rev*/ReverseGPIOP$0$Out$makeOutput();
#line 35
}
#line 35
# 45 "/opt/tinyos-2.x/tos/system/LedsP.nc"
static inline error_t LedsP$Init$init(void )
#line 45
{
  /* atomic removed: atomic calls only */
#line 46
  {
    ;
    LedsP$Led0$makeOutput();
    LedsP$Led1$makeOutput();
    LedsP$Led2$makeOutput();
    LedsP$Led0$set();
    LedsP$Led1$set();
    LedsP$Led2$set();
  }
  return SUCCESS;
}

# 51 "/opt/tinyos-2.x/tos/interfaces/Init.nc"
inline static error_t PlatformP$LedsInit$init(void ){
#line 51
  unsigned char result;
#line 51

#line 51
  result = LedsP$Init$init();
#line 51

#line 51
  return result;
#line 51
}
#line 51
# 48 "/opt/tinyos-2.x-contrib/diku/mcs51/tos/platforms/cc2430em/PlatformP.nc"
static inline error_t PlatformP$Init$init(void )
#line 48
{
  uint8_t new_clkcon;

#line 88
  SLEEP = (SLEEP & ~CC2430_SLEEP_MODE_MASK) | CC2430_SLEEP_POWERMODE_0;



  SLEEP &= ~(1 << CC2430_SLEEP_OSC_PD);
  while (!(SLEEP & (1 << CC2430_SLEEP_XOSC_STB))) ;

  new_clkcon = 0x0;

  new_clkcon = (new_clkcon & ~CC2430_CLKCON_TICKSPD_MASK) | CC2430_TICKF_DIV_1;


  new_clkcon &= ~(1 << CC2430_CLKCON_OSC32K);






  new_clkcon &= ~(1 << CC2430_CLKCON_OSC);

  new_clkcon |= 1 << CC2430_CLKCON_TICKSPD;

  CLKCON = new_clkcon;
#line 137
  PlatformP$LedsInit$init();
  return SUCCESS;
}

# 51 "/opt/tinyos-2.x/tos/interfaces/Init.nc"
inline static error_t RealMainP$PlatformInit$init(void ){
#line 51
  unsigned char result;
#line 51

#line 51
  result = PlatformP$Init$init();
#line 51

#line 51
  return result;
#line 51
}
#line 51
# 54 "/opt/tinyos-2.x/tos/interfaces/Scheduler.nc"
inline static bool RealMainP$Scheduler$runNextTask(void ){
#line 54
  unsigned char result;
#line 54

#line 54
  result = SchedulerBasicP$Scheduler$runNextTask();
#line 54

#line 54
  return result;
#line 54
}
#line 54
# 56 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
inline static error_t BlinkNoTimerTaskC$toggle$postTask(void ){
#line 56
  unsigned char result;
#line 56

#line 56
  result = SchedulerBasicP$TaskBasic$postTask(BlinkNoTimerTaskC$toggle);
#line 56

#line 56
  return result;
#line 56
}
#line 56
# 86 "/opt/tinyos-2.x/tos/system/SchedulerBasicP.nc"
static inline bool SchedulerBasicP$isWaiting(uint8_t id)
{
  return SchedulerBasicP$m_next[id] != SchedulerBasicP$NO_TASK || SchedulerBasicP$m_tail == id;
}

static inline bool SchedulerBasicP$pushTask(uint8_t id)
{
  if (!SchedulerBasicP$isWaiting(id)) 
    {
      if (SchedulerBasicP$m_head == SchedulerBasicP$NO_TASK) 
        {
          SchedulerBasicP$m_head = id;
          SchedulerBasicP$m_tail = id;
        }
      else 
        {
          SchedulerBasicP$m_next[SchedulerBasicP$m_tail] = id;
          SchedulerBasicP$m_tail = id;
        }
      return TRUE;
    }
  else 
    {
      return FALSE;
    }
}

# 94 "/opt/tinyos-2.x/tos/system/RealMainP.nc"
static inline error_t RealMainP$SoftwareInit$default$init(void )
#line 94
{
#line 94
  return SUCCESS;
}

# 51 "/opt/tinyos-2.x/tos/interfaces/Init.nc"
inline static error_t RealMainP$SoftwareInit$init(void ){
#line 51
  unsigned char result;
#line 51

#line 51
  result = RealMainP$SoftwareInit$default$init();
#line 51

#line 51
  return result;
#line 51
}
#line 51
# 56 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
inline static error_t BlinkNoTimerTaskC$delay$postTask(void ){
#line 56
  unsigned char result;
#line 56

#line 56
  result = SchedulerBasicP$TaskBasic$postTask(BlinkNoTimerTaskC$delay);
#line 56

#line 56
  return result;
#line 56
}
#line 56
# 95 "/opt/tinyos-2.x-contrib/diku/mcs51/tos/chips/mcs51/pins/HplMcs51GeneralIOC.nc"
static __inline void HplMcs51GeneralIOC$P13$set(void )
#line 95
{
#line 95
  P1_3 = 1;
}

# 29 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static void /*PlatformLedsC.Led2_rev*/ReverseGPIOP$1$In$set(void ){
#line 29
  HplMcs51GeneralIOC$P13$set();
#line 29
}
#line 29
# 59 "/opt/tinyos-2.x-contrib/diku/common/lib/ReverseGPIOP.nc"
static __inline void /*PlatformLedsC.Led2_rev*/ReverseGPIOP$1$Out$clr(void )
#line 59
{
#line 59
  /*PlatformLedsC.Led2_rev*/ReverseGPIOP$1$In$set();
}

# 30 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static void LedsP$Led2$clr(void ){
#line 30
  /*PlatformLedsC.Led2_rev*/ReverseGPIOP$1$Out$clr();
#line 30
}
#line 30
# 93 "/opt/tinyos-2.x/tos/system/LedsP.nc"
static inline void LedsP$Leds$led2On(void )
#line 93
{
  LedsP$Led2$clr();
  ;
#line 95
  ;
}

# 78 "/opt/tinyos-2.x/tos/interfaces/Leds.nc"
inline static void BlinkNoTimerTaskC$Leds$led2On(void ){
#line 78
  LedsP$Leds$led2On();
#line 78
}
#line 78
# 24 "/opt/tinyos-2.x/tos/system/NoPinC.nc"
static inline void /*PlatformLedsC.NoPinC*/NoPinC$0$GeneralIO$clr(void )
#line 24
{
}

# 30 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static void LedsP$Led1$clr(void ){
#line 30
  /*PlatformLedsC.NoPinC*/NoPinC$0$GeneralIO$clr();
#line 30
}
#line 30
# 78 "/opt/tinyos-2.x/tos/system/LedsP.nc"
static inline void LedsP$Leds$led1On(void )
#line 78
{
  LedsP$Led1$clr();
  ;
#line 80
  ;
}

# 61 "/opt/tinyos-2.x/tos/interfaces/Leds.nc"
inline static void BlinkNoTimerTaskC$Leds$led1On(void ){
#line 61
  LedsP$Leds$led1On();
#line 61
}
#line 61
# 92 "/opt/tinyos-2.x-contrib/diku/mcs51/tos/chips/mcs51/pins/HplMcs51GeneralIOC.nc"
static __inline void HplMcs51GeneralIOC$P10$set(void )
#line 92
{
#line 92
  P1_0 = 1;
}

# 29 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static void /*PlatformLedsC.Led0_rev*/ReverseGPIOP$0$In$set(void ){
#line 29
  HplMcs51GeneralIOC$P10$set();
#line 29
}
#line 29
# 59 "/opt/tinyos-2.x-contrib/diku/common/lib/ReverseGPIOP.nc"
static __inline void /*PlatformLedsC.Led0_rev*/ReverseGPIOP$0$Out$clr(void )
#line 59
{
#line 59
  /*PlatformLedsC.Led0_rev*/ReverseGPIOP$0$In$set();
}

# 30 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static void LedsP$Led0$clr(void ){
#line 30
  /*PlatformLedsC.Led0_rev*/ReverseGPIOP$0$Out$clr();
#line 30
}
#line 30
# 63 "/opt/tinyos-2.x/tos/system/LedsP.nc"
static inline void LedsP$Leds$led0On(void )
#line 63
{
  LedsP$Led0$clr();
  ;
#line 65
  ;
}

# 45 "/opt/tinyos-2.x/tos/interfaces/Leds.nc"
inline static void BlinkNoTimerTaskC$Leds$led0On(void ){
#line 45
  LedsP$Leds$led0On();
#line 45
}
#line 45
# 51 "BlinkNoTimerTaskC.nc"
static inline void BlinkNoTimerTaskC$Boot$booted(void )
#line 51
{
  BlinkNoTimerTaskC$Leds$led0On();
  BlinkNoTimerTaskC$Leds$led1On();
  BlinkNoTimerTaskC$Leds$led2On();

  BlinkNoTimerTaskC$delay$postTask();
}

# 49 "/opt/tinyos-2.x/tos/interfaces/Boot.nc"
inline static void RealMainP$Boot$booted(void ){
#line 49
  BlinkNoTimerTaskC$Boot$booted();
#line 49
}
#line 49
# 95 "/opt/tinyos-2.x-contrib/diku/mcs51/tos/chips/mcs51/pins/HplMcs51GeneralIOC.nc"
static inline void HplMcs51GeneralIOC$P13$toggle(void )
#line 95
{
  /* atomic removed: atomic calls only */
#line 95
  {
#line 95
    P1_3 = ~P1_3;
  }
}

# 31 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static void /*PlatformLedsC.Led2_rev*/ReverseGPIOP$1$In$toggle(void ){
#line 31
  HplMcs51GeneralIOC$P13$toggle();
#line 31
}
#line 31
# 59 "/opt/tinyos-2.x-contrib/diku/common/lib/ReverseGPIOP.nc"
static inline void /*PlatformLedsC.Led2_rev*/ReverseGPIOP$1$Out$toggle(void )
#line 59
{
#line 59
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 59
    {
#line 59
      /*PlatformLedsC.Led2_rev*/ReverseGPIOP$1$In$toggle();
    }
#line 60
    __nesc_atomic_end(__nesc_atomic); }
}

# 31 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static void LedsP$Led2$toggle(void ){
#line 31
  /*PlatformLedsC.Led2_rev*/ReverseGPIOP$1$Out$toggle();
#line 31
}
#line 31
# 103 "/opt/tinyos-2.x/tos/system/LedsP.nc"
static inline void LedsP$Leds$led2Toggle(void )
#line 103
{
  LedsP$Led2$toggle();
  ;
#line 105
  ;
}

# 89 "/opt/tinyos-2.x/tos/interfaces/Leds.nc"
inline static void BlinkNoTimerTaskC$Leds$led2Toggle(void ){
#line 89
  LedsP$Leds$led2Toggle();
#line 89
}
#line 89
# 25 "/opt/tinyos-2.x/tos/system/NoPinC.nc"
static inline void /*PlatformLedsC.NoPinC*/NoPinC$0$GeneralIO$toggle(void )
#line 25
{
}

# 31 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static void LedsP$Led1$toggle(void ){
#line 31
  /*PlatformLedsC.NoPinC*/NoPinC$0$GeneralIO$toggle();
#line 31
}
#line 31
# 88 "/opt/tinyos-2.x/tos/system/LedsP.nc"
static inline void LedsP$Leds$led1Toggle(void )
#line 88
{
  LedsP$Led1$toggle();
  ;
#line 90
  ;
}

# 72 "/opt/tinyos-2.x/tos/interfaces/Leds.nc"
inline static void BlinkNoTimerTaskC$Leds$led1Toggle(void ){
#line 72
  LedsP$Leds$led1Toggle();
#line 72
}
#line 72
# 92 "/opt/tinyos-2.x-contrib/diku/mcs51/tos/chips/mcs51/pins/HplMcs51GeneralIOC.nc"
static inline void HplMcs51GeneralIOC$P10$toggle(void )
#line 92
{
  /* atomic removed: atomic calls only */
#line 92
  {
#line 92
    P1_0 = ~P1_0;
  }
}

# 31 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static void /*PlatformLedsC.Led0_rev*/ReverseGPIOP$0$In$toggle(void ){
#line 31
  HplMcs51GeneralIOC$P10$toggle();
#line 31
}
#line 31
# 59 "/opt/tinyos-2.x-contrib/diku/common/lib/ReverseGPIOP.nc"
static inline void /*PlatformLedsC.Led0_rev*/ReverseGPIOP$0$Out$toggle(void )
#line 59
{
#line 59
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 59
    {
#line 59
      /*PlatformLedsC.Led0_rev*/ReverseGPIOP$0$In$toggle();
    }
#line 60
    __nesc_atomic_end(__nesc_atomic); }
}

# 31 "/opt/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static void LedsP$Led0$toggle(void ){
#line 31
  /*PlatformLedsC.Led0_rev*/ReverseGPIOP$0$Out$toggle();
#line 31
}
#line 31
# 73 "/opt/tinyos-2.x/tos/system/LedsP.nc"
static inline void LedsP$Leds$led0Toggle(void )
#line 73
{
  LedsP$Led0$toggle();
  ;
#line 75
  ;
}

# 56 "/opt/tinyos-2.x/tos/interfaces/Leds.nc"
inline static void BlinkNoTimerTaskC$Leds$led0Toggle(void ){
#line 56
  LedsP$Leds$led0Toggle();
#line 56
}
#line 56
# 69 "BlinkNoTimerTaskC.nc"
static inline void BlinkNoTimerTaskC$toggle$runTask(void )
#line 69
{

  BlinkNoTimerTaskC$Leds$led0Toggle();
  BlinkNoTimerTaskC$Leds$led1Toggle();
  BlinkNoTimerTaskC$Leds$led2Toggle();
}

# 164 "/opt/tinyos-2.x/tos/system/SchedulerBasicP.nc"
static inline void SchedulerBasicP$TaskBasic$default$runTask(uint8_t id)
{
}

# 64 "/opt/tinyos-2.x/tos/interfaces/TaskBasic.nc"
inline static void SchedulerBasicP$TaskBasic$runTask(uint8_t arg_0x7ef72230){
#line 64
  switch (arg_0x7ef72230) {
#line 64
    case BlinkNoTimerTaskC$toggle:
#line 64
      BlinkNoTimerTaskC$toggle$runTask();
#line 64
      break;
#line 64
    case BlinkNoTimerTaskC$delay:
#line 64
      BlinkNoTimerTaskC$delay$runTask();
#line 64
      break;
#line 64
    default:
#line 64
      SchedulerBasicP$TaskBasic$default$runTask(arg_0x7ef72230);
#line 64
      break;
#line 64
    }
#line 64
}
#line 64
# 104 "/opt/tinyos-2.x-contrib/diku/mcs51/tos/chips/mcs51/mcs51hardware.h"
static __inline void __nesc_disable_interrupt()
#line 104
{
#line 104
  EA = 0;
}

#line 105
static __inline void __nesc_enable_interrupt()
#line 105
{
#line 105
  EA = 1;
}

# 51 "/opt/tinyos-2.x-contrib/diku/mcs51/tos/chips/cc2430/McuSleepC.nc"
static inline void McuSleepC$McuSleep$sleep(void )
#line 51
{

  __nesc_enable_interrupt();






  SLEEP = (SLEEP & ~CC2430_SLEEP_MODE_MASK) | CC2430_SLEEP_POWERMODE_0;

  __nesc_disable_interrupt();
}

# 59 "/opt/tinyos-2.x/tos/interfaces/McuSleep.nc"
inline static void SchedulerBasicP$McuSleep$sleep(void ){
#line 59
  McuSleepC$McuSleep$sleep();
#line 59
}
#line 59
# 67 "/opt/tinyos-2.x/tos/system/SchedulerBasicP.nc"
static __inline uint8_t SchedulerBasicP$popTask(void )
{
  if (SchedulerBasicP$m_head != SchedulerBasicP$NO_TASK) 
    {
      uint8_t id = SchedulerBasicP$m_head;

#line 72
      SchedulerBasicP$m_head = SchedulerBasicP$m_next[SchedulerBasicP$m_head];
      if (SchedulerBasicP$m_head == SchedulerBasicP$NO_TASK) 
        {
          SchedulerBasicP$m_tail = SchedulerBasicP$NO_TASK;
        }
      SchedulerBasicP$m_next[id] = SchedulerBasicP$NO_TASK;
      return id;
    }
  else 
    {
      return SchedulerBasicP$NO_TASK;
    }
}

#line 138
static inline void SchedulerBasicP$Scheduler$taskLoop(void )
{
  for (; ; ) 
    {
      uint8_t nextTask;

      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
        {
          while ((nextTask = SchedulerBasicP$popTask()) == SchedulerBasicP$NO_TASK) 
            {
              SchedulerBasicP$McuSleep$sleep();
            }
        }
#line 150
        __nesc_atomic_end(__nesc_atomic); }
      SchedulerBasicP$TaskBasic$runTask(nextTask);
    }
}

# 61 "/opt/tinyos-2.x/tos/interfaces/Scheduler.nc"
inline static void RealMainP$Scheduler$taskLoop(void ){
#line 61
  SchedulerBasicP$Scheduler$taskLoop();
#line 61
}
#line 61
# 52 "/opt/tinyos-2.x/tos/system/RealMainP.nc"
  int main(void )
#line 52
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    {





      {
      }
#line 60
      ;

      RealMainP$Scheduler$init();





      RealMainP$PlatformInit$init();
      while (RealMainP$Scheduler$runNextTask()) ;





      RealMainP$SoftwareInit$init();
      while (RealMainP$Scheduler$runNextTask()) ;
    }
#line 77
    __nesc_atomic_end(__nesc_atomic); }


  __nesc_enable_interrupt();

  RealMainP$Boot$booted();


  RealMainP$Scheduler$taskLoop();




  return -1;
}

# 123 "/opt/tinyos-2.x/tos/system/SchedulerBasicP.nc"
static bool SchedulerBasicP$Scheduler$runNextTask(void )
{
  uint8_t nextTask;

  /* atomic removed: atomic calls only */
#line 127
  {
    nextTask = SchedulerBasicP$popTask();
    if (nextTask == SchedulerBasicP$NO_TASK) 
      {
        {
          unsigned char __nesc_temp = 
#line 131
          FALSE;

#line 131
          return __nesc_temp;
        }
      }
  }
#line 134
  SchedulerBasicP$TaskBasic$runTask(nextTask);
  return TRUE;
}

# 59 "BlinkNoTimerTaskC.nc"
static void BlinkNoTimerTaskC$delay$runTask(void )
#line 59
{
  uint16_t i;
#line 60
  uint16_t j;

  for (i = 0; i < 0x1FFU; i++) {
      for (j = 0; j < 0xA0; j++) {
        }
    }
  BlinkNoTimerTaskC$toggle$postTask();
  BlinkNoTimerTaskC$delay$postTask();
}

# 159 "/opt/tinyos-2.x/tos/system/SchedulerBasicP.nc"
static error_t SchedulerBasicP$TaskBasic$postTask(uint8_t id)
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 161
    {
#line 161
      {
        unsigned char __nesc_temp = 
#line 161
        SchedulerBasicP$pushTask(id) ? SUCCESS : EBUSY;

        {
#line 161
          __nesc_atomic_end(__nesc_atomic); 
#line 161
          return __nesc_temp;
        }
      }
    }
#line 164
    __nesc_atomic_end(__nesc_atomic); }
}

