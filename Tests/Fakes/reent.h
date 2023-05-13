
// #include <_ansi.h>
#include <stddef.h>
// #include <sys/_types.h>

#ifdef _REENT_GLOBAL_ATEXIT
#define _REENT_INIT_ATEXIT
#else
#define _REENT_INIT_ATEXIT _NULL, _ATEXIT_INIT,
#endif

// /* This version of _reent is laid out with "int"s in pairs, to help
//  * ports with 16-bit int's but 32-bit pointers, align nicely.  */
struct _reent {
  /* As an exception to the above put _errno first for binary
     compatibility with non _REENT_SMALL targets.  */
  int _errno; /* local copy of errno */

//   /* FILE is a big struct and may change over time.  To try to achieve binary
//      compatibility with future versions, put stdin,stdout,stderr here.
//      These are pointers into member __sf defined below.  */
//   __FILE *_stdin, *_stdout, *_stderr; /* XXX */

//   int _inc; /* used by tmpnam */

//   char *_emergency;

//   int __sdidinit; /* 1 means stdio has been init'd */

//   int _unspecified_locale_info; /* unused, reserved for locale stuff */
//   struct __locale_t *_locale;   /* per-thread locale */

//   struct _mprec *_mp;

//   void (*__cleanup)(struct _reent *);

//   int _gamma_signgam;

//   /* used by some fp conversion routines */
//   int _cvtlen; /* should be size_t */
//   char *_cvtbuf;

//   struct _rand48 *_r48;
//   struct __tm *_localtime_buf;
//   char *_asctime_buf;

//   /* signal info */
//   void (**(_sig_func))(int);

// #ifndef _REENT_GLOBAL_ATEXIT
//   /* atexit stuff */
//   struct _atexit *_atexit;
//   struct _atexit _atexit0;
// #endif

//   struct _glue __sglue;      /* root of glue chain */
//   __FILE *__sf;              /* file descriptors */
//   struct _misc_reent *_misc; /* strtok, multibyte states */
//   char *_signal_buf;         /* strsignal */
};
