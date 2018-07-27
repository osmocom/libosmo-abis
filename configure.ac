AC_INIT([libosmo-abis],
	m4_esyscmd([./git-version-gen .tarball-version]),
	[openbsc@lists.osmocom.org])

dnl *This* is the root dir, even if an install-sh exists in ../ or ../../
AC_CONFIG_AUX_DIR([.])

AM_INIT_AUTOMAKE([foreign dist-bzip2 no-dist-gzip 1.6 subdir-objects])
AC_CONFIG_TESTDIR(tests)

dnl kernel style compile messages
m4_ifdef([AM_SILENT_RULES], [AM_SILENT_RULES([yes])])

dnl include release helper
RELMAKE='-include osmo-release.mk'
AC_SUBST([RELMAKE])

dnl checks for programs
AC_PROG_MAKE_SET
AC_PROG_CC
AC_PROG_INSTALL
LT_INIT([pic-only])

dnl check for pkg-config (explained in detail in libosmocore/configure.ac)
AC_PATH_PROG(PKG_CONFIG_INSTALLED, pkg-config, no)
if test "x$PKG_CONFIG_INSTALLED" = "xno"; then
        AC_MSG_WARN([You need to install pkg-config])
fi
PKG_PROG_PKG_CONFIG([0.20])

AC_CONFIG_MACRO_DIR([m4])

CFLAGS="$CFLAGS -Wall"
CPPFLAGS="$CPPFLAGS -Wall"

AC_ARG_ENABLE(sanitize,
	[AS_HELP_STRING(
		[--enable-sanitize],
		[Compile with address sanitizer enabled],
	)],
	[sanitize=$enableval], [sanitize="no"])
if test x"$sanitize" = x"yes"
then
	CFLAGS="$CFLAGS -fsanitize=address -fsanitize=undefined"
	CPPFLAGS="$CPPFLAGS -fsanitize=address -fsanitize=undefined"
fi

# The following test is taken from WebKit's webkit.m4
saved_CFLAGS="$CFLAGS"
CFLAGS="$CFLAGS -fvisibility=hidden "
AC_MSG_CHECKING([if ${CC} supports -fvisibility=hidden])
AC_COMPILE_IFELSE([AC_LANG_SOURCE([char foo;])],
      [ AC_MSG_RESULT([yes])
        SYMBOL_VISIBILITY="-fvisibility=hidden"],
        AC_MSG_RESULT([no]))
CFLAGS="$saved_CFLAGS"
AC_SUBST(SYMBOL_VISIBILITY)

dnl Generate the output
AM_CONFIG_HEADER(config.h)

PKG_CHECK_MODULES(LIBOSMOCORE, libosmocore >= 0.12.0)
PKG_CHECK_MODULES(LIBOSMOVTY, libosmovty >= 0.12.0)
PKG_CHECK_MODULES(LIBOSMOGSM, libosmogsm >= 0.12.0)
PKG_CHECK_MODULES(ORTP, ortp >= 0.22.0)

AC_CHECK_HEADERS(dahdi/user.h,,AC_MSG_WARN(DAHDI input driver will not be built))

AC_ARG_ENABLE(sanitize,
	[AS_HELP_STRING(
		[--enable-sanitize],
		[Compile with address sanitizer enabled],
	)],
	[sanitize=$enableval], [sanitize="no"])
if test x"$sanitize" = x"yes"
then
	CFLAGS="$CFLAGS -fsanitize=address -fsanitize=undefined"
	CPPFLAGS="$CPPFLAGS -fsanitize=address -fsanitize=undefined"
fi

AC_ARG_ENABLE(werror,
	[AS_HELP_STRING(
		[--enable-werror],
		[Turn all compiler warnings into errors, with exceptions:
		 a) deprecation (allow upstream to mark deprecation without breaking builds);
		 b) "#warning" pragmas (allow to remind ourselves of errors without breaking builds)
		]
	)],
	[werror=$enableval], [werror="no"])
if test x"$werror" = x"yes"
then
	WERROR_FLAGS="-Werror"
	WERROR_FLAGS+=" -Wno-error=deprecated -Wno-error=deprecated-declarations"
	WERROR_FLAGS+=" -Wno-error=cpp" # "#warning"
	CFLAGS="$CFLAGS $WERROR_FLAGS"
	CPPFLAGS="$CPPFLAGS $WERROR_FLAGS"
fi

_cflags_save=$CFLAGS
CFLAGS="$CFLAGS $ORTP_CFLAGS"
AC_COMPILE_IFELSE(
	[AC_LANG_PROGRAM(
		[[#include <ortp/ortp.h>]],
		[[ortp_set_log_level_mask(NULL, 0xffff);]]
	)],
	[AC_DEFINE([HAVE_ORTP_LOG_DOMAIN], [1],
		[ortp_set_log_level_mask requires domain parameter])],
	[AC_DEFINE([HAVE_ORTP_LOG_DOMAIN], [0],
		[ortp_set_log_level_mask has no domain parameter])])
CFLAGS=$_cflags_save

_cflags_save=$CFLAGS
CFLAGS="$CFLAGS $ORTP_CFLAGS -Werror"
AC_COMPILE_IFELSE(
	[AC_LANG_PROGRAM(
		[[#include <ortp/ortp.h>
		void fake_cb(struct _RtpSession *r, void *arg1, void *arg2, void *arg3) { return; }]],
		[[rtp_session_signal_connect(NULL, "", fake_cb, (void*) fake_cb);]]
	)],
	[AC_DEFINE([RTP_SIGNAL_PTR_CAST(arg)], [(void*)(arg)],
		[rtp_session_signal_connect requires pointer parameter])],
	[AC_DEFINE([RTP_SIGNAL_PTR_CAST(arg)], [(unsigned long)(arg)],
		[rtp_session_signal_connect requires ulong parameter])])
CFLAGS=$_cflags_save

AC_MSG_RESULT([CFLAGS="$CFLAGS"])
AC_MSG_RESULT([CPPFLAGS="$CPPFLAGS"])

AC_OUTPUT(
	libosmoabis.pc
	libosmotrau.pc
	include/Makefile
	src/Makefile
	tests/Makefile
	Makefile)
