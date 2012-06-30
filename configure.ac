AC_INIT([libosmo-abis],
	m4_esyscmd([./git-version-gen .tarball-version]),
	[openbsc@lists.osmocom.org])

AM_INIT_AUTOMAKE([foreign dist-bzip2 no-dist-gzip 1.6])

dnl kernel style compile messages
m4_ifdef([AM_SILENT_RULES], [AM_SILENT_RULES([yes])])

dnl checks for programs
AC_PROG_MAKE_SET
AC_PROG_CC
AC_PROG_INSTALL
LT_INIT([pic-only])

AC_CONFIG_MACRO_DIR([m4])

dnl checks for header files
AC_HEADER_STDC
AC_CHECK_HEADERS(execinfo.h sys/select.h sys/socket.h syslog.h ctype.h)

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

PKG_CHECK_MODULES(LIBOSMOCORE, libosmocore >= 0.3.0)
PKG_CHECK_MODULES(LIBOSMOVTY, libosmovty >= 0.3.0)
PKG_CHECK_MODULES(LIBOSMOGSM, libosmogsm >= 0.3.10)
PKG_CHECK_MODULES(ORTP, ortp >= 0.13.1)

AC_CHECK_HEADERS(dahdi/user.h,,AC_MSG_WARN(DAHDI input driver will not be built))

AC_OUTPUT(
	libosmoabis.pc
	libosmotrau.pc
	include/Makefile
	src/Makefile
	tests/Makefile
	Makefile)
