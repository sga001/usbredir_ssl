AC_PREREQ(2.63)
AC_INIT([usbredir], [0.4.3])
AC_CONFIG_SRCDIR([configure.ac])

AM_CONFIG_HEADER([config.h])

dnl libtool versioning
# +1 : 0 : +1  == adds new functions to the interface
# +1 : 0 : 0   == changes or removes functions (changes include both
#                 changes to the signature and the semantic)
#  ? :+1 : ?   == just internal changes
# CURRENT : REVISION : AGE
LIBUSBREDIRHOST_SO_VERSION=1:0:0
AC_SUBST(LIBUSBREDIRHOST_SO_VERSION)

LIBUSBREDIRPARSER_SO_VERSION=0:0:0
AC_SUBST(LIBUSBREDIRPARSER_SO_VERSION)

AM_INIT_AUTOMAKE([foreign dist-bzip2 no-dist-gzip])
m4_ifdef([AM_SILENT_RULES],[AM_SILENT_RULES([yes])])

AC_PROG_CC
AC_HEADER_STDC
AC_PROG_INSTALL
AC_PROG_LN_S
LT_INIT
dnl AC_DEFINE(_GNU_SOURCE, [1], [Enable GNU extensions])

# Set some sane default CFLAGS, avoid having to do another release like 0.4.1
if test "$ac_test_CFLAGS" != set; then
  DEFAULT_CFLAGS="-Wall -Werror -Wp,-D_FORTIFY_SOURCE=2 -fstack-protector --param=ssp-buffer-size=4"
  for F in $DEFAULT_CFLAGS; do
    AC_MSG_CHECKING([whether $CC supports $F])
    save_CFLAGS="$CFLAGS"
    CFLAGS="$CFLAGS $F"
    AC_COMPILE_IFELSE([AC_LANG_SOURCE([ ])], [cc_flag=yes], [cc_flag=no])
    if test "x$cc_flag" != "xyes"; then
      CFLAGS="$save_CFLAGS"
    fi
    AC_MSG_RESULT([$cc_flag])
  done
fi

PKG_PROG_PKG_CONFIG
PKG_CHECK_MODULES(LIBUSB, [libusb-1.0 >= 1.0.9])

AC_CONFIG_FILES([
Makefile
usbredirhost/Makefile
usbredirhost/libusbredirhost.pc
usbredirparser/Makefile
usbredirparser/libusbredirparser.pc
usbredirserver/Makefile
usbredirtestclient/Makefile
])
AC_OUTPUT
