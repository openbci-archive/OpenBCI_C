AC_INIT([libopenbci], [0.1])
AC_CONFIG_AUX_DIR([config])
AC_CONFIG_MACRO_DIR([m4])

AM_INIT_AUTOMAKE([-Wall -Werror foreign subdir-objects])

# magic for microsoft support needed by 1.12
m4_ifdef([AM_PROG_AR], [AM_PROG_AR])

AC_PROG_CC
LT_INIT

AC_OUTPUT(Makefile)
