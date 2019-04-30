ACLOCAL_AMFLAGS = -I m4
AM_CFLAGS = -std=c99 -Wall -Werror
AM_CPPFLAGS = -I $(top_srcdir)/include

lib_LTLIBRARIES = libopenbci.la
libopenbci_la_SOURCES = src/openbci_c.c
include_HEADERS = include/openbci_c.h

noinst_PROGRAMS = test dump

test_LDADD = libopenbci.la
test_SOURCES = util/test.c

dump_LDADD = libopenbci.la
dump_SOURCES = util/dump.c
