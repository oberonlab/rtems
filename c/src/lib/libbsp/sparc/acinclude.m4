# RTEMS_CHECK_BSPDIR(RTEMS_BSP)
AC_DEFUN([RTEMS_CHECK_BSPDIR],
[
  case "$RTEMS_BSP_FAMILY" in
  erc32 )
    AC_CONFIG_SUBDIRS([erc32]);;
  leon )
    AC_CONFIG_SUBDIRS([leon]);;
  *)
    AC_MSG_ERROR([Invalid BSP]);;
  esac
])
