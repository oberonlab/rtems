# RTEMS_CHECK_BSPDIR(RTEMS_BSP)
AC_DEFUN([RTEMS_CHECK_BSPDIR],
[
  case "$RTEMS_BSP_FAMILY" in
  h8sim )
    AC_CONFIG_SUBDIRS([h8sim]);;
  *)
    AC_MSG_ERROR([Invalid BSP]);;
  esac
])
