#ifndef BOARDS_H
#define BOARDS_H

#include "nrf_gpio.h"
#include "nordic_common.h"
#if defined(GPSN)
  #include "gpsn_v2.h"
#elif defined(GATEWAY)
  #include "gateway_v2.h"
#elif defined(ROUTER)
  #include "router_v1.h"
#elif defined(CONTROLLER)
  #include "controller_v1.h"
#else
#error "Board is not defined"
#endif
#ifdef __cplusplus
extern "C" {
#endif


#ifdef __cplusplus
}
#endif

#endif
