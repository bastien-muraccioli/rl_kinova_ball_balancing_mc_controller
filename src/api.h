#pragma once

#if defined _WIN32 || defined __CYGWIN__
#  define RLKinovaBallBalancingMcController_DLLIMPORT __declspec(dllimport)
#  define RLKinovaBallBalancingMcController_DLLEXPORT __declspec(dllexport)
#  define RLKinovaBallBalancingMcController_DLLLOCAL
#else
// On Linux, for GCC >= 4, tag symbols using GCC extension.
#  if __GNUC__ >= 4
#    define RLKinovaBallBalancingMcController_DLLIMPORT __attribute__((visibility("default")))
#    define RLKinovaBallBalancingMcController_DLLEXPORT __attribute__((visibility("default")))
#    define RLKinovaBallBalancingMcController_DLLLOCAL __attribute__((visibility("hidden")))
#  else
// Otherwise (GCC < 4 or another compiler is used), export everything.
#    define RLKinovaBallBalancingMcController_DLLIMPORT
#    define RLKinovaBallBalancingMcController_DLLEXPORT
#    define RLKinovaBallBalancingMcController_DLLLOCAL
#  endif // __GNUC__ >= 4
#endif // defined _WIN32 || defined __CYGWIN__

#ifdef RLKinovaBallBalancingMcController_STATIC
// If one is using the library statically, get rid of
// extra information.
#  define RLKinovaBallBalancingMcController_DLLAPI
#  define RLKinovaBallBalancingMcController_LOCAL
#else
// Depending on whether one is building or using the
// library define DLLAPI to import or export.
#  ifdef RLKinovaBallBalancingMcController_EXPORTS
#    define RLKinovaBallBalancingMcController_DLLAPI RLKinovaBallBalancingMcController_DLLEXPORT
#  else
#    define RLKinovaBallBalancingMcController_DLLAPI RLKinovaBallBalancingMcController_DLLIMPORT
#  endif // RLKinovaBallBalancingMcController_EXPORTS
#  define RLKinovaBallBalancingMcController_LOCAL RLKinovaBallBalancingMcController_DLLLOCAL
#endif // RLKinovaBallBalancingMcController_STATIC
