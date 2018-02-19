#ifndef __SIM_SB_DEBUG_H__
#define __SIM_SB_DEBUG_H__
#include <string>
#include <cstring>
#include <iostream>

//Header-specific debugs
class SB_DEBUG {
public:
  #undef DO_DBG
  #define DO_DBG(x) static bool x;
  #include "dbg.h"
  static std::string verif_name;

 #undef DO_DBG
 #define DO_DBG(x) if(getenv(#x)) \
  {x=std::string(getenv(#x))!="0"; \
    if(x && strcmp(#x,"SUPRESS_SB_STATS")!=0) \
    {std::cout << "SB_DEBUG VAR: " << #x << " IS ON\n";}}

 static void check_env() {
   #include "dbg.h"
   if(getenv("SB_VERIF_NAME")) {
     verif_name = std::string(getenv("SB_VERIF_NAME"));
   }
 } 
};

#endif
