#include "sim-debug.hh"


#undef DO_DBG
#define DO_DBG(x) bool SB_DEBUG::x = false;
#include "dbg.h"

std::string SB_DEBUG::verif_name = std::string("");
