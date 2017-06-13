#include "sim-debug.hh"


#undef DO_DBG
#define DO_DBG(x) bool DEBUG::x = false;
#include "dbg.h"

std::string DEBUG::verif_name = std::string("");
