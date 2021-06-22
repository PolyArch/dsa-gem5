#pragma once

// Don't forget to add to SSRegNames array
// ... someday I'll do the #macro trick to automate
// Yes! The magic is applied!

#include "dsa-ext/rf.h"

enum SSCmdIdx {
#define MACRO(x) SS_##x,
#include "ss_cmd.def"
#undef MACRO
};


const char* const SSCmdNames[] = {
#define MACRO(x) #x,
#include "ss_cmd.def"
#undef MACRO
};