#ifndef __SS_REGS_HH__
#define __SS_REGS_HH__

//Don't forget to add to SSRegNames array
//... someday I'll do the #macro trick to automate -- don't judge me
enum SSRegIdx {
   SS_MEM_ADDR, SS_INDEX_ADDR, SS_ACCESS_SIZE, SS_STRIDE, SS_SCRATCH_ADDR,
   SS_NUM_STRIDES, SS_NUM_ELEM, SS_CFG_SIZE, SS_STRETCH, 
   SS_NUM_BYTES, SS_CONSTANT, SS_IN_PORT, SS_OUT_PORT,
   SS_IND_PORT, SS_IND_TYPE, SS_DTYPE, SS_OFFSET_LIST, SS_IND_MULT,
   SS_WAIT_MASK, SS_SHIFT_BYTES, SS_GARBAGE, SS_GARB_ELEM, 
   SS_NUM_ELEM2, SS_CONSTANT2, SS_FLAGS, SS_CONTEXT, SS_REPEAT, SS_REPEAT_STRETCH,
   SS_OFFSET, SS_OPCODE, SS_VAL_PORT, SS_ADDR_TYPE, SS_IS_SCRATCH,
   SS_NUM_REG
};

const char* const SSRegNames[] = {
   "MEM_ADDR", "INDEX_ADDR", "ACCESS_SIZE", "STRIDE", "SCRATCH_ADDR",
   "NUM_STRIDES", "NUM_ELEM", "CFG_SIZE", "STRETCH",
   "NUM_BYTES", "CONSTANT", "IN_PORT", "OUT_PORT",
   "IND_PORT", "IND_TYPE", "DTYPE", "OFFSET_LIST", "IND_MULT",
   "WAIT_MASK", "SHIFT_BYTES", "GARBAGE", "SS_GARB_ELEM", 
   "SS_NUM_ELEM2", "SS_CONSTANT2", "SS_FLAGS", "SS_CONTEXT", "SS_REPEAT", "SS_REPEAT_STRETCH",
   "SS_OFFSET", "SS_OPCODE", "SS_VAL_PORT", "SS_ADDR_TYPE", "SS_IS_SCR",
   "NUM_REG"
  };

enum SSCmdIdx {
    SS_BEGIN_ROI,
    SS_END_ROI,
    SS_STATS,
    SS_CFG,
    SS_CFG_PORT,
    SS_CTX,
    SS_FILL_MODE,
    SS_MEM_SCR,
    SS_MEM_PRT,
    SS_SCR_PRT,
    SS_SCR_MEM,
    SS_PRT_SCR,
    SS_PRT_MEM,
    SS_PRT_PRT,
    SS_IND_PRT,
    SS_PRT_IND,
    SS_CNS_PRT,
    SS_SET_ITERS,
    SS_GARB,
    SS_ATOMIC_SCR_OP,
    SS_CONST_SCR,
    // SS_CONFIG_ATOMIC_SCR_OP,
    SS_WAIT,
    SS_NUM_CMDS
};


const char* const SSCmdNames[] = {
    "SS_BEGIN_ROI",
    "SS_END_ROI",
    "SS_STATS", 
    "SS_CFG",
    "SS_CFG_PORT",
    "SS_CTX",
    "SS_FILL_MODE",
    "SS_MEM_SCR",
    "SS_MEM_PRT",
    "SS_SCR_PRT",
    "SS_SCR_MEM",
    "SS_PRT_SCR",
    "SS_PRT_MEM",
    "SS_PRT_PRT",
    "SS_IND_PRT",
    "SS_PRT_IND",
    "SS_CNS_PRT",
    "SS_SET_ITERS",
    "SS_GARB",
    "SS_ATOMIC_SCR_OP",
    "SS_CONST_SCR",
    // "SS_CONFIG_ATOMIC_SCR_OP",
    "SS_WAIT",
    "SS_NUM_CMDS" };

#endif