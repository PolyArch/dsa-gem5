#ifndef __SS_CONSTS_H__
#define __SS_CONSTS_H__

typedef uint64_t addr_t;

#define SBDT uint64_t           //cgra datatype
#define DATA_WIDTH sizeof(SBDT)
#define SCRATCH_SIZE (4096) //size in bytes -- 4KB

#define SB_TIMING

#define NUM_ACCEL 16
#define NUM_ACCEL_TOTAL (NUM_ACCEL+1)
#define SHARED_SP (NUM_ACCEL)
#define ACCEL_MASK 0xFFFF
#define SHARED_MASK (ACCEL_MASK+1)

#define MEM_WIDTH (64)
#define MEM_MASK ~(MEM_WIDTH-1)

#define SCR_WIDTH (64)
#define SCR_MASK ~(SCR_WIDTH-1)

#define PORT_WIDTH (64)
#define VP_LEN (64)

#define MAX_MEM_REQS (100)
#define CGRA_FIFO_LEN (32)

#define NUM_IN_PORTS  (32)
#define NUM_OUT_PORTS (32)

#define START_IND_PORTS (25)
#define STOP_IND_PORTS  (32)


#define MAX_WAIT (1000) //max cycles to wait for forward progress

#define SCR_STREAM (32)
#define MEM_WR_STREAM (33)
#define CONFIG_STREAM (34)

#define NUM_GROUPS (6)

//bit std::vectors for sb_wait
#define WAIT_SCR_WR   1 //wait for just scratch
#define WAIT_CMP      2 //wait for everything to complete
#define WAIT_SCR_RD   4 //wait for all reads to complete
#define WAIT_SCR_RD_Q 8 //wait for all reads to be de-queued
#define WAIT_MEM_WR   16//wait for all writes to complete

//fill modes
#define NO_FILL        0
#define POST_ZERO_FILL 1
#define PRE_ZERO_FILL  2
#define STRIDE_ZERO_FILL 3
#define STRIDE_DISCARD_FILL 4


#endif
