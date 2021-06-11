/*
 * Copyright (c) 2011-2014, 2016-2018, 2020 ARM Limited
 * Copyright (c) 2013 Advanced Micro Devices, Inc.
 * All rights reserved
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder.  You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
 *
 * Copyright (c) 2002-2005 The Regents of The University of Michigan
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file
 *
 *  ExecContext bears the exec_context interface for Minor.
 */

#ifndef __CPU_MINOR_EXEC_CONTEXT_HH__
#define __CPU_MINOR_EXEC_CONTEXT_HH__

#include "cpu/base.hh"
#include "cpu/exec_context.hh"
#include "cpu/minor/execute.hh"
#include "cpu/minor/pipeline.hh"
#include "cpu/simple_thread.hh"
#include "ssim.hh"
#include "debug/MinorExecute.hh"
#include "debug/SS.hh"
#include "mem/request.hh"
#include "cpu/dsa_commands.hh"
#include "dsa/debug.h"
#include "dsa/rf.h"

namespace Minor
{

/* Forward declaration of Execute */
class Execute;

/** ExecContext bears the exec_context interface for Minor.  This nicely
 *  separates that interface from other classes such as Pipeline, MinorCPU
 *  and DynMinorInst and makes it easier to see what state is accessed by it.
 */
class ExecContext : public ::ExecContext
{
  public:
    MinorCPU &cpu;

    /** ThreadState object, provides all the architectural state. */
    SimpleThread &thread;

    /** The execute stage so we can peek at its contents. */
    Execute &execute;

    /** Instruction for the benefit of memory operations and for PC */
    MinorDynInstPtr inst;

    ExecContext (
        MinorCPU &cpu_,
        SimpleThread &thread_,
        Execute &execute_,
        MinorDynInstPtr inst_) :
        cpu(cpu_),
        thread(thread_),
        execute(execute_),
        inst(inst_)
    {
        DPRINTF(MinorExecute, "ExecContext setting PC: %s\n", inst->pc);
        pcState(inst->pc);
        setPredicate(inst->readPredicate());
        setMemAccPredicate(inst->readMemAccPredicate());
        thread.setIntReg(TheISA::ZeroReg, 0);
    }

    ~ExecContext()
    {
        inst->setPredicate(readPredicate());
        inst->setMemAccPredicate(readMemAccPredicate());
    }

    Fault
    initiateMemRead(Addr addr, unsigned int size,
                    Request::Flags flags,
                    const std::vector<bool>& byte_enable =
                        std::vector<bool>()) override
    {
        assert(byte_enable.empty() || byte_enable.size() == size);
        return execute.getLSQ().pushRequest(inst, true /* load */, nullptr,
            size, addr, flags, nullptr, nullptr, byte_enable);
    }

    Fault
    writeMem(uint8_t *data, unsigned int size, Addr addr,
             Request::Flags flags, uint64_t *res,
             const std::vector<bool>& byte_enable = std::vector<bool>())
        override
    {
        assert(byte_enable.empty() || byte_enable.size() == size);
        return execute.getLSQ().pushRequest(inst, false /* store */, data,
            size, addr, flags, res, nullptr, byte_enable);
    }

    Fault
    initiateMemAMO(Addr addr, unsigned int size, Request::Flags flags,
                   AtomicOpFunctorPtr amo_op) override
    {
        // AMO requests are pushed through the store path
        return execute.getLSQ().pushRequest(inst, false /* amo */, nullptr,
            size, addr, flags, nullptr, std::move(amo_op));
    }

    RegVal
    readIntRegOperand(const StaticInst *si, int idx) override
    {
        const RegId& reg = si->srcRegIdx(idx);
        assert(reg.isIntReg());
        return thread.readIntReg(reg.index());
    }

    RegVal
    readFloatRegOperandBits(const StaticInst *si, int idx) override
    {
        const RegId& reg = si->srcRegIdx(idx);
        assert(reg.isFloatReg());
        return thread.readFloatReg(reg.index());
    }

    const TheISA::VecRegContainer &
    readVecRegOperand(const StaticInst *si, int idx) const override
    {
        const RegId& reg = si->srcRegIdx(idx);
        assert(reg.isVecReg());
        return thread.readVecReg(reg);
    }

    TheISA::VecRegContainer &
    getWritableVecRegOperand(const StaticInst *si, int idx) override
    {
        const RegId& reg = si->destRegIdx(idx);
        assert(reg.isVecReg());
        return thread.getWritableVecReg(reg);
    }

    TheISA::VecElem
    readVecElemOperand(const StaticInst *si, int idx) const override
    {
        const RegId& reg = si->srcRegIdx(idx);
        assert(reg.isVecElem());
        return thread.readVecElem(reg);
    }

    const TheISA::VecPredRegContainer&
    readVecPredRegOperand(const StaticInst *si, int idx) const override
    {
        const RegId& reg = si->srcRegIdx(idx);
        assert(reg.isVecPredReg());
        return thread.readVecPredReg(reg);
    }

    TheISA::VecPredRegContainer&
    getWritableVecPredRegOperand(const StaticInst *si, int idx) override
    {
        const RegId& reg = si->destRegIdx(idx);
        assert(reg.isVecPredReg());
        return thread.getWritableVecPredReg(reg);
    }

    void
    setIntRegOperand(const StaticInst *si, int idx, RegVal val) override
    {
        const RegId& reg = si->destRegIdx(idx);
        assert(reg.isIntReg());
        thread.setIntReg(reg.index(), val);
    }

    void
    setFloatRegOperandBits(const StaticInst *si, int idx, RegVal val) override
    {
        const RegId& reg = si->destRegIdx(idx);
        assert(reg.isFloatReg());
        thread.setFloatReg(reg.index(), val);
    }

    void
    setVecRegOperand(const StaticInst *si, int idx,
                     const TheISA::VecRegContainer& val) override
    {
        const RegId& reg = si->destRegIdx(idx);
        assert(reg.isVecReg());
        thread.setVecReg(reg, val);
    }

    void
    setVecPredRegOperand(const StaticInst *si, int idx,
                         const TheISA::VecPredRegContainer& val) override
    {
        const RegId& reg = si->destRegIdx(idx);
        assert(reg.isVecPredReg());
        thread.setVecPredReg(reg, val);
    }

    /** Vector Register Lane Interfaces. */
    /** @{ */
    /** Reads source vector 8bit operand. */
    ConstVecLane8
    readVec8BitLaneOperand(const StaticInst *si, int idx) const
                            override
    {
        const RegId& reg = si->srcRegIdx(idx);
        assert(reg.isVecReg());
        return thread.readVec8BitLaneReg(reg);
    }

    /** Reads source vector 16bit operand. */
    ConstVecLane16
    readVec16BitLaneOperand(const StaticInst *si, int idx) const
                            override
    {
        const RegId& reg = si->srcRegIdx(idx);
        assert(reg.isVecReg());
        return thread.readVec16BitLaneReg(reg);
    }

    /** Reads source vector 32bit operand. */
    ConstVecLane32
    readVec32BitLaneOperand(const StaticInst *si, int idx) const
                            override
    {
        const RegId& reg = si->srcRegIdx(idx);
        assert(reg.isVecReg());
        return thread.readVec32BitLaneReg(reg);
    }

    /** Reads source vector 64bit operand. */
    ConstVecLane64
    readVec64BitLaneOperand(const StaticInst *si, int idx) const
                            override
    {
        const RegId& reg = si->srcRegIdx(idx);
        assert(reg.isVecReg());
        return thread.readVec64BitLaneReg(reg);
    }

    /** Write a lane of the destination vector operand. */
    template <typename LD>
    void
    setVecLaneOperandT(const StaticInst *si, int idx, const LD& val)
    {
        const RegId& reg = si->destRegIdx(idx);
        assert(reg.isVecReg());
        return thread.setVecLane(reg, val);
    }
    virtual void
    setVecLaneOperand(const StaticInst *si, int idx,
            const LaneData<LaneSize::Byte>& val) override
    {
        setVecLaneOperandT(si, idx, val);
    }
    virtual void
    setVecLaneOperand(const StaticInst *si, int idx,
            const LaneData<LaneSize::TwoByte>& val) override
    {
        setVecLaneOperandT(si, idx, val);
    }
    virtual void
    setVecLaneOperand(const StaticInst *si, int idx,
            const LaneData<LaneSize::FourByte>& val) override
    {
        setVecLaneOperandT(si, idx, val);
    }
    virtual void
    setVecLaneOperand(const StaticInst *si, int idx,
            const LaneData<LaneSize::EightByte>& val) override
    {
        setVecLaneOperandT(si, idx, val);
    }
    /** @} */

    void
    setVecElemOperand(const StaticInst *si, int idx,
                      const TheISA::VecElem val) override
    {
        const RegId& reg = si->destRegIdx(idx);
        assert(reg.isVecElem());
        thread.setVecElem(reg, val);
    }

    bool
    readPredicate() const override
    {
        return thread.readPredicate();
    }

    void
    setPredicate(bool val) override
    {
        thread.setPredicate(val);
    }

    bool
    readMemAccPredicate() const override
    {
        return thread.readMemAccPredicate();
    }

    void
    setMemAccPredicate(bool val) override
    {
        thread.setMemAccPredicate(val);
    }

    TheISA::PCState
    pcState() const override
    {
        return thread.pcState();
    }

    void
    pcState(const TheISA::PCState &val) override
    {
        thread.pcState(val);
    }

    RegVal
    readMiscRegNoEffect(int misc_reg) const
    {
        return thread.readMiscRegNoEffect(misc_reg);
    }

    RegVal
    readMiscReg(int misc_reg) override
    {
        return thread.readMiscReg(misc_reg);
    }

    void
    setMiscReg(int misc_reg, RegVal val) override
    {
        thread.setMiscReg(misc_reg, val);
    }

    RegVal
    readMiscRegOperand(const StaticInst *si, int idx) override
    {
        const RegId& reg = si->srcRegIdx(idx);
        assert(reg.isMiscReg());
        return thread.readMiscReg(reg.index());
    }

    void
    setMiscRegOperand(const StaticInst *si, int idx, RegVal val) override
    {
        const RegId& reg = si->destRegIdx(idx);
        assert(reg.isMiscReg());
        return thread.setMiscReg(reg.index(), val);
    }

    void
    syscall(Fault *fault) override
    {
        thread.syscall(fault);
    }

    ThreadContext *tcBase() const override { return thread.getTC(); }

    /* @todo, should make stCondFailures persistent somewhere */
    unsigned int readStCondFailures() const override { return 0; }
    void setStCondFailures(unsigned int st_cond_failures) override {}

    ContextID contextId() { return thread.contextId(); }
    /* ISA-specific (or at least currently ISA singleton) functions */

    /* X86: TLB twiddling */
    void
    demapPage(Addr vaddr, uint64_t asn) override
    {
        thread.getITBPtr()->demapPage(vaddr, asn);
        thread.getDTBPtr()->demapPage(vaddr, asn);
    }

    RegVal
    readCCRegOperand(const StaticInst *si, int idx) override
    {
        const RegId& reg = si->srcRegIdx(idx);
        assert(reg.isCCReg());
        return thread.readCCReg(reg.index());
    }

    void
    setCCRegOperand(const StaticInst *si, int idx, RegVal val) override
    {
        const RegId& reg = si->destRegIdx(idx);
        assert(reg.isCCReg());
        thread.setCCReg(reg.index(), val);
    }

    void
    demapInstPage(Addr vaddr, uint64_t asn)
    {
        thread.getITBPtr()->demapPage(vaddr, asn);
    }

    void
    demapDataPage(Addr vaddr, uint64_t asn)
    {
        thread.getDTBPtr()->demapPage(vaddr, asn);
    }

    BaseCPU *getCpuPtr() { return &cpu; }

  public:
    // monitor/mwait funtions
    void armMonitor(Addr address) override
    { getCpuPtr()->armMonitor(inst->id.threadId, address); }

    bool mwait(PacketPtr pkt) override
    { return getCpuPtr()->mwait(inst->id.threadId, pkt); }

    void mwaitAtomic(ThreadContext *tc) override
    { return getCpuPtr()->mwaitAtomic(inst->id.threadId, tc, thread.dtb); }

    AddressMonitor *getAddrMonitor() override
    { return getCpuPtr()->getCpuAddrMonitor(inst->id.threadId); }


#ifdef ISA_HAS_SS

    void configAtomicHardware(uint64_t a, uint64_t b, uint64_t c) {
      execute.getSSIM().atomic_update_hardware_config(a,b,c);
    }

    uint64_t receiveSS(int port, int dtype) {
      ssim_t& ssim = execute.getSSIM();
      return ssim.Receive(port, dtype);
    }

    void setDSARegisterFile(int idx, uint64_t value, bool sticky) override {
      if (idx) {
        LOG(REGISTER) << idx << " " << REG_NAMES[idx] << ": "
                      << value << ", " << sticky;
        execute.getSSIM().rf[idx].value = value;
        execute.getSSIM().rf[idx].sticky = sticky;
      }
    }

    void setDSAVPState(int idx, int field, int64_t value) override {
      switch (field) {
      case 0:
        execute.getSSIM().vps[idx].broadcast = value;
        break;
      case 1:
        execute.getSSIM().vps[idx].repeat = value;
        break;
      case 2:
        execute.getSSIM().vps[idx].stretch = value;
        break;
      }
    }

    // TODO: Make merge all the registers into SSIM
    void callSSFunc(int ss_func_opcode, const std::vector<int64_t> &args = {}) override {
      DPRINTF(SS, "Do SS_COMMAND %d.\n", SSCmdNames[ss_func_opcode]);
      ssim_t& ssim = execute.getSSIM();
      ssim.inst = inst;
      switch(ss_func_opcode) {
        case SS_BEGIN_ROI:
          ssim.statistics.roi(true);
          ssim.roi_entry(true);
          return;
        case SS_END_ROI:
          ssim.statistics.roi(false);
          ssim.roi_entry(false);
          return;
        case SS_STATS:
          ssim.print_stats();
          return;
        case SS_CFG:
          ssim.LoadBitstream();
          break;
        case SS_MEM_PRT:
          CHECK(args.size() == 4);
          ssim.LoadMemoryToPort(args[0], args[1], args[2], args[3]);
          break;
        case SS_PRT_MEM:
          CHECK(args.size() == 4);
          ssim.WritePortToMemory(args[0], args[1], args[2], args[3]);
          break;
        case SS_WAIT: {
          ssim.InsertBarrier(args[0]);
          break;
        }
        case SS_CNS_PRT: {
          CHECK(args.size() == 2);
          ssim.ConstStream(args[0], args[1]);
          break;
        }
        case SS_IND_PRT: {
          CHECK(args.size() == 4);
          ssim.IndirectMemoryToPort(args[0], args[1], args[2], args[3]);
          break;
        }
        case SS_PRT_PRT: {
          ssim.Reroute(args[0], args[1]);
          break;
        }
        case SS_ATOMIC_SCR_OP: {
          CHECK(args.size() == 5);
          ssim.AtomicMemoryOperation(args[0], args[1], args[2], args[3], args[4]);
          break;
        }
        // case SS_SCR_PRT:
        //   ssim.load_scratch_to_port(thread.getSSReg(SS_REPEAT),
        //                             thread.getSSReg(SS_REPEAT_STRETCH), thread.getSSReg(SS_CFG_SIZE), thread.getSSReg(SS_CONTEXT), thread.getSSReg(SS_GARB_ELEM));
        //   break;
        // case SS_PRT_SCR:
        //   ssim.write_scratchpad();
        //   break;
        // case SS_IND_PRT:
        //   ssim.indirect(
        //     thread.getSSReg(SS_IND_PORT), thread.getSSReg(SS_IND_TYPE),
        //     thread.getSSReg(SS_IN_PORT), thread.getSSReg(SS_INDEX_ADDR),
        //     thread.getSSReg(SS_NUM_ELEM), thread.getSSReg(SS_REPEAT),
        //     thread.getSSReg(SS_REPEAT_STRETCH), thread.getSSReg(SS_OFFSET_LIST),
        //     thread.getSSReg(SS_DTYPE), thread.getSSReg(SS_IND_MULT),
        //     thread.getSSReg(SS_IS_SCRATCH), thread.getSSReg(SS_FLAGS),
        //     thread.getSSReg(SS_STRIDE), thread.getSSReg(SS_ACCESS_SIZE),
        //     thread.getSSReg(SS_STRETCH), thread.getSSReg(SS_OFFSET), thread.getSSReg(SS_CFG_SIZE), thread.getSSReg(SS_CONTEXT), thread.getSSReg(SS_GARB_ELEM)); // changed interpretation of stretch here
        //   break;
        // case SS_PRT_IND:
        //   ssim.indirect_write(
        //     thread.getSSReg(SS_IND_PORT), thread.getSSReg(SS_IND_TYPE),
        //     thread.getSSReg(SS_OUT_PORT), thread.getSSReg(SS_INDEX_ADDR),
        //     thread.getSSReg(SS_NUM_ELEM), thread.getSSReg(SS_OFFSET_LIST),
        //     thread.getSSReg(SS_DTYPE), thread.getSSReg(SS_IND_MULT),
        //     thread.getSSReg(SS_IS_SCRATCH), thread.getSSReg(SS_FLAGS),
        //     thread.getSSReg(SS_STRIDE), thread.getSSReg(SS_ACCESS_SIZE),
        //     thread.getSSReg(SS_STRETCH), thread.getSSReg(SS_OFFSET));
        //   break;
        // case SS_CNS_PRT:
        //   ssim.write_constant(
        //     thread.getSSReg(SS_NUM_STRIDES), thread.getSSReg(SS_IN_PORT),
        //     thread.getSSReg(SS_CONSTANT), thread.getSSReg(SS_NUM_ELEM),     
        //     thread.getSSReg(SS_CONSTANT2), thread.getSSReg(SS_NUM_ELEM2),    
        //     thread.getSSReg(SS_FLAGS), thread.getSSReg(SS_DTYPE), thread.getSSReg(SS_IND_TYPE));
        //   break;
        // case SS_CONST_SCR:
        //   ssim.write_constant_scratchpad(
        //     thread.getSSReg(SS_SCRATCH_ADDR), thread.getSSReg(SS_CONSTANT),
        //     thread.getSSReg(SS_NUM_STRIDES), thread.getSSReg(SS_DTYPE));
        //   break;
        // case SS_REM_PORT:
        //   ssim.multicast_remote_port(
        //     thread.getSSReg(SS_NUM_ELEM), thread.getSSReg(SS_SCRATCH_ADDR),
        //     thread.getSSReg(SS_OUT_PORT), thread.getSSReg(SS_IN_PORT),
        //     thread.getSSReg(SS_FLAGS), thread.getSSReg(SS_ADDR_TYPE),
        //     thread.getSSReg(SS_STRIDE), thread.getSSReg(SS_ACCESS_SIZE));
        //   break;
        // case SS_WAIT_DF:
        //   ssim.insert_df_barrier(
        //     thread.getSSReg(SS_NUM_ELEM), thread.getSSReg(SS_ADDR_TYPE));
        //   break;
        // case SS_BUFFET:
        //   ssim.instantiate_buffet(thread.getSSReg(SS_REPEAT), thread.getSSReg(SS_REPEAT_STRETCH));
        //   break;
        default:
          CHECK(false) << "Command not supported!";
          break;
      }
      ssim.resetNonStickyState();
    }
#endif

};

}

#endif /* __CPU_MINOR_EXEC_CONTEXT_HH__ */
