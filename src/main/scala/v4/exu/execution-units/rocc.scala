//******************************************************************************
// Copyright (c) 2013 - 2019, The Regents of the University of California (Regents).
// All Rights Reserved. See LICENSE and LICENSE.SiFive for license details.
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
// The RoCC shim unit. Similar to the LSU, in that we need to allocate entries
// for instruction bits at dispatch, and send commands strictly in order.
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

package testriscvboom.v4.exu

import chisel3._
import chisel3.util._

import org.chipsalliance.cde.config.Parameters
import freechips.rocketchip.tile.{RoCCCoreIO, RoCCInstruction}
import freechips.rocketchip.rocket._

import testriscvboom.v4.common._
import testriscvboom.v4.util._

/**
  * IO Bundle representing RoCC shim interface with the core
  */
class RoCCShimCoreIO(implicit p: Parameters) extends BoomBundle
{
  // Decode Stage
  val dis_rocc_vals    = Input(Vec(coreWidth, Bool()))
  val dis_uops         = Input(Vec(coreWidth, new MicroOp))
  val rxq_full         = Output(Bool())
  val rxq_empty        = Output(Bool())
  val rxq_idx          = Output(Vec(coreWidth, UInt(log2Ceil(numRxqEntries).W)))
  val rob_pnr_idx      = Input(UInt(robAddrSz.W))
  val rob_head_idx     = Input(UInt(robAddrSz.W))

  val rocc             = Flipped(new RoCCCoreIO)
}

/**
 * IO bundle representing the different signals to interact with the RoCC
 * Vaguely follows the IO of a functional unit.
  */
class RoCCShimIO(implicit p: Parameters) extends BoomBundle
{
  val core             = new RoCCShimCoreIO

  val req              = Flipped(new DecoupledIO(new FuncUnitReq(xLen)))
  val resp             = new DecoupledIO(new ExeUnitResp(xLen))
  val brupdate         = Input(new BrUpdateInfo())
  val status           = Input(new MStatus)
  val exception        = Input(Bool())
}

class RCQEntry(implicit p: Parameters) extends BoomBundle {
  val uop = new MicroOp()
}

/**
  * Structure similar to LSU
  *  - Holds instruction and operand bits prior to issuing RoCC inst to
  *    accelerator
  *  - After issue, holds queue of translations between logical and physical
  *    specifiers to handle RoCC responses
  */
class RoCCShim(implicit p: Parameters) extends BoomModule
{
  val io = IO(new RoCCShimIO)

  io.req.ready := true.B
  io.core.rocc.exception := false.B
  io.core.rocc.mem.req.ready := false.B
  io.core.rocc.mem.s2_nack := false.B
  io.core.rocc.mem.s2_nack_cause_raw := false.B
  io.core.rocc.mem.s2_uncached := false.B
  io.core.rocc.mem.s2_paddr := DontCare
  io.core.rocc.mem.resp.valid := false.B
  io.core.rocc.mem.resp.bits := DontCare
  io.core.rocc.mem.replay_next := false.B
  io.core.rocc.mem.s2_xcpt.ma.ld := false.B
  io.core.rocc.mem.s2_xcpt.ma.st := false.B
  io.core.rocc.mem.s2_xcpt.pf.ld := false.B
  io.core.rocc.mem.s2_xcpt.pf.st := false.B
  io.core.rocc.mem.s2_xcpt.gf.ld := false.B
  io.core.rocc.mem.s2_xcpt.gf.st := false.B
  io.core.rocc.mem.s2_xcpt.ae.ld := false.B
  io.core.rocc.mem.s2_xcpt.ae.st := false.B
  io.core.rocc.mem.s2_gpa := DontCare
  io.core.rocc.mem.s2_gpa_is_pte := false.B
  io.core.rocc.mem.uncached_resp.map(r => {
    r.valid := false.B
    r.bits := DontCare
  })
  io.core.rocc.mem.ordered := false.B
  io.core.rocc.mem.perf.acquire := false.B
  io.core.rocc.mem.perf.release := false.B
  io.core.rocc.mem.perf.grant := false.B
  io.core.rocc.mem.perf.tlbMiss := false.B
  io.core.rocc.mem.perf.blocked := false.B
  io.core.rocc.mem.perf.canAcceptStoreThenLoad := false.B
  io.core.rocc.mem.perf.canAcceptStoreThenRMW := false.B
  io.core.rocc.mem.perf.canAcceptLoadThenLoad := false.B
  io.core.rocc.mem.perf.storeBufferEmptyAfterLoad := false.B
  io.core.rocc.mem.perf.storeBufferEmptyAfterStore := false.B
  io.core.rocc.mem.clock_enabled := false.B

  // RoCC execute queue. Wait for PNR, holds operands and inst bits
  val rxq_val       = Reg(Vec(numRxqEntries, Bool()))
  val rxq_op_val    = Reg(Vec(numRxqEntries, Bool()))
  val rxq_committed = Reg(Vec(numRxqEntries, Bool()))
  val rxq_uop       = Reg(Vec(numRxqEntries, new MicroOp()))
  val rxq_inst      = Reg(Vec(numRxqEntries, UInt(32.W)))
  val rxq_rs1       = Reg(Vec(numRxqEntries, UInt(xLen.W)))
  val rxq_rs2       = Reg(Vec(numRxqEntries, UInt(xLen.W)))

  // RoCC commit queue. Wait for response, or immediate unbusy
  val rcq           = Reg(Vec(numRcqEntries, Valid(new RCQEntry)))
  val rcq_head      = RegInit(0.U(log2Ceil(numRcqEntries).W))
  val rcq_tail      = RegInit(0.U(log2Ceil(numRcqEntries).W))
  val rcq_full      = WrapInc(rcq_tail, numRcqEntries) === rcq_head

  // The instruction we are waiting for response from
  val rxq_head     = RegInit(0.U(log2Ceil(numRxqEntries).W))
  // The next instruction we are waiting to "commit" through PNR
  val rxq_com_head = RegInit(0.U(log2Ceil(numRxqEntries).W))
  val rxq_tail     = RegInit(0.U(log2Ceil(numRxqEntries).W))


  // Decode
  val rocc_idx = WireInit(0.U)
  val br_mask = WireInit(0.U(maxBrCount.W))
  var enq_val = false.B

  assert(PopCount(io.core.dis_rocc_vals) <= 1.U)
  for (w <- 0 until coreWidth) {
    val enq_this = !enq_val && io.core.dis_rocc_vals(w) && io.core.dis_uops(w).is_rocc
    when (enq_this) {
      rocc_idx := w.U
    }

    io.core.rxq_idx(w) := Mux(enq_val, WrapInc(rxq_tail, numRxqEntries), rxq_tail)

    enq_val = enq_val || enq_this
  }

  when (enq_val) {
    rxq_val      (rxq_tail) := true.B
    rxq_op_val   (rxq_tail) := false.B
    rxq_committed(rxq_tail) := false.B
    rxq_uop      (rxq_tail) := io.core.dis_uops(rocc_idx)
    rxq_inst     (rxq_tail) := io.core.dis_uops(rocc_idx).inst
    rxq_tail                := WrapInc(rxq_tail, numRxqEntries)
  }

  // Wait for operands
  when (io.req.valid && !IsKilledByBranch(io.brupdate, io.exception, io.req.bits.uop)) {
    val rxq_idx = io.req.bits.uop.rxq_idx
    assert(io.req.bits.uop.rob_idx === rxq_uop(rxq_idx).rob_idx,
      "Mismatch between RoCCUnit request and RoCC execute head")
    assert(rxq_val(rxq_idx),
      "Trying to execute rocc inst without the instruction bits")

    rxq_op_val   (rxq_idx)      := true.B
    rxq_uop      (rxq_idx).pdst := io.req.bits.uop.pdst
    rxq_rs1      (rxq_idx)      := io.req.bits.rs1_data
    rxq_rs2      (rxq_idx)      := io.req.bits.rs2_data
  }

  // Wait for ROB to OK us to execute
  when (rxq_val   (rxq_com_head) &&
        IsOlder(rxq_uop(rxq_com_head).rob_idx, io.core.rob_pnr_idx, io.core.rob_head_idx)) {
    rxq_committed(rxq_com_head)   := true.B
    rxq_com_head                  := WrapInc(rxq_com_head, numRxqEntries)
  }

  // Execute
  io.core.rocc.cmd.valid := false.B
  io.core.rocc.cmd.bits.inst     := rxq_inst(rxq_head).asTypeOf(new RoCCInstruction)
  io.core.rocc.cmd.bits.rs1      := rxq_rs1(rxq_head)
  io.core.rocc.cmd.bits.rs2      := rxq_rs2(rxq_head)
  io.core.rocc.cmd.bits.status   := io.status

  val resp_arb = Module(new Arbiter(new ExeUnitResp(xLen), 2))
  resp_arb.io.in(0).valid     := false.B
  resp_arb.io.in(0).bits.uop  := rxq_uop(rxq_head)
  resp_arb.io.in(0).bits.data := DontCare
  resp_arb.io.in(0).bits.predicated := false.B
  resp_arb.io.in(0).bits.fflags.valid := false.B
  resp_arb.io.in(0).bits.fflags.bits := DontCare
  when (rxq_op_val   (rxq_head) &&
        rxq_val      (rxq_head) &&
        rxq_committed(rxq_head) &&
        io.core.rocc.cmd.ready  &&
        resp_arb.io.in(0).ready &&
        !rcq_full) {
    io.core.rocc.cmd.valid         := true.B

    when (rxq_uop(rxq_head).dst_rtype =/= RT_X) {
      rcq_tail                       := WrapInc(rcq_tail, numRcqEntries)
      rcq(rcq_tail).valid            := true.B
      rcq(rcq_tail).bits.uop         := rxq_uop(rxq_head)
    } .otherwise {
      resp_arb.io.in(0).valid        := true.B
    }

    rxq_val(rxq_head)              := false.B
    rxq_head                       := WrapInc(rxq_head, numRxqEntries)
  }


  io.core.rxq_full  := WrapInc(rxq_tail, numRxqEntries) === rxq_head
  io.core.rxq_empty := rxq_tail === rxq_head

  //--------------------------
  // Branches
  for (i <- 0 until numRxqEntries) {
    when (rxq_val(i)) {
      rxq_uop(i).br_mask := GetNewBrMask(io.brupdate, rxq_uop(i))
      when (IsKilledByBranch(io.brupdate, io.exception, rxq_uop(i))) {
        rxq_val(i)      := false.B
        rxq_op_val(i)   := false.B
      }
    }
  }
  when (io.brupdate.b2.mispredict && !io.exception) {
    rxq_tail := io.brupdate.b2.uop.rxq_idx
  }


  //--------------------------
  // Exception / Reset

  when (reset.asBool) {
    rxq_tail     := 0.U
    rxq_head     := 0.U
    rxq_com_head := 0.U
    for (i <- 0 until numRxqEntries) {
      rxq_val(i)       := false.B
      rxq_op_val(i)    := false.B
      rxq_committed(i) := false.B
    }
  } .elsewhen (io.exception) {
    rxq_tail := rxq_com_head
    for (i <- 0 until numRxqEntries) {
      when (!rxq_committed(i)) {
        rxq_val(i)      := false.B
        rxq_op_val(i)   := false.B
      }
    }
  }

  //------------------
  // Handle responses

  resp_arb.io.in(1).valid     := io.core.rocc.resp.valid
  resp_arb.io.in(1).bits.uop  := rcq(rcq_head).bits.uop
  resp_arb.io.in(1).bits.data := io.core.rocc.resp.bits.data
  resp_arb.io.in(1).bits.predicated := io.core.rocc.resp.bits.data
  resp_arb.io.in(1).bits.fflags.valid := false.B
  resp_arb.io.in(1).bits.fflags.bits := DontCare
  io.core.rocc.resp.ready := resp_arb.io.in(1).ready
  io.resp <> resp_arb.io.out

  when (io.core.rocc.resp.fire) {
    assert(rcq(rcq_head).valid && rcq(rcq_head).bits.uop.ldst === io.core.rocc.resp.bits.rd)
    rcq(rcq_head).valid := false.B
    rcq_head := WrapInc(rcq_head, numRcqEntries)
  }


  when (reset.asBool) {
    for (i <- 0 until numRcqEntries) {
      rcq(i).valid := false.B
    }
  }

}
