// See LICENSE.SiFive for license details.
// See LICENSE.Berkeley for license details.

package testriscvboom.v3.exu

import chisel3._
import chisel3.util.{BitPat, Fill, Cat, Reverse, PriorityEncoderOH, PopCount, MuxLookup}
import org.chipsalliance.cde.config.Parameters
import freechips.rocketchip.tile.CoreModule
import freechips.rocketchip.util._
import freechips.rocketchip.rocket._

object ALUTMR {
  val SZ_ALU_FN = 5
  def FN_X    = BitPat("b?????")
  def FN_ADD  = 0.U
  def FN_SL   = 1.U
  def FN_SEQ  = 2.U
  def FN_SNE  = 3.U
  def FN_XOR  = 4.U
  def FN_SR   = 5.U
  def FN_OR   = 6.U
  def FN_AND  = 7.U
  def FN_CZEQZ = 8.U
  def FN_CZNEZ = 9.U
  def FN_SUB  = 10.U
  def FN_SRA  = 11.U
  def FN_SLT  = 12.U
  def FN_SGE  = 13.U
  def FN_SLTU = 14.U
  def FN_SGEU = 15.U
  def FN_UNARY = 16.U
  def FN_ROL  = 17.U
  def FN_ROR  = 18.U
  def FN_BEXT = 19.U

  def FN_ANDN = 24.U
  def FN_ORN  = 25.U
  def FN_XNOR = 26.U

  def FN_MAX  = 28.U
  def FN_MIN  = 29.U
  def FN_MAXU = 30.U
  def FN_MINU = 31.U
  def FN_MAXMIN = BitPat("b111??")

  // Mul/div reuse some integer FNs
  def FN_DIV  = FN_XOR
  def FN_DIVU = FN_SR
  def FN_REM  = FN_OR
  def FN_REMU = FN_AND

  def FN_MUL    = FN_ADD
  def FN_MULH   = FN_SL
  def FN_MULHSU = FN_SEQ
  def FN_MULHU  = FN_SNE

  def isMulFN(fn: UInt, cmp: UInt) = fn(1,0) === cmp(1,0)
  def isSub(cmd: UInt) = cmd(3)
  def isCmp(cmd: UInt) = (cmd >= FN_SLT && cmd <= FN_SGEU)
  def isMaxMin(cmd: UInt) = (cmd >= FN_MAX && cmd <= FN_MINU)
  def cmpUnsigned(cmd: UInt) = cmd(1)
  def cmpInverted(cmd: UInt) = cmd(0)
  def cmpEq(cmd: UInt) = !cmd(3)
  def shiftReverse(cmd: UInt) = !cmd.isOneOf(FN_SR, FN_SRA, FN_ROR, FN_BEXT)
  def bwInvRs2(cmd: UInt) = cmd.isOneOf(FN_ANDN, FN_ORN, FN_XNOR)
}

import ALUTMR._


abstract class AbstractALUTMR(implicit p: Parameters) extends CoreModule()(p) {
  val io = IO(new Bundle {
    val dw = Input(UInt(SZ_DW.W))
    val fn = Input(UInt(SZ_ALU_FN.W))
    val in2 = Input(UInt(xLen.W))
    val in1 = Input(UInt(xLen.W))
    val out = Output(UInt(xLen.W))
    val adder_out = Output(UInt(xLen.W))
    val cmp_out = Output(Bool())
  })
}

class ALUTMR(implicit p: Parameters) extends AbstractALUTMR()(p) {
  override def desiredName = "RocketALUTMR"

  /////////////////////////////////////////////////////////// ALU 1 ///////////////////////////////////////////////////////////

  // ADD, SUB
  val in2_inva = Mux(isSub(io.fn), ~io.in2, io.in2)
  val in1_xor_in2a = io.in1 ^ in2_inva
  val in1_and_in2a = io.in1 & in2_inva
  val adder_out1 = io.in1 + in2_inva + isSub(io.fn)
  
  // SLT, SLTU
  val slta =
    Mux(io.in1(xLen-1) === io.in2(xLen-1), adder_out1(xLen-1),
    Mux(cmpUnsigned(io.fn), io.in2(xLen-1), io.in1(xLen-1)))
  val cmp_out1 = cmpInverted(io.fn) ^ Mux(cmpEq(io.fn), in1_xor_in2a === 0.U, slta)

  // SLL, SRL, SRA
  val (shamta, shin_ra) =
    if (xLen == 32) (io.in2(4,0), io.in1)
    else {
      require(xLen == 64)
      val shin_hi_32 = Fill(32, isSub(io.fn) && io.in1(31))
      val shin_hi = Mux(io.dw === DW_64, io.in1(63,32), shin_hi_32)
      val shamta = Cat(io.in2(5) & (io.dw === DW_64), io.in2(4,0))
      (shamta, Cat(shin_hi, io.in1(31,0)))
    }
  val shina = Mux(shiftReverse(io.fn), Reverse(shin_ra), shin_ra)
  val shout_ra = (Cat(isSub(io.fn) & shina(xLen-1), shina).asSInt >> shamta)(xLen-1,0)
  val shout_la = Reverse(shout_ra)
  val shouta = Mux(io.fn === FN_SR || io.fn === FN_SRA || io.fn === FN_BEXT, shout_ra, 0.U) |
              Mux(io.fn === FN_SL,                                          shout_la, 0.U)

  // CZEQZ, CZNEZ
  val in2_not_zeroa = io.in2.orR
  val cond_outa = Option.when(usingConditionalZero)(
    Mux((io.fn === FN_CZEQZ && in2_not_zeroa) || (io.fn === FN_CZNEZ && !in2_not_zeroa), io.in1, 0.U)
  )

  // AND, OR, XOR
  val logica = Mux(io.fn === FN_XOR || io.fn === FN_OR || io.fn === FN_ORN || io.fn === FN_XNOR, in1_xor_in2a, 0.U) |
              Mux(io.fn === FN_OR || io.fn === FN_AND || io.fn === FN_ORN || io.fn === FN_ANDN, in1_and_in2a, 0.U)

  val bext_maska = Mux(coreParams.useZbs.B && io.fn === FN_BEXT, 1.U, ~(0.U(xLen.W)))
  val shift_logica = (isCmp (io.fn) && slta) | logica | (shouta & bext_maska)
  val shift_logic_conda = cond_outa match {
    case Some(co) => shift_logica | co
    case _ => shift_logica
  }

  // CLZ, CTZ, CPOP
  val tz_ina = MuxLookup((io.dw === DW_32) ## !io.in2(0), 0.U)(Seq(
    0.U -> io.in1,
    1.U -> Reverse(io.in1),
    2.U -> 1.U ## io.in1(31,0),
    3.U -> 1.U ## Reverse(io.in1(31,0))
  ))
  val popc_ina = Mux(io.in2(1),
    Mux(io.dw === DW_32, io.in1(31,0), io.in1),
    PriorityEncoderOH(1.U ## tz_ina) - 1.U)(xLen-1,0)
  val counta = PopCount(popc_ina)
  val in1_bytesa = io.in1.asTypeOf(Vec(xLen / 8, UInt(8.W)))
  val orcba = VecInit(in1_bytesa.map(b => Fill(8, b =/= 0.U))).asUInt
  val rev8a = VecInit(in1_bytesa.reverse).asUInt
  val unarya = MuxLookup(io.in2(11,0), counta)(Seq(
    0x287.U -> orcba,
    (if (xLen == 32) 0x698 else 0x6b8).U -> rev8a,
    0x080.U -> io.in1(15,0),
    0x604.U -> Fill(xLen-8, io.in1(7)) ## io.in1(7,0),
    0x605.U -> Fill(xLen-16, io.in1(15)) ## io.in1(15,0)
  ))

  // MAX, MIN, MAXU, MINU
  val maxmin_outa = Mux(cmp_out1, io.in2, io.in1)

  // ROL, ROR
  val rot_shamta = Mux(io.dw === DW_32, 32.U, xLen.U) - shamta
  val rotina = Mux(io.fn(0), shin_ra, Reverse(shin_ra))
  val rotout_ra = (rotina >> rot_shamta)(xLen-1,0)
  val rotout_la = Reverse(rotout_ra)
  val rotouta = Mux(io.fn(0), rotout_ra, rotout_la) | Mux(io.fn(0), shout_la, shout_ra)

  val out1 = MuxLookup(io.fn, shift_logic_conda)(Seq(
    FN_ADD -> adder_out1,
    FN_SUB -> adder_out1
  ) ++ (if (coreParams.useZbb) Seq(
    FN_UNARY -> unarya,
    FN_MAX -> maxmin_outa,
    FN_MIN -> maxmin_outa,
    FN_MAXU -> maxmin_outa,
    FN_MINU -> maxmin_outa,
    FN_ROL -> rotouta,
    FN_ROR -> rotouta,
  ) else Nil))


  /////////////////////////////////////////////////////////// ALU 2 ///////////////////////////////////////////////////////////

  // ADD, SUB
  val in2_invb = Mux(isSub(io.fn), ~io.in2, io.in2)
  val in1_xor_in2b = io.in1 ^ in2_invb
  val in1_and_in2b = io.in1 & in2_invb
  val adder_out2 = io.in1 + in2_invb + isSub(io.fn)

  // SLT, SLTU
  val sltb =
    Mux(io.in1(xLen-1) === io.in2(xLen-1), adder_out2(xLen-1),
    Mux(cmpUnsigned(io.fn), io.in2(xLen-1), io.in1(xLen-1)))
  val cmp_out2 = cmpInverted(io.fn) ^ Mux(cmpEq(io.fn), in1_xor_in2b === 0.U, sltb)

  // SLL, SRL, SRA
  val (shamtb, shin_rb) =
    if (xLen == 32) (io.in2(4,0), io.in1)
    else {
      require(xLen == 64)
      val shin_hi_32 = Fill(32, isSub(io.fn) && io.in1(31))
      val shin_hi = Mux(io.dw === DW_64, io.in1(63,32), shin_hi_32)
      val shamtb = Cat(io.in2(5) & (io.dw === DW_64), io.in2(4,0))
      (shamtb, Cat(shin_hi, io.in1(31,0)))
    }
  val shinb = Mux(shiftReverse(io.fn), Reverse(shin_rb), shin_rb)
  val shout_rb = (Cat(isSub(io.fn) & shinb(xLen-1), shinb).asSInt >> shamtb)(xLen-1,0)
  val shout_lb = Reverse(shout_rb)
  val shoutb = Mux(io.fn === FN_SR || io.fn === FN_SRA || io.fn === FN_BEXT, shout_rb, 0.U) |
              Mux(io.fn === FN_SL,                                          shout_lb, 0.U)

  // CZEQZ, CZNEZ
  val in2_not_zerob = io.in2.orR
  val cond_outb = Option.when(usingConditionalZero)(
    Mux((io.fn === FN_CZEQZ && in2_not_zerob) || (io.fn === FN_CZNEZ && !in2_not_zerob), io.in1, 0.U)
  )

  // AND, OR, XOR
  val logicb = Mux(io.fn === FN_XOR || io.fn === FN_OR || io.fn === FN_ORN || io.fn === FN_XNOR, in1_xor_in2b, 0.U) |
              Mux(io.fn === FN_OR || io.fn === FN_AND || io.fn === FN_ORN || io.fn === FN_ANDN, in1_and_in2b, 0.U)

  val bext_maskb = Mux(coreParams.useZbs.B && io.fn === FN_BEXT, 1.U, ~(0.U(xLen.W)))
  val shift_logicb = (isCmp (io.fn) && sltb) | logicb | (shoutb & bext_maskb)
  val shift_logic_condb = cond_outb match {
    case Some(co) => shift_logicb | co
    case _ => shift_logicb
  }

  // CLZ, CTZ, CPOP
  val tz_inb = MuxLookup((io.dw === DW_32) ## !io.in2(0), 0.U)(Seq(
    0.U -> io.in1,
    1.U -> Reverse(io.in1),
    2.U -> 1.U ## io.in1(31,0),
    3.U -> 1.U ## Reverse(io.in1(31,0))
  ))
  val popc_inb = Mux(io.in2(1),
    Mux(io.dw === DW_32, io.in1(31,0), io.in1),
    PriorityEncoderOH(1.U ## tz_inb) - 1.U)(xLen-1,0)
  val countb = PopCount(popc_inb)
  val in1_bytesb = io.in1.asTypeOf(Vec(xLen / 8, UInt(8.W)))
  val orcbb = VecInit(in1_bytesb.map(b => Fill(8, b =/= 0.U))).asUInt
  val rev8b = VecInit(in1_bytesb.reverse).asUInt
  val unaryb = MuxLookup(io.in2(11,0), countb)(Seq(
    0x287.U -> orcbb,
    (if (xLen == 32) 0x698 else 0x6b8).U -> rev8b,
    0x080.U -> io.in1(15,0),
    0x604.U -> Fill(xLen-8, io.in1(7)) ## io.in1(7,0),
    0x605.U -> Fill(xLen-16, io.in1(15)) ## io.in1(15,0)
  ))

  // MAX, MIN, MAXU, MINU
  val maxmin_outb = Mux(cmp_out2, io.in2, io.in1)

  // ROL, ROR
  val rot_shamtb = Mux(io.dw === DW_32, 32.U, xLen.U) - shamtb
  val rotinb = Mux(io.fn(0), shin_rb, Reverse(shin_rb))
  val rotout_rb = (rotinb >> rot_shamtb)(xLen-1,0)
  val rotout_lb = Reverse(rotout_rb)
  val rotoutb = Mux(io.fn(0), rotout_rb, rotout_lb) | Mux(io.fn(0), shout_lb, shout_rb)

  val out2 = MuxLookup(io.fn, shift_logic_condb)(Seq(
    FN_ADD -> adder_out2,
    FN_SUB -> adder_out2
  ) ++ (if (coreParams.useZbb) Seq(
    FN_UNARY -> unaryb,
    FN_MAX -> maxmin_outb,
    FN_MIN -> maxmin_outb,
    FN_MAXU -> maxmin_outb,
    FN_MINU -> maxmin_outb,
    FN_ROL -> rotoutb,
    FN_ROR -> rotoutb,
  ) else Nil))


  /////////////////////////////////////////////////////////// ALU 3 ///////////////////////////////////////////////////////////

  // ADD, SUB
  val in2_invc = Mux(isSub(io.fn), ~io.in2, io.in2)
  val in1_xor_in2c = io.in1 ^ in2_invc
  val in1_and_in2c = io.in1 & in2_invc
  val adder_out3 = io.in1 + in2_invc + isSub(io.fn)

  // SLT, SLTU
  val sltc =
    Mux(io.in1(xLen-1) === io.in2(xLen-1), adder_out3(xLen-1),
    Mux(cmpUnsigned(io.fn), io.in2(xLen-1), io.in1(xLen-1)))
  val cmp_out3 = cmpInverted(io.fn) ^ Mux(cmpEq(io.fn), in1_xor_in2c === 0.U, sltc)

  // SLL, SRL, SRA
  val (shamtc, shin_rc) =
    if (xLen == 32) (io.in2(4,0), io.in1)
    else {
      require(xLen == 64)
      val shin_hi_32 = Fill(32, isSub(io.fn) && io.in1(31))
      val shin_hi = Mux(io.dw === DW_64, io.in1(63,32), shin_hi_32)
      val shamtc = Cat(io.in2(5) & (io.dw === DW_64), io.in2(4,0))
      (shamtc, Cat(shin_hi, io.in1(31,0)))
    }
  val shinc = Mux(shiftReverse(io.fn), Reverse(shin_rc), shin_rc)
  val shout_rc = (Cat(isSub(io.fn) & shinc(xLen-1), shinc).asSInt >> shamtc)(xLen-1,0)
  val shout_lc = Reverse(shout_rc)
  val shoutc = Mux(io.fn === FN_SR || io.fn === FN_SRA || io.fn === FN_BEXT, shout_rc, 0.U) |
              Mux(io.fn === FN_SL,                                          shout_lc, 0.U)

  // CZEQZ, CZNEZ
  val in2_not_zeroc = io.in2.orR
  val cond_outc = Option.when(usingConditionalZero)(
    Mux((io.fn === FN_CZEQZ && in2_not_zeroc) || (io.fn === FN_CZNEZ && !in2_not_zeroc), io.in1, 0.U)
  )

  // AND, OR, XOR
  val logicc = Mux(io.fn === FN_XOR || io.fn === FN_OR || io.fn === FN_ORN || io.fn === FN_XNOR, in1_xor_in2c, 0.U) |
              Mux(io.fn === FN_OR || io.fn === FN_AND || io.fn === FN_ORN || io.fn === FN_ANDN, in1_and_in2c, 0.U)

  val bext_maskc = Mux(coreParams.useZbs.B && io.fn === FN_BEXT, 1.U, ~(0.U(xLen.W)))
  val shift_logicc = (isCmp (io.fn) && sltc) | logicc | (shoutc & bext_maskc)
  val shift_logic_condc = cond_outc match {
    case Some(co) => shift_logicc | co
    case _ => shift_logicc
  }

  // CLZ, CTZ, CPOP
  val tz_inc = MuxLookup((io.dw === DW_32) ## !io.in2(0), 0.U)(Seq(
    0.U -> io.in1,
    1.U -> Reverse(io.in1),
    2.U -> 1.U ## io.in1(31,0),
    3.U -> 1.U ## Reverse(io.in1(31,0))
  ))
  val popc_inc = Mux(io.in2(1),
    Mux(io.dw === DW_32, io.in1(31,0), io.in1),
    PriorityEncoderOH(1.U ## tz_inc) - 1.U)(xLen-1,0)
  val countc = PopCount(popc_inc)
  val in1_bytesc = io.in1.asTypeOf(Vec(xLen / 8, UInt(8.W)))
  val orcbc = VecInit(in1_bytesc.map(b => Fill(8, b =/= 0.U))).asUInt
  val rev8c = VecInit(in1_bytesc.reverse).asUInt
  val unaryc = MuxLookup(io.in2(11,0), countc)(Seq(
    0x287.U -> orcbc,
    (if (xLen == 32) 0x698 else 0x6b8).U -> rev8c,
    0x080.U -> io.in1(15,0),
    0x604.U -> Fill(xLen-8, io.in1(7)) ## io.in1(7,0),
    0x605.U -> Fill(xLen-16, io.in1(15)) ## io.in1(15,0)
  ))

  // MAX, MIN, MAXU, MINU
  val maxmin_outc = Mux(cmp_out3, io.in2, io.in1)

  // ROL, ROR
  val rot_shamtc = Mux(io.dw === DW_32, 32.U, xLen.U) - shamtc
  val rotinc = Mux(io.fn(0), shin_rc, Reverse(shin_rc))
  val rotout_rc = (rotinc >> rot_shamtc)(xLen-1,0)
  val rotout_lc = Reverse(rotout_rc)
  val rotoutc = Mux(io.fn(0), rotout_rc, rotout_lc) | Mux(io.fn(0), shout_lc, shout_rc)

  val out3 = MuxLookup(io.fn, shift_logic_condc)(Seq(
    FN_ADD -> adder_out3,
    FN_SUB -> adder_out3
  ) ++ (if (coreParams.useZbb) Seq(
    FN_UNARY -> unaryc,
    FN_MAX -> maxmin_outc,
    FN_MIN -> maxmin_outc,
    FN_MAXU -> maxmin_outc,
    FN_MINU -> maxmin_outc,
    FN_ROL -> rotoutc,
    FN_ROR -> rotoutc,
  ) else Nil))


  /////////////////////////////////////////////////////// MAJORITY VOTE ///////////////////////////////////////////////////////

  io.adder_out := ((adder_out1 & adder_out2) | (adder_out1 & adder_out3) | (adder_out2 & adder_out3))
  io.cmp_out := ((cmp_out1 & cmp_out2) | (cmp_out1 & cmp_out3) | (cmp_out2 & cmp_out3))
  val out = ((out1 & out2) | (out1 & out3) | (out2 & out3))
  io.out := out
  if (xLen > 32) {
    require(xLen == 64)
    when (io.dw === DW_32) { io.out := Cat(Fill(32, out(31)), out(31,0)) }
  }
}