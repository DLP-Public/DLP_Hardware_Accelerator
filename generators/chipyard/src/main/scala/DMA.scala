package memaccer

import chisel3._
import chisel3.util._
import freechips.rocketchip.config.Parameters
import freechips.rocketchip.diplomacy.{LazyModule, LazyModuleImp, IdRange}
import freechips.rocketchip.util.DecoupledHelper
import testchipip.{StreamChannel, TLHelper}
import icenet.{ReservationBufferAlloc, ReservationBufferData, ReservationBuffer, Aligner}

class StreamReadRequest extends Bundle {
  val address = UInt(48.W)
  val length = UInt(15.W)
  val partial = Bool() 
}

class StreamReader(nXacts: Int, outFlits: Int, maxBytes: Int)
    (implicit p: Parameters) extends LazyModule {

  val core = LazyModule(new StreamReaderCore(nXacts, outFlits, maxBytes))
  val node = core.node

  lazy val module = new LazyModuleImp(this) {
    val dataBits = core.module.dataBits

    val io = IO(new Bundle {
      val req = Flipped(Decoupled(new StreamReadRequest))
      val resp = Decoupled(Bool())
      val out = Decoupled(new StreamChannel(dataBits))
    })

    core.module.io.req <> io.req
    io.resp <> core.module.io.resp
    // @lyq on
    // printf("@DMA: using my modified DMA...\n")
    val buffer = Module(new ReservationBuffer(nXacts, outFlits, dataBits))
    buffer.io.alloc <> core.module.io.alloc
    buffer.io.in <> core.module.io.out

    val aligner = Module(new Aligner(dataBits))
    aligner.io.in <> buffer.io.out
    io.out <> aligner.io.out
  }
}

class StreamReaderCore(nXacts: Int, outFlits: Int, maxBytes: Int)
    (implicit p: Parameters) extends LazyModule {
  val node = TLHelper.makeClientNode(
    name = "stream-reader", sourceId = IdRange(0, nXacts))

  lazy val module = new LazyModuleImp(this) {
    val (tl, edge) = node.out(0)
    val dataBits = tl.params.dataBits
    val beatBytes = dataBits / 8
    val byteAddrBits = log2Ceil(beatBytes)
    val addrBits = tl.params.addressBits
    val lenBits = 15

    val io = IO(new Bundle {
      val req = Flipped(Decoupled(new StreamReadRequest))
      val resp = Decoupled(Bool())
      val alloc = Decoupled(new ReservationBufferAlloc(nXacts, outFlits))
      val out = Decoupled(new ReservationBufferData(nXacts, dataBits))
    })

    val s_idle :: s_read :: s_resp :: Nil = Enum(3)
    val state = RegInit(s_idle)

    // Physical (word) address in memory
    val sendaddr = Reg(UInt(addrBits.W))
    // Number of words to send
    val sendlen  = Reg(UInt(lenBits.W))
    // 0 if last packet in sequence, 1 otherwise
    val sendpart = Reg(Bool())

    val xactBusy = RegInit(0.U(nXacts.W))
    val xactOnehot = PriorityEncoderOH(~xactBusy)
    val xactId = OHToUInt(xactOnehot)
    val xactLast = Reg(UInt(nXacts.W))
    val xactLeftKeep = Reg(Vec(nXacts, UInt(beatBytes.W)))
    val xactRightKeep = Reg(Vec(nXacts, UInt(beatBytes.W)))

    val reqSize = MuxCase(byteAddrBits.U,
      (log2Ceil(maxBytes) until byteAddrBits by -1).map(lgSize =>
          // Use the largest size (beatBytes <= size <= maxBytes)
          // s.t. sendaddr % size == 0 and sendlen > size
          (sendaddr(lgSize-1,0) === 0.U &&
            (sendlen >> lgSize.U) =/= 0.U) -> lgSize.U))
    val isLast = (xactLast >> tl.d.bits.source)(0) && edge.last(tl.d)
    val canSend = state === s_read && !xactBusy.andR

    val fullKeep = ~0.U(beatBytes.W)
    val loffset = Reg(UInt(byteAddrBits.W))
    val roffset = Reg(UInt(byteAddrBits.W))
    val lkeep = fullKeep << loffset
    val rkeep = fullKeep >> roffset
    val first = Reg(Bool())


    xactBusy := (xactBusy | Mux(tl.a.fire, xactOnehot, 0.U)) &    // 使用通道标志方法
                    ~Mux(tl.d.fire && edge.last(tl.d),
                          UIntToOH(tl.d.bits.source), 0.U)

    val helper = DecoupledHelper(tl.a.ready, io.alloc.ready)

    io.req.ready := state === s_idle
    io.alloc.valid := helper.fire(io.alloc.ready, canSend)
    io.alloc.bits.id := xactId
    io.alloc.bits.count := (1.U << (reqSize - byteAddrBits.U))
    tl.a.valid := helper.fire(tl.a.ready, canSend)
    tl.a.bits := edge.Get(
      fromSource = xactId,
      toAddress = sendaddr,
      lgSize = reqSize)._2

    val outLeftKeep = xactLeftKeep(tl.d.bits.source)
    val outRightKeep = xactRightKeep(tl.d.bits.source)

    io.out.valid := tl.d.valid
    io.out.bits.id := tl.d.bits.source                          // 返回master ID
    io.out.bits.data.data := tl.d.bits.data
    io.out.bits.data.keep := MuxCase(fullKeep, Seq(
      (edge.first(tl.d) && edge.last(tl.d)) -> (outLeftKeep & outRightKeep),
      edge.first(tl.d) -> outLeftKeep,
      edge.last(tl.d)  -> outRightKeep))
    io.out.bits.data.last := isLast
    tl.d.ready := io.out.ready
    io.resp.valid := state === s_resp
    io.resp.bits := true.B

    when (io.req.fire) {
      val req = io.req.bits
      val lastaddr = req.address + req.length
      val startword = req.address(addrBits-1, byteAddrBits)
      val endword = lastaddr(addrBits-1, byteAddrBits) +
                      Mux(lastaddr(byteAddrBits-1, 0) === 0.U, 0.U, 1.U)

      loffset := req.address(byteAddrBits-1, 0)
      roffset := Cat(endword, 0.U(byteAddrBits.W)) - lastaddr
      first := true.B

      sendaddr := Cat(startword, 0.U(byteAddrBits.W))
      sendlen  := Cat(endword - startword, 0.U(byteAddrBits.W))
      sendpart := req.partial
      state := s_read

      assert(req.length > 0.U, s"request length must be >0")
    }

    when (tl.a.fire) {
      val reqBytes = 1.U << reqSize
      sendaddr := sendaddr + reqBytes
      sendlen  := sendlen - reqBytes
      when (sendlen === reqBytes) {
        xactLast := (xactLast & ~xactOnehot) | Mux(sendpart, 0.U, xactOnehot)
        xactRightKeep(xactId) := rkeep
        state := s_resp
      } .otherwise {
        xactLast := xactLast & ~xactOnehot
        xactRightKeep(xactId) := fullKeep
      }
      when (first) {
        first := false.B
        xactLeftKeep(xactId) := lkeep
      } .otherwise {
        xactLeftKeep(xactId) := fullKeep
      }
    }

    when (io.resp.fire) {
      state := s_idle
    }
  }
}

class StreamWriteRequest extends Bundle {
  val address = UInt(48.W)
  val length = UInt(16.W)
  val strideMod = Bool()    // TODO: 没有使用到，需要剔除该变量
}

class StreamInData(val w: Int) extends Bundle{                // 添加mask功能替代原来的StreamChannel
  val data = UInt(w.W)
  val mask = UInt(8.W)
}
class StreamWriter(nXacts: Int, maxBytes: Int)
    (implicit p: Parameters) extends LazyModule {
  val node = TLHelper.makeClientNode(
    name = "stream-writer", sourceId = IdRange(0, nXacts))

  lazy val module = new LazyModuleImp(this) {
    val (tl, edge) = node.out(0)
    val dataBits = tl.params.dataBits
    val beatBytes = dataBits / 8
    val byteAddrBits = log2Ceil(beatBytes)
    val addrBits = tl.params.addressBits
    val lenBits = 16

    val io = IO(new Bundle {
      val req = Flipped(Decoupled(new StreamWriteRequest))
      val resp = Decoupled(UInt(lenBits.W))
      val in = Flipped(Decoupled(new StreamInData(dataBits)))
    })

    val s_idle :: s_data :: s_resp :: Nil = Enum(3)
    val state = RegInit(s_idle)
    // @lyq on
    //printf("@wDMA: writer state = %d\n", state)

// 处理头部不beatBytes对齐的数据
    val length = Reg(UInt(lenBits.W))
    val baseAddr = Reg(UInt(addrBits.W))
    val offset = Reg(UInt(addrBits.W))
    val addrMerged = baseAddr + offset
    val bytesToSend = length - offset
    val baseByteOff = baseAddr(byteAddrBits-1, 0)
    val byteOff = addrMerged(byteAddrBits-1, 0)
    val extraBytes = Mux(baseByteOff === 0.U, 0.U, beatBytes.U - baseByteOff)

// 可用的通道
    val xactBusy = RegInit(0.U(nXacts.W))
    val xactOnehot = PriorityEncoderOH(~xactBusy)
    val xactId = OHToUInt(xactOnehot)

// 请求的beat编号
    val maxBeats = maxBytes / beatBytes
    val beatIdBits = log2Ceil(maxBeats)

// mask 功能定义
    val strideMod = RegInit(false.B)
    val overhangMask = RegInit(0.U(beatBytes.W))

// tileLink 请求寄存器
    val beatsLeft = Reg(UInt(beatIdBits.W))
    val headAddr = Reg(UInt(addrBits.W))
    val headXact = Reg(UInt(log2Ceil(nXacts).W))
    val headSize = Reg(UInt(log2Ceil(maxBytes + 1).W))

    val newBlock = beatsLeft === 0.U
    val canSend = !xactBusy.andR || !newBlock

    val reqSize = MuxCase(0.U,                                                  // 以addrMerged对齐的数据大小发送
      (log2Ceil(maxBytes) until 0 by -1).map(lgSize =>                          // 不包括0
          (addrMerged(lgSize-1,0) === 0.U &&
            (bytesToSend >> lgSize.U) =/= 0.U) -> lgSize.U))

    xactBusy := (xactBusy | Mux(tl.a.fire && newBlock, xactOnehot, 0.U)) &    // 一个block子请求，使用一条通道
                    ~Mux(tl.d.fire, UIntToOH(tl.d.bits.source), 0.U)          // 释放掉占用的通道

    val overhang = RegInit(0.U(dataBits.W))
    //val sendTrail = bytesToSend <= extraBytes
    val sendTrail = bytesToSend <= extraBytes && (offset =/= 0.U)
    // printf("@DMA: sendTrail = %d, bytesToSend = %d, extraBytes = %d, offset = %d\n", sendTrail, bytesToSend, extraBytes, offset)
    val fulldata = (overhang |  (io.in.bits.data << Cat(baseByteOff, 0.U(3.W))))
    val fullmask = overhangMask | (io.in.bits.mask << baseByteOff)              // mask 为 1 ，对应字节回写

// 同一个tileLink 请求只允许mask 和data 发送改变
    val fromSource = Mux(newBlock, xactId, headXact)
    val toAddress = Mux(newBlock, addrMerged, headAddr)
    val lgSize = Mux(newBlock, reqSize, headSize)
    val wdata = fulldata(dataBits-1, 0)
    val mask = fullmask(beatBytes-1, 0)
    // val wmask = Cat((0 until beatBytes).map(                                    // wmask 为 1 ，对应字节回写
    //   i => (i.U >= byteOff) && (i.U < bytesToSend)).reverse)
     val wmask = Cat((0 until beatBytes).map(
        i => (i.U >= byteOff) && (i.U < bytesToSend + byteOff)).reverse)
    val strideMask = wmask & Mux(sendTrail, overhangMask, mask)                   // strideMod 下对应的对齐'wmask'
    val wpartial = !wmask.andR
    //printf("@DMA: wmask = %x, byteOff = %d, bytesToSend = %d, sendTrail = %d\n", wmask, byteOff, bytesToSend, sendTrail)

    val putPartial = edge.Put(
      fromSource = xactId,
      toAddress = addrMerged & ~(beatBytes-1).U(addrBits.W),
      lgSize = log2Ceil(beatBytes).U,
      data = Mux(sendTrail, overhang, wdata),
      //mask = Mux(strideMod, strideMask, wmask))._2
      mask = strideMask)._2
    // printf("@DMA: putPartial-> addr = %x, lgSize = %d, data = %x(s%d-o%x-w%x), mask = %x(m%d-sm%x-wm%x)\n",
    //    addrMerged & ~(beatBytes-1).U(addrBits.W), log2Ceil(beatBytes).U, Mux(sendTrail, overhang, wdata), sendTrail, overhang, wdata, Mux(strideMod, strideMask, wmask), strideMod, strideMask, wmask)

    val putFull = edge.Put(
      fromSource = fromSource,
      toAddress = toAddress,
      lgSize = lgSize,
      data = wdata,
      mask = mask)._2                                                           // mask 过滤无效数据
    // printf("@DMA: putFull-> addr = %x, lgSize = %d, data = %x, mask = %x\n",
    //    toAddress, lgSize, wdata, mask)

    io.req.ready := state === s_idle
    tl.a.valid := (state === s_data) && (io.in.valid || sendTrail) && canSend
    tl.a.bits := Mux(wpartial, putPartial, putFull)
    tl.d.ready := xactBusy.orR
    io.in.ready := state === s_data && canSend && !sendTrail && tl.a.ready
    //io.in.ready := state === s_data && canSend && (!sendTrail || (sendTrail && PopCount(wmask)>=bytesToSend)) && tl.a.ready
    io.resp.valid := state === s_resp && !xactBusy.orR                        // 等待请求完成，才response
    io.resp.bits := length
    // printf("@wDMA: tl.a -> fire = %d(v%d-r%d)\n", tl.a.fire, tl.a.valid, tl.a.ready)
    // printf("@wDMA: io.in -> fire = %d(v%d-r%d), bits = %x\n", io.in.fire, io.in.valid, io.in.ready, io.in.bits.data)

    when (io.req.fire) {
      offset := 0.U
      baseAddr := io.req.bits.address
      length := io.req.bits.length
      beatsLeft := 0.U
      strideMod := io.req.bits.strideMod
      overhang := 0.U                       // overhang 不初始化会导致当前请求受上次请求影响
      overhangMask := 0.U
      state := s_data
      // printf("@DMA: writer req-> baseAddr = %x, length = %d, strideMod = %d\n", io.req.bits.address, io.req.bits.length, io.req.bits.strideMod)
    }

    when (tl.a.fire) {
      when (!newBlock) {
        beatsLeft := beatsLeft - 1.U
      } .elsewhen (reqSize > byteAddrBits.U) {              // 请求大小大于 2^3 个字节时，才会进入
        val nBeats = 1.U << (reqSize - byteAddrBits.U)      // reqSize 包含beats数： 请求字节数 / beatBytes
        beatsLeft := nBeats - 1.U                           // 每个block，初始化beatsLeft
        headAddr := addrMerged
        headXact := xactId
        headSize := reqSize
      }

      val bytesSent = PopCount(wmask)
      offset := offset + bytesSent
      overhang := fulldata >> dataBits.U                    // fulldata 中剩下没有发送的字节
      overhangMask := fullmask >> beatBytes.U               // mask 中 1的数据将不会被发送
      
      when (bytesSent === bytesToSend) { state := s_resp }
      // printf("@DMA: tlfire -> wpartial = %d, toAddress = %x\n", wpartial, Mux(wpartial, addrMerged & ~(beatBytes-1).U(addrBits.W), toAddress))
      // printf("@DMA: tlfire -> wdata = %x, overhang = %x, hfulldata = %x, lfulldata = %x\n", wdata, overhang, fulldata >> dataBits.U, fulldata(63, 0))      
    }
    
    //when (io.resp.fire) { state := s_idle }       // 避免请求不能连续发送
    when (state === s_resp) { state := s_idle }    
    
    // test write mask
    // printf("@DMA: state = %d, xactBusy = %x, xactId = %d, fromSource = %d\n", state, xactBusy, xactId, fromSource)
    // printf("@DMA: io.in -> fire = %d(v%d-r%d), data = %x, mask = %x\n", io.in.fire, io.in.valid, io.in.ready, io.in.bits.data, io.in.bits.mask)
    // printf("@DMA: tl.a -> fire = %d(v%d-r%d), sourceID = %d\n", tl.a.fire, tl.a.valid, tl.a.ready, Mux(wpartial, xactId, fromSource))
    // printf("@DMA: tl.d -> fire = %d(v%d-r%d), sourceID = %d\n", tl.d.fire, tl.d.valid, tl.d.ready, tl.d.bits.source)
    
    // printf("@DMA: mask = %x, wmask = %x, strideMask = %x, overhangMask = %x\n", mask, wmask, strideMask, overhangMask)
    // printf("@DMA: h_fullmask = %x, l_fullmask = %x\n", (fullmask>>8.U)(7,0), fullmask(7,0))
  }
}