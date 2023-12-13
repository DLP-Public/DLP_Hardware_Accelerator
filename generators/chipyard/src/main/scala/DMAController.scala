package memaccer

import chisel3._
import chisel3.util._
import freechips.rocketchip.config.Parameters   // Parameters
import freechips.rocketchip.diplomacy._         // LazyModule
import freechips.rocketchip.tilelink._          // TLIdentityNode



class DMAreqBundle(n:Int) extends Bundle{
    // 每次请求需要处理n个数
    val rid = UInt(3.W)             // 请求号
    val ren = UInt(1.W)             // 1: read， 0：write
    val baseAddr = UInt(64.W)       // 基地址
    val lgElem = UInt(2.W)          // 元素大小 0：1B，1：2B，2：4B，3：8B
    val elems = UInt(16.W)          // 元素总数
    val stride = UInt(16.W)         // 步幅
    val vmask = Vec(n, Bool())

    // indexed 访存模式
    val idxMod = UInt(1.W)          // 1: indexed, 0:stride
    // val idxAddrMod = UInt(1.W)      // 0: 间接地址偏移量使用stride patern获取， 1：偏移量由RoCC/加速器传递进来
    // val idxBaseAddr = UInt(64.W)    // 间接基址偏移量地址
    // val idxLgElem = UInt(2.W)       // 间接地址数组元素类型

    // 其他的一些参数配置
    // val rbackup = UInt(26.W)     // 备用bits
    val maxReqBytes = UInt(2.W)     // 0->8B, 1->16B, 2->32B, 3->64B
    val strideMod = UInt(2.W)       //non-unit stride 模式处理方式，0->优化，1->固定请求大小maxReqBytes，2->请求单元素
    // @lyq modify
    //override def cloneType: this.type = new DMAreqBundle(n).asInstanceOf[this.type]
    //new DMAreqBundle(n).asInstanceOf[this.type]
}

class DMArespBundle extends Bundle{
    val rid = UInt(3.W)
}

class DMAController(implicit p:Parameters) extends LazyModule{
    import DMAConfig._
    import ScratchpadConfig._
    // 内存访问节点
    val reader = LazyModule(new StreamReader(nXacts, outFlits, maxBytes)(p))
    val writer = LazyModule(new StreamWriter(nXacts, maxBytes)(p)) 
    val node = TLXbar()
    //@lyq node
    node := TLBuffer() := reader.node
    node := TLBuffer() := writer.node

    // Controller逻辑实现
    lazy val module = new LazyModuleImp(this){
         val io = IO(new Bundle{
             val req = Flipped(Decoupled(new DMAreqBundle(nbank)))
             val resp = Decoupled(new DMArespBundle)
             val dataR = Decoupled(UInt(64.W))
             val dataW = Flipped(Decoupled(UInt(64.W)))
         })

        // 主状态机
        val m_IDLE :: m_UPDATE_REQ :: m_READ :: m_WRITE :: m_DONE :: Nil = Enum(5)// todo: idle 与 update_req之前添加请求调度
        val m_state = RegInit(m_IDLE)   
        val r_idle :: r_read :: r_rQfull :: Nil = Enum(3)
        val r_state = RegInit(r_idle)   
        val w_idle :: w_write :: Nil = Enum(2)
        val w_state = RegInit(w_idle)
        // @lyq see if initial state is correct
        // printf("@ctrl: m_state = %d, r_state = %d, w_state =%d\n", m_state, r_state, w_state)

        // 请求队列
        val reqQ = Module(new Queue(new DMAreqBundle(nbank), reqQEntris))
        val req = Reg(new DMAreqBundle(nbank))   
        reqQ.io.enq <> io.req
        reqQ.io.deq.ready := m_state === m_DONE

        //@lyq print req.rid
        //printf("@ctrl: req.rid = [%d]\treq.baseAddr = [%d]\n", req.rid, req.baseAddr)
        //printf("@ctrl: req.rid = %d\n", reqQ.io.enq.bits.rid)

        // 响应队列
        val respQ = Module(new Queue(new DMArespBundle, respQEntris))
        io.resp <> respQ.io.deq
        respQ.io.enq.valid := m_state === m_DONE
        respQ.io.enq.bits := DontCare
        
        // 读队列   
        val readQ = Module(new Queue(chiselTypeOf(io.dataR.bits), rQEntris))
        io.dataR <> readQ.io.deq   
        readQ.io.enq.valid := false.B 
        readQ.io.enq.bits := DontCare

        // 写队列
        val writeQ = Module(new Queue(chiselTypeOf(io.dataW.bits), wQEntris))
        writeQ.io.enq <> io.dataW
        writeQ.io.deq.ready := false.B
        // printf("@ctrl: writeQ.enq->fire = %d(v%d-r%d) , bits = %x\n", writeQ.io.enq.fire, writeQ.io.enq.valid, writeQ.io.enq.ready, writeQ.io.enq.bits)
        // printf("@ctrl: writeQ.deq->fire = %d(v%d-r%d), bits = %x\n", writeQ.io.deq.fire, writeQ.io.deq.valid, writeQ.io.deq.ready, writeQ.io.deq.bits)

        // indexed 访存模式队列
        val idxQ = Module(new Queue(chiselTypeOf(io.dataW.bits), idxQEntris))
        idxQ.io.enq.valid := false.B
        idxQ.io.enq.bits := DontCare
        idxQ.io.deq.ready := false.B 



        
        // 变量定义       
        val haveIdxMod = req.idxMod === 1.U

        val totalBytes = req.elems << req.lgElem
        val byteCnt = RegInit(0.U(16.W))
        val issueByteCnt = RegInit(0.U(16.W))
        val issueAddr = RegInit(0.U(64.W))    
        val reqSize = RegInit(0.U(16.W))    // 每次内存请求大小
        val leftBytes = Mux(totalBytes > byteCnt, totalBytes - byteCnt, 0.U)
        val leftReadQBytes = (rQEntris.asUInt - readQ.io.count) << 3.U            // todo 参数化，每一项有8个字节
       
        val subBeatBytes = RegInit(0.U(16.W))
        
          
        val rdone = (m_state === m_READ) && (leftBytes === 0.U)
        val wdone = Wire(Bool())
        wdone := false.B  
        
        // 单个请求最大值 0->8B, 1->16B, 2->32B, 3->64B
        val cmdMaxReqBytes = Mux(req.ren===1.U, req.maxReqBytes, 3.U(2.W))                                
        // non-unit stride 模式处理方式
        // 0->优化，1->固定请求大小maxReqBytes，2->请求单元素
        val cmdNonUintMod = Mux(req.ren===1.U, req.strideMod, 0.U(2.W)) 
                                              
        val maxReqBytes = MuxCase(0.U, Seq(                                         // 请求最大值
            (cmdMaxReqBytes === 0.U) -> 8.U,
            (cmdMaxReqBytes === 1.U) -> 16.U,
            (cmdMaxReqBytes === 2.U) -> 32.U,
            (cmdMaxReqBytes === 3.U) -> 64.U))                                                 
        val elemBytes = 1.U << req.lgElem                                           // 元素大小
        val strideBytes = req.stride << req.lgElem                                  // 相邻元素距离
        //val subReqElems = (maxReqBytes + strideBytes - elemBytes) / strideBytes     // 请求包含元素个数
        val subReqElems = Mux(haveIdxMod, 1.U, (maxReqBytes + strideBytes - elemBytes) / strideBytes)
        val subReqElemsBytes = subReqElems << req.lgElem                                        // 请求有效元素字节数
        val subReqBytes = MuxCase(0.U, Seq(                                                     //  每次请求的大小
            //(cmdNonUintMod === 0.U) -> ((subReqElems - 1.U) * strideBytes + elemBytes),          // 普通优化
            (cmdNonUintMod === 0.U) -> Mux(subReqElems === 1.U, elemBytes, maxReqBytes),  
            (cmdNonUintMod === 1.U) -> maxReqBytes,                                               // 固定请求大小
            (cmdNonUintMod === 2.U) -> elemBytes,                                               // 每个请求一个元素
            (cmdNonUintMod === 3.U) -> 0.U))                                                    // todo: 进一步优化           
        val subReqElemsCnt = RegInit(0.U(16.W))
        
        val curNbits = RegInit(0.U(log2Ceil(64+1).W))
        def incWrap(cur: UInt, inc: UInt):UInt ={
            val unwrapped = cur +& inc
            Mux(unwrapped >= 64.U, 0.U, unwrapped)
        }        
        
        // 读取数据
        val alignDatValid = RegInit(false.B)
        alignDatValid := false.B                                        // 默认false.B, 当alignDat填满时为true.B
        val alignDat = RegInit(0.U(64.W))
        
        // indexed pattern 变量
        val idxElemCnt = RegInit(0.U(16.W))
        val wBackMod = RegInit(0.U(3.W))                                                                                                                                             

        // 状态转移
        m_state := MuxCase(m_state, Seq(
            ((m_state === m_IDLE) && reqQ.io.deq.valid)                         -> m_UPDATE_REQ,
            ((m_state === m_UPDATE_REQ) && (req.ren === 1.U))                   -> m_READ,
            ((m_state === m_UPDATE_REQ)  && (req.ren === 0.U))                  -> m_WRITE,
            ((m_state === m_READ && rdone) || (m_state === m_WRITE && wdone))   -> m_DONE,
            (m_state === m_DONE)                                                -> m_IDLE))    
        

        // 条件改变
        when(m_state === m_IDLE){
            reqSize := 0.U
            byteCnt := 0.U
            issueByteCnt := 0.U
            curNbits := 0.U
            idxElemCnt := 0.U
            wBackMod := 0.U
            r_state := r_idle
            w_state := w_idle
            req := reqQ.io.deq.bits
        }
        when(m_state === m_UPDATE_REQ){
            // todo 请求优先调度时，可以修改这里req赋值
            // printf("@ctrl: idxMod = %d, baseAddr = %x, rid = %d, ren = %d, lgElem = %d, elems = %d, stride = %d, maxReqBytes = %d, strideMod = %d\n", 
            //     req.idxMod, req.baseAddr, req.rid, req.ren, req.lgElem, req.elems, req.stride, req.maxReqBytes, req.strideMod)         
        }
        

/*************************************************************** 取数操作 ****************************************************************/
        // 连线
        val readio = reader.module.io    
        readio.req.valid := false.B        
        readio.req.bits.address := issueAddr(47,0).asUInt
        readio.req.bits.length := reqSize(14,0).asUInt
        readio.req.bits.partial := false.B
        readio.resp.ready := readio.resp.valid               
        readio.out.ready := readQ.io.enq.ready || leftBytes===0.U   // 读队列有空间或者当前请求完成了清除多余数据
        // 读取的数据保存到readQ
        readQ.io.enq.valid := alignDatValid 
        readQ.io.enq.bits := alignDat

        // @lyq check if readioReq link up
        // printf("@ctrl: readioReq->fire = %d(v%d-r%d), size = %d, addr = %x\n", readio.req.fire, readio.req.valid, readio.req.ready, readio.req.bits.length, readio.req.bits.address)
        // printf("@ctrl: readioOut->fire = %d(v%d-r%d), bits = %x\n", readio.out.fire, readio.out.valid, readio.out.ready, readio.out.bits.data)
        
        when(m_state === m_READ){
            val dat = readio.out.bits.data
            printf("@dma enter m_READ\n")
            val strideReqSize = Mux(req.stride === 1.U, 
                    // unit stride
                    Mux(leftBytes < leftReadQBytes, leftBytes, leftReadQBytes), // 连续请求允许的最大值 min(leftBytes, leftReadQBytes)
                    // non-unit stride
                    Mux(subReqElemsBytes > leftReadQBytes, 0.U, subReqBytes))   // 读队列存在空间时，发起subReqBytes大小的请求  
            val idxReqSize = Mux(writeQ.io.deq.valid && leftBytes < leftReadQBytes, elemBytes, 0.U)
            val tmpReqSize = Mux(haveIdxMod, idxReqSize, strideReqSize)                                                                                    
            when(r_state === r_idle && tmpReqSize > 0.U){
                r_state := Mux(leftBytes>0.U, r_read, r_idle)
                reqSize := tmpReqSize
                issueAddr := req.baseAddr
                subBeatBytes := 0.U
                subReqElemsCnt := 0.U
                issueByteCnt := 0.U
            }
            when(rdone){                    // 一旦rdone有效，m_state切换到m_done状态
                r_state := r_idle 
                issueByteCnt := 0.U  
                subBeatBytes := 0.U
                subReqElemsCnt := 0.U  
            }
            // 读队列虚满时，停止发送请求
            // 响应的请求数据将会分别保存到DMA读缓存和读队列当中，需要调整两者队列大小保证不会溢出            
            //readio.req.valid := (r_state === r_read) && (issueByteCnt < totalBytes) && (reqSize > 0.U)
            readio.req.valid := (r_state === r_read) && 
                                (issueByteCnt < totalBytes) && 
                                (reqSize > 0.U) &&
                                Mux(haveIdxMod, writeQ.io.deq.valid, true.B)
            //issueAddr := req.baseAddr + issueByteCnt * req.stride                // todo 考虑使用寄存器代替乘法
            issueAddr := Mux(haveIdxMod, 
                            req.baseAddr + (writeQ.io.deq.bits << req.lgElem),      // << 运算优先级比+低
                            req.baseAddr + issueByteCnt * req.stride )
            when(readio.req.fire){
                val curIssueByteCnt = issueByteCnt + Mux(haveIdxMod, elemBytes,
                                        Mux(req.stride === 1.U, reqSize, subReqElemsBytes))             
                r_state := Mux(tmpReqSize === 0.U && curIssueByteCnt < totalBytes, r_rQfull, r_state)
                reqSize := tmpReqSize
                //issueByteCnt := issueByteCnt + Mux(req.stride === 1.U, reqSize, subReqElemsBytes)
                issueByteCnt := curIssueByteCnt
                writeQ.io.deq.ready := haveIdxMod               // 请求发送完毕
            }
            when(tmpReqSize > 0.U && r_state === r_rQfull){
                r_state := r_read
                reqSize := tmpReqSize
            }            
            printf("@ctrl: r_state = %d, issueAddr = %x, reqSize = %d, issueByteCnt = %d\n", r_state, issueAddr, reqSize, issueByteCnt)
            printf("@ctrl: byteCnt = %d, totalBytes = %d\n", byteCnt, totalBytes);
            // printf("@ctrl: alignDatValid = %d, alignDat = %x, dat = %x\n", alignDatValid, alignDat, dat)
            // 处理响应数据
            
            when(readio.out.fire && (leftBytes > 0.U)){
                when(req.stride === 1.U && !haveIdxMod){
                    alignDat := dat
                    alignDatValid := true.B
                    byteCnt := byteCnt + 8.U
                }.otherwise{                    
                    val tmp = subBeatBytes + strideBytes - elemBytes
                    //val beatElems = (tmp + 8.U) / strideBytes - tmp / strideBytes)
                    val beatElems =  Mux(haveIdxMod, 1.U, (tmp + 8.U) / strideBytes - tmp / strideBytes)
                    val idx = (subReqElemsCnt * strideBytes)(2,0):UInt
                    val beatVec = VecInit(dat(7,0), dat(15,8), dat(23,16), dat(31, 24),
                                        dat(39, 32), dat(47, 40), dat(55, 48), dat(63, 56))
                    val incBits = (beatElems << req.lgElem) << 3.U                                      //填充了多少bits
                    subBeatBytes := Mux(subReqBytes <= subBeatBytes + 8.U, 0.U, subBeatBytes + 8.U)
                    subReqElemsCnt := Mux(subReqElems <= subReqElemsCnt + beatElems, 0.U, subReqElemsCnt + beatElems)
                    byteCnt := byteCnt + (beatElems << req.lgElem)
                    val curAlignDat = Mux(curNbits === 0.U, 0.U, alignDat)
                    //@lyq change data type Bits -> UInt
                    switch(beatElems){                            
                        is(1.U){// uint8-64
                            alignDat := curAlignDat | 
                                Mux(req.lgElem === 0.U, beatVec(idx),
                                Mux(req.lgElem === 1.U, Cat(beatVec(idx+1.U), beatVec(idx)),
                                Mux(req.lgElem === 2.U, Cat(beatVec(idx+3.U), beatVec(idx+2.U), 
                                beatVec(idx+1.U), beatVec(idx)), dat))) << curNbits
                        }
                        is(2.U){// uint8-16
                            alignDat := curAlignDat |
                                Mux(req.lgElem === 0.U, Cat(beatVec(idx + strideBytes.asUInt), beatVec(idx)),
                                Cat(beatVec(idx + strideBytes.asUInt + 1.U), beatVec(idx + strideBytes.asUInt),
                                beatVec(idx+1.U), beatVec(idx))) << curNbits
                        }
                        is(3.U){// uint8
                            alignDat := curAlignDat | Cat(beatVec(idx + strideBytes.asUInt << 1.U),
                                beatVec(idx + strideBytes.asUInt), beatVec(idx)) << curNbits
                        }
                        is(4.U){// uint8
                            alignDat := curAlignDat | Cat(beatVec(6), beatVec(4),
                                beatVec(2), beatVec(0)) << curNbits
                        }
                    }
                    
                    curNbits := incWrap(curNbits, incBits)
                    alignDatValid := beatElems > 0.U && ((incWrap(curNbits, incBits) === 0.U) || 
                        (byteCnt + (beatElems << req.lgElem) >= totalBytes))
                    // printf("@ctrl: beatElems = %d, subBeatBytes = %d, subReqElemsCnt = %d\n", beatElems, subBeatBytes, subReqElemsCnt)
                    // printf("@ctrl: curNbits = %d, incBits = %d\n", curNbits, incBits)
                }

            }// end of when(readio.out.fire)

        }// end of when(m_state === m_READ)
        
/*************************************************************** 回写操作 ****************************************************************/
        val streamMod = req.stride === 1.U
        val singleMod = req.stride > 1.U && subReqElems === 1.U
        val strideMod = (req.stride > 1.U) && !(subReqElems === 1.U)
        val writeMod = Cat(streamMod, singleMod, strideMod)
        

        val wTotalBytes = Mux(streamMod || singleMod, totalBytes,
            totalBytes*req.stride - elemBytes*(req.stride - 1.U))   // 64B中包含两个或以上的元素，无效数据会经过mask过滤
        val walignDatValid = Wire(Bool())
        walignDatValid := false.B
        val walignDat = Wire(UInt(64.W))
        walignDat := 0.U
        val wmask = Wire(UInt(8.W))
        wmask := 0xff.U   

        // DMA写接口
        val writerio = writer.module.io
        writerio.req.valid := (m_state === m_WRITE) && (w_state === w_write) && (issueByteCnt < wTotalBytes)
        //writerio.req.bits.address := issueAddr(47, 0).asUInt
        writerio.req.bits.address := Mux(haveIdxMod, req.baseAddr + (idxQ.io.deq.bits << req.lgElem), issueAddr(47, 0).asUInt)
        writerio.req.bits.length := reqSize(14, 0).asUInt
        writerio.req.bits.strideMod := strideMod
        writerio.in.valid := walignDatValid
        writerio.in.bits.data := walignDat
        writerio.in.bits.mask := wmask
        writerio.resp.ready := writerio.resp.valid
        // printf("@ctrl: writerio.req -> fire = %d(v%d-r%d), address = %x, len = %d\n", 
        //     writerio.req.fire, writerio.req.valid, writerio.req.ready, writerio.req.bits.address, writerio.req.bits.length)
        // printf("@ctrl: writerio.in -> fire = %d(v%d-r%d), data = %x, mask = %x\n",
        //     writerio.in.fire, writerio.in.valid, writerio.in.ready, writerio.in.bits.data, writerio.in.bits.mask)
        // printf("@ctrl: idxElemCnt = %d, wBackMod(010) = %d\n", idxElemCnt, wBackMod)
        // printf("@ctrl: idxQ.enq->fire = %d(v%d-r%d), bits = %x\n", idxQ.io.enq.fire, idxQ.io.enq.valid, idxQ.io.enq.ready, idxQ.io.enq.bits)
        // indexed pattern 下将地址保存在idxQ        
        when(m_state === m_WRITE && haveIdxMod && idxElemCnt < req.elems){
            idxQ.io.enq <> writeQ.io.deq
            idxElemCnt := idxElemCnt + Mux(idxQ.io.enq.fire, 1.U, 0.U)
        }.elsewhen(m_state === m_WRITE && (!haveIdxMod || (haveIdxMod && idxElemCnt >= req.elems))){
            wBackMod := Mux(haveIdxMod, "b010".U, writeMod)     // 采用singleMod 处理indexed pattern
        }

        //val needVmask = !(req.vmask.asUInt.andR === 1.U)
        val ipe = Wire(UInt(log2Ceil(nbank).W))
        ipe := byteCnt >> req.lgElem      // 开始写回第ipe elem
        val vrmask = req.vmask
        val vrmaskT = (nbank - 1).U 
        val pad = 0.U(1.W)
        val vipe = VecInit(ipe & vrmaskT, (ipe+1.U) & vrmaskT, (ipe+2.U) & vrmaskT, (ipe+3.U) & vrmaskT,
            (ipe+4.U) & vrmaskT, (ipe+5.U) & vrmaskT, (ipe+6.U) & vrmaskT, (ipe+7.U) & vrmaskT)
        // wBackMod 处理
        switch(wBackMod){// wBackMod 非零时m_state = m_WRITE
            is("b100".U){// streamMod  
                // 发送请求              
                when(w_state === w_idle){
                    issueAddr := req.baseAddr
                    reqSize := totalBytes
                    w_state := w_write                    
                }
                when(w_state === w_write){
                    when(writerio.req.fire){
                        issueByteCnt := reqSize
                    }
                    // 数据传输
                    walignDatValid := writeQ.io.deq.valid
                    walignDat := writeQ.io.deq.bits

                    // DMA存储带掩码vmask的向量 
                    wmask := 0xff.U
                    val vm = req.vmask
                    switch(req.lgElem){
                        is(0.U){
                            wmask := Cat(vrmask(vipe(7)), vrmask(vipe(6)), vrmask(vipe(5)), vrmask(vipe(4)),
                                vrmask(vipe(3)), vrmask(vipe(2)), vrmask(vipe(1)), vrmask(vipe(0)))
                        }
                        is(1.U){
                            wmask := Cat(Fill(2, vrmask(vipe(3))), Fill(2, vrmask(vipe(2))),
                                Fill(2, vrmask(vipe(1))), Fill(2, vrmask(vipe(0))))
                        }
                        is(2.U){
                            wmask := Cat(Fill(4, vrmask(vipe(1))), Fill(4, vrmask(vipe(0))))
                        }
                        is(3.U){
                            wmask := Fill(8, vrmask(vipe(0)))
                        }
                    }
                
                    when(writerio.in.fire){
                        writeQ.io.deq.ready := true.B
                        val sendBytes = byteCnt + 8.U
                        byteCnt := sendBytes
                        when(sendBytes >= wTotalBytes){
                            wdone := true.B
                            w_state := w_idle
                        }
                        // printf("@ctrl:streamMod ipe%d, wmask%x\n", ipe, wmask)
                    }
                }// end of when(w_state === w_write)  
            }// end of "b100"
            is("b010".U){// singleMod
                when(w_state === w_idle){
                    issueAddr := req.baseAddr
                    issueByteCnt := 0.U
                    reqSize := elemBytes
                    w_state := w_write
                    curNbits := 0.U
                }

                when(w_state === w_write){
                    when(writerio.req.fire){
                        issueAddr := issueAddr + strideBytes
                        issueByteCnt := issueByteCnt + elemBytes
                        // 完成一次indexed 写回, idxQ出队一个元素
                        idxQ.io.deq.ready := true.B
                    }
                    walignDatValid := writeQ.io.deq.valid
                    walignDat := writeQ.io.deq.bits >> curNbits
                    val wmask1 = MuxCase(0xff.U, Seq(
                        (elemBytes === 1.U) -> "b00000001".U,
                        (elemBytes === 2.U) -> "b00000011".U,
                        (elemBytes === 4.U) -> "b00001111".U,
                        (elemBytes === 8.U) -> "b11111111".U))
                    // DMA存储带掩码vmask的向量                    
                    wmask := Mux(req.vmask(ipe), wmask1, "b00000000".U)                      
                   
                    when(writerio.in.fire){
                        val sendBytes = byteCnt + elemBytes
                        byteCnt := sendBytes
                        writeQ.io.deq.ready := sendBytes>0.U && (sendBytes(2,0)===0.U || sendBytes>=wTotalBytes)
                        curNbits := incWrap(curNbits, elemBytes<<3.U)
                        when(sendBytes >= wTotalBytes){
                            wdone := true.B
                            w_state := w_idle
                        }
                        // printf("@ctrl:singleMod vmask(%d)=%d, wmask1=%x, wmask=%x\n", ipe, req.vmask(ipe), wmask1, wmask)
                    }
                }
            }// end of "b010"
            is("b001".U){// strideMod

                val curNbytes = RegInit(0.U(4.W))
                

                when(w_state === w_idle){
                    issueAddr := req.baseAddr
                    issueByteCnt := 0.U
                    reqSize := wTotalBytes
                    w_state := w_write
                    curNbytes := 0.U
                    subBeatBytes := 0.U
                    subReqElemsCnt := 0.U
                }
                when(w_state === w_write){
                    when(writerio.req.fire){
                        issueAddr := issueAddr + wTotalBytes
                        issueByteCnt := wTotalBytes
                    }
                    walignDatValid := writeQ.io.deq.valid
                    val tmp = subBeatBytes + strideBytes - elemBytes
                    val beatElems =  (tmp + 8.U) / strideBytes - tmp / strideBytes
                    val idx = (subReqElemsCnt * strideBytes)(2,0)
                    val idxBits = idx << 3.U
                    val incBytes = beatElems << req.lgElem
                    val dat = writeQ.io.deq.bits
                    val datVec = VecInit(dat(7,0), dat(15,8), dat(23,16), dat(31, 24),
                                        dat(39, 32), dat(47, 40), dat(55, 48), dat(63, 56))
                    
                    ipe := subReqElemsCnt & (nbank - 1).U       // 写回第ipe个元素
                    switch(beatElems){
                        is(0.U){
                            walignDat := 0.U
                            wmask := "b00000000".U
                        }
                        is(1.U){ 
                            val sendDat = MuxCase(0.U, Seq(
                                (elemBytes===1.U) -> datVec(curNbytes),
                                (elemBytes===2.U) -> Cat(datVec(curNbytes+1.U), datVec(curNbytes)),
                                (elemBytes===4.U) -> Cat(datVec(curNbytes+3.U), datVec(curNbytes+2.U),datVec(curNbytes+1.U), datVec(curNbytes)),
                                (elemBytes===8.U) -> dat))                                                                      
                            walignDat := sendDat << idxBits
                            wmask := Mux(req.vmask(ipe), (~(0xff.U << elemBytes)) << idx, "b00000000".U)                            
                        }
                        is(2.U){// int8, int16
                            val wmask1 = Wire(UInt(8.W))
                            wmask1 := 0.U
                            when(req.lgElem === 0.U){
                                val sendDat1 = Cat(0.U(56.W),datVec(curNbytes))
                                val sendDat2 = Cat(0.U(56.W),datVec(curNbytes+1.U))
                                walignDat := ((sendDat2 << (strideBytes<<3.U)(6,0)) | sendDat1) << idxBits
                                wmask1 := (1.U(8.W)<<(idx + strideBytes(2,0))) | (1.U(8.W)<<idx)
                                //printf("@ctrl: mask = %x, part1 = %x, part2 = %x\n", wmask, 1.U<<(strideBytes(2,0)-1.U), 1.U<<idx)
                            }
                            when(req.lgElem === 1.U){
                                val sendDat1 = Cat(datVec(curNbytes+1.U), datVec(curNbytes))
                                val sendDat2 = Cat(datVec(curNbytes+3.U), datVec(curNbytes+2.U))
                                walignDat := Mux(req.stride === 3.U, 
                                            Cat(sendDat2, 0.U(32.W), sendDat1),
                                            Cat(0.U(16.W), sendDat2, 0.U(16.W), sendDat1))
                                wmask1 := Mux(req.stride === 3.U, "b11000011".U, "b00110011".U)                                
                            }
                            
                            wmask := wmask1 & Cat(Fill(4, req.vmask(vipe(1))), Fill(4, req.vmask(vipe(0))))

                        }// end of is(2.U)
                        is(3.U){// int8
                            walignDat := Mux(idx===0.U,
                                        Cat(0.U(8.W),   datVec(curNbytes+2.U),
                                            0.U(16.W),  datVec(curNbytes+1.U),
                                            0.U(16.W),  datVec(curNbytes)),
                                        Cat(datVec(curNbytes+2.U), 0.U(16.W), 
                                            datVec(curNbytes+1.U), 0.U(16.W),
                                            datVec(curNbytes), 0.U(8.W)))
                            //wmask := Mux(idx===0.U,"b01001001".U, "b10010010".U)                           
                            wmask := Cat(pad, vrmask(vipe(2)), pad, pad, 
                                vrmask(vipe(1)), pad, pad, vrmask(vipe(0))) << 
                                Mux(idx===0.U, 0.U, 1.U)
                        }
                        is(4.U){// int8
                            walignDat := Cat(0.U(8.W), datVec(curNbytes+3.U),
                                             0.U(8.W), datVec(curNbytes+2.U),
                                             0.U(8.W), datVec(curNbytes+1.U),
                                             0.U(8.W), datVec(curNbytes))
                            //wmask := "b01010101".U      // 最高位在前                            
                            wmask := Cat(pad, vrmask(vipe(3)), pad, vrmask(vipe(2)), 
                                pad, vrmask(vipe(1)), pad, vrmask(vipe(0)))
                        }
                    }// end of switch(beatElems)

                    when(writerio.in.fire){
                        val sendBytes = byteCnt + 8.U
                        byteCnt := sendBytes
                        when(sendBytes >= wTotalBytes){
                            wdone := true.B
                            w_state := w_idle
                        }
                        subBeatBytes := subBeatBytes + 8.U
                        subReqElemsCnt := subReqElemsCnt + beatElems
                        val nextNbytes = Mux(incBytes+curNbytes >= 8.U, 0.U, incBytes+curNbytes)
                        curNbytes := nextNbytes
                        writeQ.io.deq.ready := (nextNbytes === 0.U || sendBytes >= wTotalBytes)  && (wmask =/= 0x00.U)   // 发送有效的数据导致64bits数据消耗掉
                        // printf("@ctrl:strideMod wmask%x, ipe%d\n", wmask, ipe)
                    }

                }// end of when(w_state===w_write)
            }// end of "b001"
        }
        
    }

}