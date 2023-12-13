package memaccer
// @lyq test
 object DMAConfig{
     val nXacts: Int = 8            // 正在处理的请求数
     val maxBytes: Int = 64         //  cache line大小
     val outFlits: Int = 32        // 读数据队列大小
     val inFlits: Int = 32         // 写数据队列大小
     val rNwords: Int = 64         // 读缓存大小
     val wNwords: Int = 64         // 写缓存大小
     val dataBits: Int = 64        // 数据位宽

     // 支持解耦获取设置的队列，直接给加速器使用
     val reqQEntris = 8
     val respQEntris = 8
     val rQEntris = 64
     val wQEntris = 8
     val idxQEntris = 64
 }

//object DMAConfig{
//    val nXacts: Int = 4            // 正在处理的请求数
//    val maxBytes: Int = 64         //  cache line大小
//    val outFlits: Int = 4        // 读数据队列大小
//    val inFlits: Int = 4         // 写数据队列大小
//    val rNwords: Int = 4         // 读缓存大小
//    val wNwords: Int = 4         // 写缓存大小
//    val dataBits: Int = 64        // 数据位宽
//
//    // 支持解耦获取设置的队列，直接给加速器使用
//    val reqQEntris = 1
//    val respQEntris = 1
//    val rQEntris = 1
//    val wQEntris = 1
//    val idxQEntris = 1
//}