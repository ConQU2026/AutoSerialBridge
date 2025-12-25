#include <gtest/gtest.h>
#include <vector>
#include <chrono>
#include "serial_pkg/packet_handler.hpp"
#include "protocol.h"

using namespace auto_serial_bridge;

struct BenchmarkCmdVel {
    float linear_x;
    float angular_z;
};

// PacketHandler 性能基准测试
// 测量打包和解析 N 个数据包的时间
TEST(PerformanceTest, BatchProcessingThroughput) {
    PacketHandler handler(4096);
    BenchmarkCmdVel data = {1.23f, -4.56f};
    
    // 预先准备一个有效的数据包缓冲区以节省时间
    std::vector<uint8_t> one_packet = handler.pack(PACKET_ID_CMDVEL, data);
    
    const int kIterations = 1000000;
    
    auto start = std::chrono::high_resolution_clock::now();
    
    for (int i = 0; i < kIterations; ++i) {
        // 1. 打包 (模拟 MCU 或 ROS 端打包)
        std::vector<uint8_t> packed = handler.pack(PACKET_ID_CMDVEL, data);
        
        // 2. 投喂 (模拟接收)
        handler.feed_data(packed);
        
        // 3. 解析 (模拟处理)
        Packet pkt;
        bool result = handler.parse_packet(pkt);
        
        if (!result) {
            FAIL() << "Failed to parse packet at iteration " << i;
        }
    }
    
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
    
    double avg_time_us = (double)duration * 1000.0 / kIterations;
    double pps = kIterations / ((double)duration / 1000.0);
    
    std::cout << "[ PERF ] Processed " << kIterations << " packets in " << duration << " ms" << std::endl;
    std::cout << "[ PERF ] Average time per packet: " << avg_time_us << " us" << std::endl;
    std::cout << "[ PERF ] Throughput: " << pps << " packets/sec" << std::endl;

    EXPECT_GT(pps, 10000.0);
}
