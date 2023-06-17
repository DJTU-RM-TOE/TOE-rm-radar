#include <opencv2/opencv.hpp>

#include <string>
#include <iostream>
#include <fstream>

#include "radar_robot_detector/init_trt.h"



int main(int argc, char **argv)
{
    onnx_loader("/home/evence/ros2_ws/toe_ctrl/src/TOE-rm-radar/radar_robot_detector/model/best.onnx");
    
    // build engine
    // 构建engine
    IBuilderConfig *config = builder->createBuilderConfig();

    config->setMemoryPoolLimit(MemoryPoolType::kWORKSPACE, 1U << 20);

    IHostMemory *serializedModel = builder->buildSerializedNetwork(*network, *config);

    //
    // 由于序列化引擎包含权重的必要副本，因此不再需要解析器、网络定义、构建器配置和构建器，可以安全地删除它们：
    delete parser;
    delete network;
    delete config;
    delete builder;

    delete serializedModel;

    IRuntime *runtime = createInferRuntime(logger);

    return 0;
}