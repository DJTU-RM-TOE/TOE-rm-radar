#ifndef __INIT_TRT__
#define __INIT_TRT__

#include <NvInfer.h>
#include "NvOnnxParser.h"

#include <string>
#include <iostream>

using namespace nvinfer1;
using namespace nvonnxparser;

// 实例化记录器界面
class Logger : public ILogger
{
    void log(Severity severity, const char *msg) noexcept override
    {
        // suppress info-level messages
        if (severity <= Severity::kWARNING)
            std::cout << msg << std::endl;
    }
} logger;

IBuilder *builder = createInferBuilder(logger);

// 创建网络定义
uint32_t flag = 1U << static_cast<uint32_t>(NetworkDefinitionCreationFlag::kEXPLICIT_BATCH);

INetworkDefinition *network = builder->createNetworkV2(flag);

// load onnx
// 加载onnx

IParser *parser = createParser(*network, logger);
void onnx_loader(const char *modelFilePath)
{
    parser->parseFromFile(modelFilePath, static_cast<int32_t>(ILogger::Severity::kWARNING));
    for (int32_t i = 0; i < parser->getNbErrors(); ++i)
    {
        std::cout << parser->getError(i)->desc() << std::endl;
    }
}

#endif
