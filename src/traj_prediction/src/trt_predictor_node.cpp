#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/ColorRGBA.h>
#include <NvInfer.h>
#include <cuda_runtime_api.h>
#include <fstream>
#include <vector>

using namespace nvinfer1;

class Logger : public ILogger
{
    // TensorRT 的日志回调函数，重写 log 方法
    void log(Severity severity, const char *msg) noexcept override
    {
        // 只打印警告及以上级别的 TRT 系统日志到 ROS_WARN 中
        if (severity <= Severity::kWARNING)
            ROS_WARN("[TRT] %s", msg);
    }
} gLogger; // 实例化全局唯一的日志捕获器

// 基于 TensorRT 的快速轨迹推理机节点核心类
class TrtPredictor
{
private:
    // TensorRT 推理环境的三大核心组件指针
    IRuntime *runtime_{nullptr};          // 推理运行时环境
    ICudaEngine *engine_{nullptr};        // 反序列化解析出的引擎
    IExecutionContext *context_{nullptr}; // 能够实际发起计算的上下文环境

    void *buffers_[2];               // [0]为输入显存，[1]为输出显存的指针
    int input_index_, output_index_; // 输入与输出张量在引擎中的绑定索引号

    // 神经网络维度规格参数
    int HIST_LEN; // 输入的历史轨迹帧数 (如 10)
    int FUT_LEN;  // 欲输出的未来轨迹帧数 (如 20)
    int FEAT_DIM; // 每个历史点的特征维度数 (通常为 6，xyz等)
    int OUT_DIM;  // 每个预测点的坐标以及方差维度数 (例如 x,y,z,std_x,std_y,std_z 为 6)

    // 配置读入的启动传参
    std::string engine_path_;     // TensorRT engine 权重的存放路径
    std::string mot_topic_name_;  // 感知端多目标追踪的数据流输入话题名
    std::string pred_topic_name_; // 往外发射的轨迹预测数据流话题名

    // ROS 句柄及通讯总线组件
    ros::NodeHandle nh_;
    ros::Publisher pred_pub_; // 预测出的点连线数组播发器
    ros::Subscriber mot_sub_; // 订阅历史跟踪坐标信息的监听器

public:
    TrtPredictor() : nh_("~")
    {
        // 1. 读取 YAML 参数文件或启动命令中覆写的运行参数并赋予默认值
        nh_.param<std::string>("engine_path", engine_path_, "/home/yyf/TransEgo/src/traj_prediction/models/transformer_model.engine");
        nh_.param<int>("hist_len", HIST_LEN, 10);
        nh_.param<int>("fut_len", FUT_LEN, 20);
        nh_.param<int>("feat_dim", FEAT_DIM, 6);
        nh_.param<int>("out_dim", OUT_DIM, 3);
        nh_.param<std::string>("mot_topic_name", mot_topic_name_, "/mot_markers");
        nh_.param<std::string>("pred_topic_name", pred_topic_name_, "/traj_prediction/predicted_trajectories");

        // 2. 加载 .engine 引擎文件序列化字节流
        std::ifstream file(engine_path_, std::ios::binary);
        if (file.good())
        {
            file.seekg(0, std::ios::end);
            size_t size = file.tellg(); // 获取文件大小
            file.seekg(0, std::ios::beg);
            std::vector<char> trtModelStream(size);
            file.read(trtModelStream.data(), size); // 读进内存
            file.close();

            // 3. 通过 Trt 规范还原工作引擎以及获取出入接口索引
            runtime_ = createInferRuntime(gLogger);
            engine_ = runtime_->deserializeCudaEngine(trtModelStream.data(), size);
            context_ = engine_->createExecutionContext();
            input_index_ = engine_->getBindingIndex("input_tensor");   // 读取代码约定的底层输入张量槽位名
            output_index_ = engine_->getBindingIndex("output_tensor"); // 读取代码约定的底层输出张量槽位名

            // 4. 为张量数据流预先在 GPU 上划定并全量绑定输入输出的高速显存 (Cuda Malloc)
            // 我们分配单Batch的显存，对于多目标采取循环单Batch推断（也可视显存修改为Dynamic Batching）
            cudaMalloc(&buffers_[input_index_], 1 * HIST_LEN * FEAT_DIM * sizeof(float));
            cudaMalloc(&buffers_[output_index_], 1 * FUT_LEN * OUT_DIM * sizeof(float));
        }

        // 5. 挂载收发端开始通讯循环 (Node init 结束)
        pred_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(pred_topic_name_, 1);
        mot_sub_ = nh_.subscribe(mot_topic_name_, 1, &TrtPredictor::motCallback, this);
    }

    ~TrtPredictor()
    {
        // 析构函数中：严密注销释放 GPU 中的残留数据缓存并清退 Trt 指针
        cudaFree(buffers_[input_index_]);
        cudaFree(buffers_[output_index_]);
        if (context_)
            context_->destroy();
        if (engine_)
            engine_->destroy();
        if (runtime_)
            runtime_->destroy();
    }

    // 真机或仿真环境传来的 MOT 回调处理函数（主运算循环）
    void motCallback(const visualization_msgs::MarkerArray::ConstPtr &msg)
    {
        if (msg->markers.empty()) // 无有效对象则退回
            return;

        int num_targets = msg->markers.size() / 2;                  // 除去 text marker 假设真实数量为 markers.size()/2 或者只遍历类型为 CUBE 的
        visualization_msgs::MarkerArray pred_markers;               // 准备发送的大包数据阵列
        ros::Time current_time = msg->markers.front().header.stamp; // 截取感知帧对齐时刻

        // 简化的逻辑，只挑选带有几何物理体积特征的 CUBE 抓取进行计算循环
        int target_idx = 0;
        for (const auto &m : msg->markers)
        {
            if (m.type != visualization_msgs::Marker::CUBE) // 跳过文字等装饰Marker
                continue;

            // 划定等体积单浮点连续长条内存以容纳向 GPU 交接的数据
            std::vector<float> input_data(HIST_LEN * FEAT_DIM, 0.0f);
            std::vector<float> output_data(FUT_LEN * OUT_DIM, 0.0f);

            // 步骤一：获取检测框的中心给所有历史填充（简版实现，仅用来测试链路闭环）
            for (int k = 0; k < HIST_LEN; ++k)
            {
                // 用当前这一帧单点去粗略填充长度为 HIST_LEN 的特征时序 (正式应用需改成保存的真历史)
                input_data[k * FEAT_DIM + 0] = m.pose.position.x;
                input_data[k * FEAT_DIM + 1] = m.pose.position.y;
                input_data[k * FEAT_DIM + 2] = m.pose.position.z;
            }

            // 步骤二：实施高性能 GPU 推理流水线
            // 用 cudaMemcpy 从主机内存(CPU Ram) 拷贝预备数据前往 GPU 显存！
            cudaMemcpy(buffers_[input_index_], input_data.data(), input_data.size() * sizeof(float), cudaMemcpyHostToDevice);

            // 无阻塞触碰点火 Trt 模型内的全连接/Attention 晶体管计算流
            context_->executeV2(buffers_);

            // 计算完结并把推测的连贯路线从 GPU 显存取回原主机内存(CPU Ram)
            cudaMemcpy(output_data.data(), buffers_[output_index_], output_data.size() * sizeof(float), cudaMemcpyDeviceToHost);

            // 步骤三：利用返回得到的新生点阵，构建可视化预测连线/安全方差球阵列 Marker
            visualization_msgs::Marker marker;
            marker.header.frame_id = "world";
            marker.header.stamp = current_time; // 绝对预测起始时间对齐给Ego
            marker.ns = "prediction";
            marker.id = m.id;                                      // 给不同目标赋予独立继承的Marker ID以防覆盖！
            marker.type = visualization_msgs::Marker::SPHERE_LIST; // SPHERE_LIST 能展示各散点并利用 scale 显示
            marker.action = visualization_msgs::Marker::ADD;
            marker.scale.x = 0.2;
            marker.scale.y = 0.2;
            marker.scale.z = 0.2;

            // 循环遍历取回的那一段含连续20（FUT_LEN）长的时间点序列
            for (int f = 0; f < FUT_LEN; ++f)
            {
                geometry_msgs::Point pt;
                // 按位剥离预测出的 [x, y, z] 的时序点塞入 marker
                pt.x = output_data[f * OUT_DIM + 0];
                pt.y = output_data[f * OUT_DIM + 1];
                pt.z = output_data[f * OUT_DIM + 2];
                marker.points.push_back(pt);

                // 判断是否支持 6 维扩容（带有后三维附带的 Gaussian Variance 方差不确定度评估）
                std_msgs::ColorRGBA c;
                if (OUT_DIM >= 6)
                {
                    // 借用将原本表达绘图渲染色的 RGB (因为是无用花边)，强行顶替用来传输重要浮点指标 std_xyz！
                    c.r = output_data[f * OUT_DIM + 3];
                    c.g = output_data[f * OUT_DIM + 4];
                    c.b = output_data[f * OUT_DIM + 5];
                }
                else
                {
                    // 退让防报错假体保底默认值
                    c.r = 1.0;
                    c.g = 0.5;
                    c.b = 0.0;
                }
                c.a = 1.0;
                marker.colors.push_back(c); // 对应每个 SPHERE 下发独立带“方差”的渲染色
            }
            // 将此目标的完整单道预测轨线挂入发送缓冲区阵列
            pred_markers.markers.push_back(marker);
            target_idx++;
        }

        // 步骤四：整体广播给 Ego-Planner，发出的 MarkerArray 之中已携带所有行人的空间偏移量和标准方差
        pred_pub_.publish(pred_markers);
    }
};

int main(int argc, char **argv)
{
    // ROS常规套路：节点初始化、实例化工作对象并阻塞死循环等待接收外部激发回调函数
    ros::init(argc, argv, "trt_predictor_node");
    TrtPredictor predictor;
    ros::spin();
    return 0;
}
