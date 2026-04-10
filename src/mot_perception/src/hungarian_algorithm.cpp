#include "../include/hungarian_algorithm.h"
#include <limits>
#include <cstdlib>
#include <iostream>
using namespace std;
HungarianAlgorithm::HungarianAlgorithm() {} // 缺省构造函数

// 求解二分图最优分配问题（输入代价矩阵，输出匹配关联索引）
double HungarianAlgorithm::Solve(std::vector<std::vector<double>> &DistMatrix, std::vector<int> &Assignment)
{
    unsigned int nRows = DistMatrix.size();    // 获取矩阵行数（历史追踪器数量）
    unsigned int nCols = DistMatrix[0].size(); // 获取矩阵列数（当前检测框数量）

    double *distMatrixIn = new double[nRows * nCols]; // 申请一维数组用于展平二维代价矩阵
    int *assignment = new int[nRows];                 // 申请数组存储每行的分配结果
    double cost = 0.0;                                // 初始化最优匹配总代价计算器

    // 遍历二维矩阵，按行优先法则展平填充到一维数组
    for (unsigned int i = 0; i < nRows; i++)
    {
        for (unsigned int j = 0; j < nCols; j++)
        {
            distMatrixIn[i * nCols + j] = DistMatrix[i][j]; // 填入代价元素
        }
    }

    // 调用底层核心 C 风格函数执行匈牙利算法最优解算
    assignmentoptimal(assignment, &cost, distMatrixIn, nRows, nCols);

    Assignment.clear(); // 清空旧的外部输出数组
    // 遍历行数，将底层的分配结果写回输出向量中
    for (unsigned int r = 0; r < nRows; r++)
    {
        Assignment.push_back(assignment[r]); // 填入追踪器 r 对应的检测框索引
    }

    delete[] distMatrixIn; // 释放展平矩阵的堆内存
    delete[] assignment;   // 释放分配结果的堆内存
    return cost;           // 返回匹配成功后的整体综合代价值
}

// 匈牙利配对算法核心实现，基于状态机和增广路寻优
void HungarianAlgorithm::assignmentoptimal(int *assignment, double *cost, double *distMatrix, int nOfRows, int nOfColumns)
{
    double *distMatrixTemp, *distMatrixEnd, *columnEnd, value, minValue;           // 定义矩阵遍历指针与极值缓存
    bool *coveredColumns, *coveredRows, *starMatrix, *newStarMatrix, *primeMatrix; // 定义状态掩码（覆盖与图标记矩阵）
    int nOfElements, minDim, row, col;                                             // 定义矩阵元素总数、最小维度及行列游标

    *cost = 0; // 重置初始总代价为 0
    for (row = 0; row < nOfRows; row++)
    {
        assignment[row] = -1; // 初始化所有行的分配结果为未匹配 (-1)
    }

    nOfElements = nOfRows * nOfColumns; // 计算代价矩阵的元素总个数

    distMatrixTemp = (double *)malloc(nOfElements * sizeof(double)); // 在堆上开辟临时矩阵，防止污染原数据
    distMatrixEnd = distMatrixTemp + nOfElements;                    // 定位临时矩阵的越界尾指针

    // 深拷贝外部代价矩阵到内部临时矩阵
    for (row = 0; row < nOfElements; row++)
    {
        value = distMatrix[row];     // 获取原矩阵节点值
        distMatrixTemp[row] = value; // 赋值给临时矩阵
    }

    // 初始化各类状态布尔矩阵，默认全量置 0 (false)
    coveredColumns = (bool *)calloc(nOfColumns, sizeof(bool)); // 记录被覆盖的列
    coveredRows = (bool *)calloc(nOfRows, sizeof(bool));       // 记录被覆盖的行
    starMatrix = (bool *)calloc(nOfElements, sizeof(bool));    // 记录打上星标（有效独立）的零元素
    primeMatrix = (bool *)calloc(nOfElements, sizeof(bool));   // 记录打上撇标（待定）的零元素
    newStarMatrix = (bool *)calloc(nOfElements, sizeof(bool)); // 记录推演中的颠倒星相矩阵

    // 步骤 0：矩阵基础变换，通过行列减去极小值，人为制造足够的零元素
    if (nOfRows <= nOfColumns) // 如果行数小于等于列数
    {
        minDim = nOfRows; // 取行数作为有效最小维度匹配上限

        for (row = 0; row < nOfRows; row++) // 纵向遍历所有行
        {
            distMatrixEnd = distMatrixTemp + (row + 1) * nOfColumns; // 标记当前行的尾端点
            minValue = *(distMatrixEnd - nOfColumns);                // 假定行首元素为最小值
            // 在本行内寻找真实的最小代价值
            for (double *t = distMatrixEnd - nOfColumns + 1; t < distMatrixEnd; t++)
            {
                if (*t < minValue) // 发现更小的值
                {
                    minValue = *t; // 更新最小值基准
                }
            }
            // 本行所有元素统一减去该行最小值，必然产生至少一个 0
            for (double *t = distMatrixEnd - nOfColumns; t < distMatrixEnd; t++)
            {
                *t -= minValue; // 执行偏移相减
            }
        }
    }
    else // 如果行数大于列数（检测框少，追踪器多）
    {
        minDim = nOfColumns; // 取列数作为有效最小维度匹配上限

        for (col = 0; col < nOfColumns; col++) // 横向遍历所有列
        {
            double *temp = distMatrixTemp + col; // 定位至列首
            minValue = *temp;                    // 假定列首元素为最小值
            // 在本列内寻找真实的最小代价值
            for (row = 1; row < nOfRows; row++)
            {
                temp += nOfColumns; // 指针按步长跳跃至下一行同一列
                if (*temp < minValue)
                {
                    minValue = *temp; // 更新列最小值基准
                }
            }
            temp = distMatrixTemp + col; // 游标重置回列首
            // 本列所有元素统一减去该列最小值，必然产生至少一个 0
            for (row = 0; row < nOfRows; row++)
            {
                *temp -= minValue;  // 执行偏移相减
                temp += nOfColumns; // 指针按步长跳跃至下一行
            }
        }
    }

    int step = 1;     // 初始化算法状态机的入口，设定为步骤 1
    while (step != 7) // 状态 7 为结算态，循环直到跳入 7 结束
    {
        switch (step)
        {
        case 1:
            // 步骤 1：遍历矩阵并打初始星标。如果某个零元素的同行同列均未被覆盖，则赋星并覆盖该行列
            for (row = 0; row < nOfRows; row++)
            {
                for (col = 0; col < nOfColumns; col++)
                {
                    if (distMatrixTemp[row * nOfColumns + col] == 0) // 定位到零元素
                    {
                        if (!coveredRows[row] && !coveredColumns[col]) // 检验该零的同行同列是否"自由" (未被覆盖)
                        {
                            starMatrix[row * nOfColumns + col] = true; // 给这个零打上星标
                            coveredColumns[col] = true;                // 立刻覆盖该元素所在的列
                            coveredRows[row] = true;                   // 立刻覆盖该元素所在的行
                        }
                    }
                }
            }

            // 初期赋星结束，抹除所有行列的覆盖痕迹，保持矩阵整洁进入下一步
            for (row = 0; row < nOfRows; row++)
            {
                coveredRows[row] = false; // 解除行覆盖
            }
            for (col = 0; col < nOfColumns; col++)
            {
                coveredColumns[col] = false; // 解除列覆盖
            }
            step = 2; // 状态推进至步骤 2
            break;

        case 2:
            // 步骤 2：对所有带星零所在的列进行覆盖，并判断是否已达成满匹配要求
            for (row = 0; row < nOfRows; row++)
            {
                for (col = 0; col < nOfColumns; col++)
                {
                    if (starMatrix[row * nOfColumns + col]) // 检索星标矩阵
                    {
                        coveredColumns[col] = true; // 若带有星标，覆盖此列
                    }
                }
            }
            {
                int count = 0; // 计化器表示当前找到多少条有效的星状列
                for (col = 0; col < nOfColumns; col++)
                {
                    if (coveredColumns[col])
                    {
                        count++; // 如果列被覆盖，计数累加
                    }
                }

                if (count >= minDim) // 如果覆盖的列数达到行列最小值边界 (代表所有人/物均已分配完)
                {
                    step = 7; // 跳跃到末尾步骤 7 收尾清算
                }
                else
                {
                    step = 3; // 配对未满，需要进入步骤 3 深度挖掘可行零路线
                }
            }
            break;

        case 3:
        {
            // 步骤 3：在剩余未被覆盖的区域探寻零，为其打上撇标(prime)，寻找增广路径口
            bool zerosFound = true; // 循环探索标志
            while (zerosFound)
            {
                zerosFound = false; // 初始假设本轮未发现
                for (col = 0; col < nOfColumns; col++)
                {
                    if (!coveredColumns[col]) // 仅搜索还未被覆盖的区域列
                    {
                        for (row = 0; row < nOfRows; row++)
                        {
                            if (!coveredRows[row] && distMatrixTemp[row * nOfColumns + col] == 0) // 寻找到一个未覆盖的零
                            {
                                primeMatrix[row * nOfColumns + col] = true; // 先给这个空白零打上待判定的撇标
                                int starCol = 0;                            // 预备用作该行寻找星标列的索引

                                for (starCol = 0; starCol < nOfColumns; starCol++)
                                {
                                    if (starMatrix[row * nOfColumns + starCol]) // 检索同行中是否已经有了别人占据的星状零
                                    {
                                        break; // 找到了则退出检索
                                    }
                                }
                                if (starCol == nOfColumns) // 该行遍历到头未提前 break，意味着本行是全空的（没有星标存在）
                                {
                                    step = 4;          // 这个撇号可以作为增广路径起点，跳至步骤 4 执行换位
                                    zerosFound = true; // 继续探索标记
                                    break;
                                }
                                else
                                {
                                    // 同行早存在其他星标：产生资源冲突
                                    coveredRows[row] = true;         // 将本行加入覆盖区
                                    coveredColumns[starCol] = false; // 将那颗已有星标所处的列解除覆盖，暴露出来
                                    zerosFound = true;               // 标记发现变动，重启寻找
                                    break;
                                }
                            }
                        }
                    }
                    if (step == 4) // 一旦满足步骤 4 触发条件，彻底打断最外层 while 搜索
                        break;
                }
            }

            if (step != 4) // 如果全图没有可用的活零，说明当前矩阵掩码耗尽
            {
                step = 5; // 跳转步骤 5 强制计算新零补给
            }
        }
        break;

        case 4:
        {
            // 步骤 4：构建与反转增广路径，将首尾相接的撇标合法替换为星标（扩大匹配数）
            for (int r = 0; r < nOfElements; r++)
            {
                newStarMatrix[r] = starMatrix[r]; // 备份星标现场以供操作反转
            }
            int r = 0, c = 0; // 暂存游标
            for (row = 0; row < nOfRows; row++)
            {
                for (col = 0; col < nOfColumns; col++)
                {
                    // 承接步骤 3，揪出引发本次翻转且同行无星的那颗导火索撇标
                    if (!coveredRows[row] && !coveredColumns[col] && distMatrixTemp[row * nOfColumns + col] == 0)
                    {
                        primeMatrix[row * nOfColumns + col] = true; // 确保其留有撇标痕迹
                        r = row;                                    // 记录其所在行
                        c = col;                                    // 记录其所在列
                        break;
                    }
                }
                if (c != 0 || r != 0) // 获取到坐标即闪退
                    break;
            }
            int starRow = 0;           // 追溯用的旧星标行缓存
            bool starRowFound = false; // 追溯标记
            while (true)               // 执行交替链路追溯
            {
                newStarMatrix[r * nOfColumns + c] = true; // 【重转将】：当前撇标直接拉正，升格转为星标
                starRowFound = false;                     // 设初态

                for (starRow = 0; starRow < nOfRows; starRow++)
                {
                    if (starMatrix[starRow * nOfColumns + c]) // 从同列之中，寻找可能挤占的旧星标位置
                    {
                        starRowFound = true; // 找到同列存在的老星标
                        break;
                    }
                }
                if (!starRowFound) // 如果没有找到同列老星标，链路追溯合法到头可以切断
                {
                    break;
                }
                newStarMatrix[starRow * nOfColumns + c] = false; // 卸位：那个挡路的同列老星标被抹去身份
                r = starRow;                                     // 前往旧星标所在行
                for (col = 0; col < nOfColumns; col++)
                {
                    if (primeMatrix[r * nOfColumns + col]) // 寻找抹掉旧星时它同行的替补（也是个撇状标记点）
                    {
                        c = col; // 重置列向坐标
                        break;
                    }
                }
            }

            // 将处理后的新链路网同步回主变量区
            for (int r = 0; r < nOfElements; r++)
            {
                primeMatrix[r] = false;           // 链路反转完成，清除所有撇标历史
                starMatrix[r] = newStarMatrix[r]; // 更新新制定的星相布局
            }
            for (row = 0; row < nOfRows; row++)
            {
                coveredRows[row] = false; // 重置全局盖行记忆
            }
            for (col = 0; col < nOfColumns; col++)
            {
                coveredColumns[col] = false; // 重置全局盖列记忆
            }

            step = 2; // 返场回步骤 2，依靠更新后的布星情况验算分配计数
        }
        break;

        case 5:
        {
            // 步骤 5：矩阵补偿。当所有掩模列均找不到0且匹配不满时，需全局提取未覆盖的最小正代价制造新零
            double minUncoveredValue = std::numeric_limits<double>::max(); // 将变量推到物理极高值准备求底
            for (row = 0; row < nOfRows; row++)
            {
                if (!coveredRows[row]) // 只关注暴露出来的行
                {
                    for (col = 0; col < nOfColumns; col++)
                    {
                        if (!coveredColumns[col]) // 只关注暴露出来的列
                        {
                            if (distMatrixTemp[row * nOfColumns + col] < minUncoveredValue) // 发掘最小曝光元
                            {
                                minUncoveredValue = distMatrixTemp[row * nOfColumns + col]; // 截获最细微的误差基准
                            }
                        }
                    }
                }
            }

            if (minUncoveredValue > 0) // 如果该下界存在且合理
            {
                // 施加全局十字补偿算子
                for (row = 0; row < nOfRows; row++)
                {
                    for (col = 0; col < nOfColumns; col++)
                    {
                        if (coveredRows[row])
                        {
                            distMatrixTemp[row * nOfColumns + col] += minUncoveredValue; // 被划线的行，数值抬升
                        }
                        if (!coveredColumns[col])
                        {
                            distMatrixTemp[row * nOfColumns + col] -= minUncoveredValue; // 未划线的列，数值下压，从而催生前所未有的"0"点
                        }
                    }
                }
            }

            step = 3; // 矩阵中涌现了新的零区域，回跳步骤 3 发挥其势能
        }
        break;
        }
    }

    // 步骤 7：算法落幕清算。核验并抓取所有稳居位点的星标转换成实质的配对表
    for (row = 0; row < nOfRows; row++)
    {
        for (col = 0; col < nOfColumns; col++)
        {
            if (starMatrix[row * nOfColumns + col]) // 判断这里是否含有最终确认不转移的星标
            {
                assignment[row] = col;                       // 将该检测框（col）发配给该行追踪器（row）
                *cost += distMatrix[row * nOfColumns + col]; // 向全局代价值池贡献此次派单的初始原价
            }
        }
    }

    // 用完即排免内存泄露，清空所有 C 式堆申请
    free(distMatrixTemp); // 回收算子缓冲矩阵
    free(coveredColumns); // 回收列覆盖清单
    free(coveredRows);    // 回收行覆盖清单
    free(starMatrix);     // 回收星图掩码图层
    free(primeMatrix);    // 回收撇图待定图层
    free(newStarMatrix);  // 回收新星图备用翻转层
}
