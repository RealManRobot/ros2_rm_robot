#ifndef RM_QUEUE_H
#define RM_QUEUE_H
//////////////////////////////////////////////////////////////////////////////////
//睿尔曼智能科技有限公司        Author:Dong Qinpeng
//创建日期:2022/08/23
//版本：V4.0
//版权所有，盗版必究。
//Copyright(C) 睿尔曼智能科技有限公司
//All rights reserved
//文档说明：该文档定义了Socket通讯队列数据信息
//////////////////////////////////////////////////////////////////////////////////
#ifdef __cplusplus
extern "C" {
#endif
//Socket发送队列数据结构体
typedef struct
{
    int len;                     //该次发送字节数量
    char data[300];              //发送数据
    int sockHandle;
    int questMode;
    int flg;
    int isblock;
}Socket_Tx_Data;

typedef struct rm_queue
{
    Socket_Tx_Data *pBase;
    int front;
    int rear;
    int maxsize;
} RM_QUEUE;

RM_QUEUE Socket_Tx_Buffer;
RM_QUEUE Socket_Tx_Buffer_M;
RM_QUEUE Socket_Tx_Buffer_NonBlock;
RM_QUEUE Socket_Tx_Buffer_Block;
///
/// \brief RM_Queue_Create      创建队列
/// \param Q                    队列指针
/// \param maxsize              队列最大容量
/// \return                     成功:1, 失败0
///
int RM_Queue_Create(RM_QUEUE * Q, int maxsize);
///
/// \brief RM_Queue_Deleate      创建队列
/// \param Q                    队列指针
/// \param maxsize              队列最大容量
/// \return                     成功:1, 失败0
///
void RM_Queue_Deleate(RM_QUEUE * Q);

///
/// \brief RM_Queue_Full    判断队列是否已经存满
/// \param Q                队列指针
/// \return                 已满:1, 未满0
///
int RM_Queue_Full(RM_QUEUE * Q);

///
/// \brief RM_Queue_Empty   判断队列是否为空
/// \param Q                队列指针
/// \return                 空:1, 非空:0
///
int RM_Queue_Empty(RM_QUEUE * Q);

///
/// \brief RM_Enqueu        添加数据
/// \param Q                队列指针
/// \param val              数据
/// \return                 成功:1, 失败:0
///
int RM_Enqueu(RM_QUEUE *Q, Socket_Tx_Data *val);

///
/// \brief RM_Dequeue        移除数据
/// \param Q                队列指针
/// \param val              数据
/// \return                 成功:1, 失败:0
///
int RM_Dequeue(RM_QUEUE *Q, Socket_Tx_Data *val);

///
/// \brief RM_Queue_Clear    清空队列
/// \param Q                队列指针
/// \return                 已满:1, 未满0
///
void RM_Queue_Clear(RM_QUEUE * Q);

#ifdef __cplusplus
}
#endif
#endif // RM_QUEUE_H
