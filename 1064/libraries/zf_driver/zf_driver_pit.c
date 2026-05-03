/*********************************************************************************************************************
* RT1064DVL6A Opensourec Library ����RT1064DVL6A ��Դ�⣩��һ�����ڹٷ� SDK �ӿڵĵ�������Դ��
* Copyright (c) 2022 SEEKFREE ��ɿƼ�
* 
* ���ļ��� RT1064DVL6A ��Դ���һ����
* 
* RT1064DVL6A ��Դ�� ���������
* �����Ը���������������ᷢ���� GPL��GNU General Public License���� GNUͨ�ù�������֤��������
* �� GPL �ĵ�3�棨�� GPL3.0������ѡ��ģ��κκ����İ汾�����·�����/���޸���
* 
* ����Դ��ķ�����ϣ�����ܷ������ã�����δ�������κεı�֤
* ����û�������������Ի��ʺ��ض���;�ı�֤
* ����ϸ����μ� GPL
* 
* ��Ӧ�����յ�����Դ���ͬʱ�յ�һ�� GPL �ĸ���
* ���û�У������<https://www.gnu.org/licenses/>
* 
* ����ע����
* ����Դ��ʹ�� GPL3.0 ��Դ����֤Э�� ������������Ϊ���İ汾
* ��������Ӣ�İ��� libraries/doc �ļ����µ� GPL3_permission_statement.txt �ļ���
* ����֤������ libraries �ļ����� �����ļ����µ� LICENSE �ļ�
* ��ӭ��λʹ�ò����������� ���޸�����ʱ���뱣����ɿƼ��İ�Ȩ����������������
* 
* �ļ�����          zf_driver_pit
* ��˾����          �ɶ���ɿƼ����޹�˾
* �汾��Ϣ          �鿴 libraries/doc �ļ����� version �ļ� �汾˵��
* ��������          IAR 8.32.4 or MDK 5.33
* ����ƽ̨          RT1064DVL6A
* ��������          https://seekfree.taobao.com/
* 
* �޸ļ�¼
* ����              ����                ��ע
* 2022-09-21        SeekFree            first version
********************************************************************************************************************/

#include "zf_common_clock.h"
#include "zf_common_debug.h"
#include "zf_common_interrupt.h"


#include "zf_driver_pit.h"

//-------------------------------------------------------------------------------------------------------------------
// �������     PIT �ж�ʹ��
// ����˵��     pit_chn             PIT ����ģ���
// ���ز���     void
// ʹ��ʾ��     pit_enable(PIT_CH0);
// ��ע��Ϣ     
//-------------------------------------------------------------------------------------------------------------------
void pit_enable (pit_index_enum pit_chn)
{
    PIT_StartTimer(PIT, (pit_chnl_t)pit_chn);
}

//-------------------------------------------------------------------------------------------------------------------
// �������     PIT �жϽ�ֹ
// ����˵��     pit_chn             PIT ����ģ���
// ���ز���     void
// ʹ��ʾ��     pit_disable(PIT_CH0);
// ��ע��Ϣ     
//-------------------------------------------------------------------------------------------------------------------
void pit_disable (pit_index_enum pit_chn)
{
    PIT_StopTimer(PIT, (pit_chnl_t)pit_chn);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     PIT定时器初始化，是pit_ms_init和pit_us_init的底层实现
// 参数说明     pit_chn             要初始化的PIT通道（PIT_CH0~PIT_CH3）
// 参数说明     period              PIT计数周期（以时钟周期为单位）
// 返回参数     void
// 使用示例     pit_init(PIT_CH0, 120);
// 备注信息     初始化PIT定时器，配置计数周期并启动定时器
//-------------------------------------------------------------------------------------------------------------------
void pit_init (pit_index_enum pit_chn, uint32 period)
{
    static uint8 init_flag;
    
    if(0 == init_flag)
    {
        init_flag = 1;
        pit_config_t pitConfig;
    
        PIT_GetDefaultConfig(&pitConfig);   //Ĭ������Ϊfalse
                  
        PIT_Init(PIT, &pitConfig);          //��һ�γ�ʼ�����ڴ�ʱ��
        PIT_Deinit(PIT);                    //��λ����
        PIT_Init(PIT, &pitConfig);          //���³�ʼ��������ȷ�Ĳ���
    }
    
    
    PIT_SetTimerPeriod(PIT, (pit_chnl_t)pit_chn, period);
    PIT_EnableInterrupts(PIT, (pit_chnl_t)pit_chn, kPIT_TimerInterruptEnable);//��PITͨ��0�ж�
	PIT_SetTimerChainMode(PIT, (pit_chnl_t)pit_chn, false);
    PIT_StartTimer(PIT, (pit_chnl_t)pit_chn);
    EnableIRQ(PIT_IRQn);
}
