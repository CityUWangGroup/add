#include <string>
#include <ctype.h>
#include <float.h>
#include <math.h>
class transform_imu
{
private:
    char *stcTime;
    char *stcAcc;
    char *stcGyro;
    char *stcAngle;

public:
    struct Acc
    {
        double x;
        double y;
        double z;
    }acc{0,0,0};
    struct Gyro
    {
        double x;
        double y;
        double z;
    }gyro{0,0,0};
    struct Angle
    {
        double r;
        double p;
        double y;
    }angle{0,0,0};
    
/*public:
    transform_imu(double,double,double);
    //构造函数,也可尝试用初始化列表方式*/

    void FetchData(auto &data, int usLength)
{   
    int index = 0;
    //char *pData_head = data;
    //printf("count%x\n",*pData_head); 
    printf("%x\n",data[index]);
    printf("count%d\n",usLength); 
    while (usLength >= 11)//一个完整数据帧11字节
    {
        printf("%x\n",data[index + 1]);        
        if (data[index] != 0x55)//0x55是协议头
        {
            index++;//指针(/索引)后移，继续找协议头
            usLength--;
            continue;
        }
        //for(int i = 0;i < 1000;i++){printf("once\n");}
        if(data[index + 1] == 0x50) //time
        {
            stcTime = &data[index + 2];
            //memcpy(&stcTime, &data[index + 2], 8);
        }    
        else if(data[index + 1] == 0x51) //加速度
        {
            stcAcc = &data[index + 2];
            //memcpy(&stcAcc, &pData_head[index + 2], 8);
            acc.x = ((short)((short)stcAcc[1]<<8 | stcAcc[0])) / 32768.00 * 16 * 9.8;
            acc.y = ((short)((short)stcAcc[3]<<8 | stcAcc[2])) / 32768.00 * 16 * 9.8;
            acc.z = ((short)((short)stcAcc[5]<<8 | stcAcc[4])) / 32768.00 * 16 * 9.8;
        }
        else if(data[index + 1] == 0x52)
        {
            stcGyro = &data[index + 2];
            //memcpy(&stcGyro, &pData_head[index + 2], 8);
            gyro.x = ((short)((short)stcGyro[1]<<8 | stcGyro[0])) / 32768.00 * 2000 / 180 * M_PI;//弧度制
            gyro.y = ((short)((short)stcGyro[3]<<8 | stcGyro[2])) / 32768.00 * 2000 / 180 * M_PI;
            gyro.z = ((short)((short)stcGyro[5]<<8 | stcGyro[4])) / 32768.00 * 2000 / 180 * M_PI;
        }   
        else if(data[index + 1] == 0x53)
        {
            stcAngle = &data[index + 2];
            //memcpy(&stcAngle, &pData_head[index + 2], 8);
            angle.r = ((short)((short)stcAngle[1]<<8 | stcGyro[0])) / 32768.00 * M_PI;
            angle.p = ((short)((short)stcAngle[3]<<8 | stcGyro[2])) / 32768.00 * M_PI;
            angle.y = ((short)((short)stcAngle[5]<<8 | stcGyro[4])) / 32768.00 * M_PI; 
        }
 
      /*case 0x54: //磁力计
            memcpy(&stcMag, &pData_head[2], 8);
            mag.x = stcMag[0];
            mag.y = stcMag[1];
            mag.z = stcMag[2];*/
        //这里我一开始用switch case的写法    
      /*case 0x59: //四元数
            memcpy(&stcQuat, &pData_head[2], 8);
            quat.w = stcQuat[0] / 32768.00;
            quat.x = stcQuat[1] / 32768.00;
            quat.y = stcQuat[2] / 32768.00;
            quat.z = stcQuat[3] / 32768.00;*/
            
            //这个型号imu传感器6轴，暂时不启用这些读取
            //default:printf("over\n");
        
        printf("over\n");
        usLength = usLength - 11;
        index += 11;
    
    }
}

};

