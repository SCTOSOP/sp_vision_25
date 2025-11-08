#include "wit_imu.hpp"
#include "wit_c_sdk.h"
#include "serial.h"
#include <chrono>
#include <thread>

namespace io {

#define Quaternion_UPDATE 0x01
volatile char s_cDataUpdate = 0;
int fd, s_iCurBaud = 9600;
const int c_uiBaud[] = {2400 , 4800 , 9600 , 19200 , 38400 , 57600 , 115200 , 230400 , 460800 , 921600};

static void SensorDataUpdata(uint32_t uiReg, uint32_t uiRegNum)
{
    int i;
    for(i = 0; i < uiRegNum; i++)
    {
        switch(uiReg)
        {          
			case q0:{
				s_cDataUpdate |= Quaternion_UPDATE;
				}
        }
		uiReg++;
    }
}

void AutoScanSensor(unsigned char * dev)
{
	int i, iRetry;
	unsigned char  cBuff[1];
	
	for(i = 1; i < 10; i++)
	{
		serial_close(fd);
		s_iCurBaud = c_uiBaud[i];
		fd = serial_open(dev , c_uiBaud[i]);
		
		iRetry = 2;
		do
		{
			s_cDataUpdate = 0;
			WitReadReg(AX, 3);
			std::this_thread::sleep_for(std::chrono::milliseconds(200));
			while(serial_read_data(fd, cBuff, 1))
			{
				WitSerialDataIn(cBuff[0]);
			}
			if(s_cDataUpdate != 0)
			{
				printf("%d baud find sensor\r\n\r\n", c_uiBaud[i]);
				return ;
			}
			iRetry--;
		}while(iRetry);		
	}
	printf("can not find sensor\r\n");
	printf("please check your connection\r\n");
    std::exit(-1);
}

WIT_IMU::WIT_IMU(std::string dev_name_, int rate_) : dev_name(std::move(dev_name_)), rate(rate_) {
    WitInit(WIT_PROTOCOL_NORMAL, 0x50);
	WitRegisterCallBack(SensorDataUpdata);
    AutoScanSensor(reinterpret_cast<unsigned char*>(const_cast<char*>(dev_name.c_str())));
}

WIT_IMU::~WIT_IMU() {
    
}

Eigen::Quaterniond WIT_IMU::get_data() {
	unsigned char cBuff[1];
	float imu_data[4] = {0};
	while(serial_read_data(fd, cBuff, 1))
    {
        WitSerialDataIn(cBuff[0]);
    }

	for ( int i = 0; i < 4; i++)
    {
    	imu_data[i] = sReg[q0+i] / 32768.0f;
    }

	return Eigen::Quaterniond(imu_data[0], imu_data[1], imu_data[2], imu_data[3]);
}

}