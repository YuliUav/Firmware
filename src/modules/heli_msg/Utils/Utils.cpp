#if (defined _WIN32) || (defined __linux__) || (defined __QNXNTO__)

#include <iostream>
#include <cstring>

#ifdef __QNXNTO__
#include <sys/neutrino.h>
#endif

#include "Utils.h"

using namespace std;

void CommonTest();
void ProtocolTest();
void SharedMemoryTest();
void* SerialPortRead(void*);
void* SerialPortWrite(void*);
void* UdpReceive(void*);
void* UdpSend(void*);
void TimerTest();

int main()
{
	cout << "Welcome to the Utils Process" << endl;

	/// Common function test
	cout << endl << "1 Common function test" << endl;
	CommonTest();

	/// Standard packet protocol test
	cout << endl << "2 Standard packet protocol test" << endl;
	ProtocolTest();

	/// Shared memory test (linux and qnx)
	cout << endl << "3 Shared memory test (linux and qnx)" << endl;
	SharedMemoryTest();

	cout << endl
		<< "4 Serial port communicating (linux and qnx), udp client communicating (linux and qnx) and timer (qnx) test"
		<< endl;

#if (defined __linux__) || (defined __QNXNTO__)
	/// Serial port communicating test
	pthread_t serialPortRead;
	pthread_t serialPortWrite;
	pthread_create(&serialPortRead, NULL, &SerialPortRead, NULL);
	pthread_create(&serialPortWrite, NULL, &SerialPortWrite, NULL);

	/// Udp client communicating test
	pthread_t udpReceive;
	pthread_t udpSend;
	pthread_create(&udpReceive, NULL, &UdpReceive, NULL);
	pthread_create(&udpSend, NULL, &UdpSend, NULL);
#endif

	/// Timer test
	TimerTest();

	cin.get();
	return EXIT_SUCCESS;
}

void CommonTest()
{
	Common common;
	unsigned char memory[16] =
	{ 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15 };
	cout << "Memory(0-15): ";
	common.MemoryPrint(memory, 16);
	cout << endl << "Binary(255): ";
	common.BinaryPrint(255);
	cout << endl << "Timestamp: " << fixed << common.GetTimestamp() << endl;
	char* strTime = common.GetTimeString();
	cout << "StrTime: " << strTime << endl;
	delete[] strTime; // Ensure that you free the array when you done with it or you will leak memory
	common.StatusPrint("Status: status printing test", __FILE__, __FUNCTION__,
		__LINE__);
}

void ProtocolTest()
{
	Common common;
	Protocol protocol;
	/// 1 Binary protocol
	/// 1.1 Packet set
	unsigned char binaryData[18] =
	{ 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18 };
	BinaryPacket binaryPacket =
	{ binaryData, 1, 18 };
	/// 1.2 Encode packet
	BinaryBuffer binaryBuffer = protocol.BinaryPacketEncode(binaryPacket);
	cout << "Binary buffer:" << endl << "data = ";
	common.MemoryPrint(binaryBuffer.data, binaryBuffer.length);
	cout << endl << "length = " << binaryBuffer.length << ", errors = "
		<< binaryBuffer.errors << endl;
	/// 1.3 Decode packet
	memcpy(binaryBuffer.data + binaryBuffer.length, binaryBuffer.data, binaryBuffer.length);
	binaryPacket = protocol.BinaryBufferDecode(&binaryBuffer);
	cout << "Binary packet:" << endl << "data = ";
	common.MemoryPrint(binaryPacket.data, binaryPacket.length);
	cout << endl << "id = " << binaryPacket.id << ", length = "
		<< binaryPacket.length << ", errors: " << binaryBuffer.errors
		<< endl;
	delete[] binaryPacket.data; // Ensure that you free the packet when you done with it or you will leak memory
	/// 2 Ascii protocol
	/// 2.1 Packet set
	char
		*asciiData =
		(char*) "GPGSV,3,1,11,10,63,137,17,07,61,098,15,05,59,290,20,08,54,157,30";
	AsciiPacket asciiPacket =
	{ (unsigned char*)asciiData, 64 };
	/// 2.2 Encode packet
	AsciiBuffer asciiBuffer = protocol.AsciiPacketEncode(asciiPacket);
	asciiBuffer.data[asciiBuffer.length] = '\0';
	cout << "Ascii buffer:" << endl << "data = " << asciiBuffer.data
		<< "length = " << asciiBuffer.length << ", errors = "
		<< asciiBuffer.errors << endl;
	/// 1.3 Decode packet
	asciiPacket = protocol.AsciiBufferDecode(&asciiBuffer);
	asciiPacket.data[asciiPacket.length] = '\0';
	cout << "Ascii packet:" << endl << "data = " << asciiPacket.data << endl
		<< "length = " << asciiPacket.length << ", errors = "
		<< asciiBuffer.errors << endl;
	delete[] asciiPacket.data; // Ensure that you free the packet when you done with it or you will leak memory
}

void SharedMemoryTest()
{
#if (defined __linux__) || (defined __QNXNTO__)
	Common common;
	SharedMemory sharedMemory;
#ifdef __linux__
	const char* shmName = "ShmX";
#else
	const char* shmName = "/dev/shmem/ShmX";
#endif
	ShmX* shmXPtr;
	if (sharedMemory.Initialize(shmName, &shmXPtr) == 1)
	{
		common.StatusPrint("Error: unable to create shared memory", __FILE__,
			__FUNCTION__, __LINE__);
		return;
	}
	pthread_rwlock_wrlock(&(shmXPtr->rwlock));
	shmXPtr->timestamp = common.GetTimestamp();
	shmXPtr->parameter = 16;
	pthread_rwlock_unlock(&(shmXPtr->rwlock));

	pthread_rwlock_rdlock(&(shmXPtr->rwlock));
	cout << shmXPtr->timestamp << endl;
	cout << shmXPtr->parameter << endl;
	pthread_rwlock_unlock(&(shmXPtr->rwlock));
#endif
}

void* SerialPortRead(void*)
{
#if (defined __linux__) || (defined __QNXNTO__)
	Common common;
	common.StatusPrint("in serial port read",  __FILE__, __FUNCTION__, __LINE__);
	/// 1 Open serial port
#ifdef __linux__
	const char* portName = "/dev/ttyUSB0"; // Usb1
#else
	const char* portName = "COM1"; //  Com1
#endif
	SerialPort serialPort(portName, 115200, 'N', 8, 1);
	switch (serialPort.Open())
	{
	case 1:
		common.StatusPrint("Error: unable to open serial port", __FILE__,
			__FUNCTION__, __LINE__);
		return NULL;
	case 2:
		common.StatusPrint("Error: invalid baud rate", __FILE__, __FUNCTION__,
			__LINE__);
		return NULL;
	case 3:
		common.StatusPrint("Error: invalid parity", __FILE__, __FUNCTION__,
			__LINE__);
		return NULL;
	case 4:
		common.StatusPrint("Error: invalid number of data bits", __FILE__,
			__FUNCTION__, __LINE__);
		return NULL;
	case 5:
		common.StatusPrint("Error: invalid number of stop bits", __FILE__,
			__FUNCTION__, __LINE__);
		return NULL;
	}
	/// 2 Read
	unsigned char bufferRead[1024];
	int bytesRead;
	serialPort.DiscardInBuffer();
	while (1)
	{
		bytesRead = serialPort.Read(bufferRead, 0, 1023);
		bufferRead[bytesRead] = '\0';
		cout << bufferRead;
		usleep(1000000); // Suspend a thread for 1s
	}
	/// 3 Close serial port
	serialPort.Close();
#endif
	return NULL;
}

void* SerialPortWrite(void*)
{
#if (defined __linux__) || (defined __QNXNTO__)
	Common common;
	/// 1 Open serial port
#ifdef __linux__
	const char* portName = "/dev/ttyUSB0"; // Usb1
#else
	const char* portName = "COM1"; ///dev/ser1 Com1
#endif
	SerialPort serialPort(portName, 115200, 'N', 8, 1);
	switch (serialPort.Open())
	{
	case 1:
		common.StatusPrint("Error: unable to open serial port", __FILE__,
			__FUNCTION__, __LINE__);
		return NULL;
	case 2:
		common.StatusPrint("Error: invalid baud rate", __FILE__, __FUNCTION__,
			__LINE__);
		return NULL;
	case 3:
		common.StatusPrint("Error: invalid parity", __FILE__, __FUNCTION__,
			__LINE__);
		return NULL;
	case 4:
		common.StatusPrint("Error: invalid number of data bits", __FILE__,
			__FUNCTION__, __LINE__);
		return NULL;
	case 5:
		common.StatusPrint("Error: invalid number of stop bits", __FILE__,
			__FUNCTION__, __LINE__);
		return NULL;
	}
	/// 2 Write
	char bufferWrite[40];
	serialPort.DiscardOutBuffer();
	while (1)
	{
		sprintf(bufferWrite, "[%lf]: %s\n", common.GetTimestamp(),
			"Serial port write");
		serialPort.Write((unsigned char*)bufferWrite, 0, 39);
		usleep(1000000); // Suspend a thread for 1s
	}
	/// 3 Close serial port
	serialPort.Close();
#endif
	return NULL;
}

void* UdpReceive(void*)
{
#if (defined __linux__) || (defined __QNXNTO__)
	Common common;
	/// 1 Initialize udp client by specifying server address
	UdpClient udpClient("255.255.255.255", 8001);
	/// 2 Bind localhost to udp client
	if (udpClient.Bind("0.0.0.0", 8001) == 1)
	{
		common.StatusPrint(
			"Error: unable to bind the specified ip and port to udp client",
			__FILE__, __FUNCTION__, __LINE__);
		return NULL;
	}
	/// 3 Receive
	unsigned char dgramReceive[1024];
	int bytesReceive;
	while (1)
	{
		bytesReceive = udpClient.Receive(dgramReceive, 0, 1023);
		dgramReceive[bytesReceive] = '\0';
		cout << dgramReceive;
	}
	/// 4 Close udp client
	udpClient.Close();
#endif
	return NULL;
}

void* UdpSend(void*)
{
#if (defined __linux__) || (defined __QNXNTO__)
	Common common;
	/// 1 Initialize udp client by specifying server address
	UdpClient udpClient("255.255.255.255", 8002);
	/// 2 Set broadcast
	udpClient.SetBroadcast();
	/// 3 Send
	char bufferSend[38];
	while (1)
	{
		sprintf(bufferSend, "[%lf]: %s\n", common.GetTimestamp(),
			"Udp client send");
		udpClient.Send((unsigned char*)bufferSend, 0, 37);
		usleep(1000000); // Suspend a thread for 1s
	}
	/// 4 Close udp client
	udpClient.Close();
#endif
	return NULL;
}

void TimerTest()
{
#ifdef __QNXNTO__
	Timer timer(1000);
	timer.Start();
	TimerMessage msg;
	while (1)
		if (MsgReceive(timer.chid, &msg, sizeof(TimerMessage), NULL) == 0
			&& msg.pulse.code == PulseFromTimer)
			cout << "We got a pulse from our timer" << endl;
	timer.Close();
#endif
}

#endif
