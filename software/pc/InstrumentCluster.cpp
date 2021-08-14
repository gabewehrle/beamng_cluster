// InstrumentCluster.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <iostream>
#include <type_traits>
#include <bitset>
#include <time.h>
#include <WS2tcpip.h>
#include "Network.h"
#include "SerialPort.hpp"
#include "xxhash.h"

using namespace std;

struct dataPacket {
	unsigned int* time;			// time in milliseconds (to check order)
	char* car;					// Car name (4 bytes)
	unsigned short* flags;		// Info (see OG_x below)
	unsigned char* gear;		// Reverse:0, Neutral:1, First:2...
	unsigned char* plid;		// Unique ID of viewed player (0 = none)
	float* speed;				// M/S
	float* rpm;					// RPM
	float* turbo;				// BAR
	float* engtemp;				// C
	float* fuel;				// 0 to 1
	float* oilpressure;			// BAR
	float* oiltemp;				// C
	unsigned int* dashlights;	// Dash lights available (see DL_x below)
	unsigned int* showlights;	// Dash lights currently switched on
	float* throttle;			// 0 to 1
	float* brake;				// 0 to 1
	float* clutch;				// 0 to 1
	char* display1;				// Usually Fuel (16)
	char* display2;				// Usually Settings (16)

	XXH32_hash_t hash;
	char delimeter = '\n';
};

template <typename T>
void bytesToType(const unsigned char* bytes, int start, int length, T& data);

void subArray(const unsigned char* bytes, int start, int length, char* data);

void GoToXY(int column, int line);

void printDataPacket(dataPacket& packet, int y);

void populateDataPacket(unsigned char* data, dataPacket& packet);

void calcPacketHash(unsigned char* data, dataPacket& packet);

long getTime();

const char* portName = "\\\\.\\COM4";

SerialPort* arduino;

int main() {
	int PORT_RECV = 4444;

	dataPacket clusterData;

	//86, 154, 91, 66
	//byte speedSend[5] = { 86, 154, 91, 66, '\n' };

	const int DELAY_TIME = 100;
	int lastTime = 0;


	// OG_x - bits for OutGaugePack Flags
	// OG_SHIFT      1         key
	// OG_CTRL       2         key
	// OG_TURBO      8192      show turbo gauge
	// OG_KM         16384     if not set - user prefers MILES
	// OG_BAR        32768     if not set - user prefers PSI

	// DL_x - bits for OutGaugePack DashLights and ShowLights
	//DL_SHIFT,           // bit 0    - shift light
	//DL_FULLBEAM,        // bit 1    - full beam
	//DL_HANDBRAKE,       // bit 2    - handbrake
	//DL_PITSPEED,        // bit 3    - pit speed limiter
	//DL_TC,              // bit 4    - TC active or switched off
	//DL_SIGNAL_L,        // bit 5    - left turn signal
	//DL_SIGNAL_R,        // bit 6    - right turn signal
	//DL_SIGNAL_ANY,      // bit 7    - shared turn signal
	//DL_OILWARN,         // bit 8    - oil pressure warning
	//DL_BATTERY,         // bit 9    - battery warning
	//DL_ABS,             // bit 10   - ABS active or switched off
	//DL_ESC,			  // bit 11	- ESC
	//DL_HEADLIGHT,		  // bit 12	- Headlights on or off
	//DL_CHECK_ENGINE,	  // bit 13	- Check Engine
	//DL_ESC_OFF,		  // bit 14	- ESC turned off
	//DL_FOGLIGHTS		  // bit 15	- Fog lights on or off
	//DL_CRUISE_MAIN	  // bit 16	- Cruise main
	//DL_CRUISE_SET		  // bit 17	- Cruise set
	//DL_DOOR_OPEN		  // bit 18	- Door open
	//DL_TRUNK_OPEN		  // bit 19	- Trunk open
	//DL_TPMS			  // bit 20 - TPMS
	//DL_TREAD			  // bit 21 - Low tire pressure
	//ENG_ON			  // bit 22 - Is the engine on?
	//STARTER_ON		  // bit 23 - Is the starter running?
	//IGN_ON			  // bit 24 - Is the ignition on?

	try
	{
		WSASession Session;
		UDPSocket SocketRecv;
		static char bufferRecv[96]; //UDP packet should be 92 bytes, 4 more just in case
		populateDataPacket(reinterpret_cast<unsigned char*>(bufferRecv), clusterData);

		SocketRecv.Bind(PORT_RECV);

		arduino = new SerialPort(portName);

		while (true) {
			sockaddr_in addressIn = SocketRecv.RecvFrom(bufferRecv, sizeof(bufferRecv));

			printDataPacket(clusterData, 0);

			if (getTime() - lastTime >= DELAY_TIME) {
				lastTime = getTime();
				bool hasWritten;
				//char receivedString[50];
				if (arduino->isConnected()) {
					calcPacketHash(reinterpret_cast<unsigned char*>(bufferRecv), clusterData);
					hasWritten = arduino->writeSerialPort(reinterpret_cast<byte*>(bufferRecv), 92); //Send data to Arduino
					hasWritten &= arduino->writeSerialPort(reinterpret_cast<byte*>(&clusterData.hash), 4); //Send checksum
					//hasWritten &= arduino->writeSerialPort(reinterpret_cast<byte*>(&clusterData.delimeter), 1); //Send packet delimeter

					//arduino->readSerialPort(receivedString, 50);
					cout << "Status: " << hasWritten << '\n';
					//cout << "GOT: " << receivedString << "                                \n";
				}
				else {
					cout << "Status: DISCONNECTED\n";
				}

				char str[INET_ADDRSTRLEN];
				cout << "Sending to: " << portName;
				inet_ntop(AF_INET, &(addressIn.sin_addr), str, INET_ADDRSTRLEN);
				cout << " (" << DELAY_TIME << " ms)\nReceiving from: " << str << '\n';
			}
		}
	}
	catch (std::system_error & e)
	{
		std::cout << e.what();
	}
}

template <typename T>
void bytesToType(const unsigned char* bytes, int start, int length, T& data) {
	memcpy(&data, &bytes[start], length);
}

void subArray(const unsigned char* bytes, int start, int length, char* data) {
	for (int i = 0; i < length; ++i) {
		data[i] = bytes[i + start];
	}
}

void GoToXY(int column, int line) {
	// Create a COORD structure and fill in its members.
	// This specifies the new position of the cursor that we will set.
	COORD coord;
	coord.X = column;
	coord.Y = line;

	// Obtain a handle to the console screen buffer.
	// (You're just using the standard console, so you can use STD_OUTPUT_HANDLE
	// in conjunction with the GetStdHandle() to retrieve the handle.)
	// Note that because it is a standard handle, we don't need to close it.
	HANDLE hConsole = GetStdHandle(STD_OUTPUT_HANDLE);

	// Finally, call the SetConsoleCursorPosition function.
	if (!SetConsoleCursorPosition(hConsole, coord)) {
		// Uh-oh! The function call failed, so you need to handle the error.
		// You can call GetLastError() to get a more specific error code.
		// ...
	}
}

void populateDataPacket(unsigned char* data, dataPacket& packet) {
	//Point packet.XXX at its location in "data". Don't actually copy bytes into a separate pointer
	packet.time = reinterpret_cast<unsigned int*>(data);
	packet.car = reinterpret_cast<char*>(data + 4);
	packet.flags = reinterpret_cast<unsigned short*>(data + 8);
	packet.gear = reinterpret_cast<unsigned char*>(data + 10);
	packet.speed = reinterpret_cast<float*>(data + 12);
	packet.rpm = reinterpret_cast<float*>(data + 16);
	packet.turbo = reinterpret_cast<float*>(data + 20);
	packet.engtemp = reinterpret_cast<float*>(data + 24);
	packet.fuel = reinterpret_cast<float*>(data + 28);
	packet.oilpressure = reinterpret_cast<float*>(data + 32);
	packet.oiltemp = reinterpret_cast<float*>(data + 36);
	packet.dashlights = reinterpret_cast<unsigned int*>(data + 40);
	packet.showlights = reinterpret_cast<unsigned int*>(data + 44);
	packet.throttle = reinterpret_cast<float*>(data + 48);
	packet.brake = reinterpret_cast<float*>(data + 52);
	packet.clutch = reinterpret_cast<float*>(data + 56);
	packet.display1 = reinterpret_cast<char*>(data + 60);
	packet.display2 = reinterpret_cast<char*>(data + 76);
}

void calcPacketHash(unsigned char* data, dataPacket& packet) {
	packet.hash = XXH32(data, 92, 0xebac6cdb); //hash is a uint32. Use a random seed (same for client)
}

void printDataPacket(dataPacket& packet, int y) {
	GoToXY(0, y); //We get 25 lines by default in a window
	cout << "Time: " << *packet.time << "                "
		<< "\nSpeed: " << *packet.speed << "                "
		<< "\nGear: " << +*packet.gear << "                "
		<< "\nRPM: " << *packet.rpm << "                "
		<< "\nEngine temp: " << *packet.engtemp << "                "
		<< "\nFuel: " << *packet.fuel << "                "
		<< "\nOil Pressure: " << *packet.oilpressure << "                "
		<< "\nOil temp: " << *packet.oiltemp << "                "
		<< "\nShow lights: " << bitset<32>(*packet.showlights) << "                "
		<< "\nDisplay 1: ";
	for (int i = 0; i < 16; ++i) {
		cout << packet.display1[i];
	}
	cout << "\nDisplay 2: ";
	for (int i = 0; i < 16; ++i) {
		cout << packet.display2[i];
	}
	cout << "\n\n";

	int rpm = static_cast<int>(*packet.rpm);

	byte rpmL = (rpm & 0xFF);
	byte rpmH = ((rpm & ~0xFF) >> 8);
	cout << "rpmL: '" << hex << +rpmL << "', rpmH: '" << hex << +rpmH << "'     \n";
}

long getTime() {
	return clock() / (CLOCKS_PER_SEC / 1000);
}