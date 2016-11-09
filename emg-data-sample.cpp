// Copyright (C) 2013-2014 Thalmic Labs Inc.
// Distributed under the Myo SDK license agreement. See LICENSE.txt for details.

// This sample illustrates how to use EMG data. EMG streaming is only supported for one Myo at a time.
#define _USE_MATH_DEFINES
#include <cmath>
#include <array>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <winsock.h>
#include <myo/myo.hpp>
#define SIZE 512


#define SERVER "127.0.0.1"  //ip address of udp server 155.198.133.113// 155.198.97.107
#define BUFLEN 512  //Max length of buffer
#define SERVERPORT 8888   //serverport(simulink) The port on which to listen for incoming data

struct sockaddr_in si_other;
WSADATA wsa;
SOCKET s = INVALID_SOCKET;                         // change int s to SOCKET s, avoid possible loss of data
int slen = sizeof(si_other);
int8_t emg_val[8] = {};
//[timestamp + 8*int8]
int8_t udpData[8 + 8] = {};
uint64_t timestamp1 = 0;

class DataCollector : public myo::DeviceListener {
public:
    DataCollector()
		: emgSamples(), roll_w(0), pitch_w(0), yaw_w(0)
    {
    }
	int roll_w, pitch_w, yaw_w;
	long t0, tLast;

	void onOrientationData(myo::Myo* myo, uint64_t timestamp, const myo::Quaternion<float>& quat)
	{
		using std::atan2;
		using std::asin;
		using std::sqrt;
		using std::max;
		using std::min;

		// Calculate Euler angles (roll, pitch, and yaw) from the unit quaternion.
		float roll = atan2(2.0f * (quat.w() * quat.x() + quat.y() * quat.z()),
			1.0f - 2.0f * (quat.x() * quat.x() + quat.y() * quat.y()));
		float pitch = asin(max(-1.0f, min(1.0f, 2.0f * (quat.w() * quat.y() - quat.z() * quat.x()))));
		float yaw = atan2(2.0f * (quat.w() * quat.z() + quat.x() * quat.y()),
			1.0f - 2.0f * (quat.y() * quat.y() + quat.z() * quat.z()));

		// Convert the floating point angles in radians to a scale from 0 to 18.
		roll_w = static_cast<int>((roll + (float)M_PI) / (M_PI * 2.0f) * 18);
		pitch_w = static_cast<int>((pitch + (float)M_PI / 2.0f) / M_PI * 18);
		yaw_w = static_cast<int>((yaw + (float)M_PI) / (M_PI * 2.0f) * 18);
	}
    // onUnpair() is called whenever the Myo is disconnected from Myo Connect by the user.
    void onUnpair(myo::Myo* myo, uint64_t timestamp)
    {
        // We've lost a Myo.
        // Let's clean up some leftover state.
        emgSamples.fill(0);
    }

    // onEmgData() is called whenever a paired Myo has provided new EMG data, and EMG streaming is enabled.
    void onEmgData(myo::Myo* myo, uint64_t timestamp, const int8_t* emg)
    {
		timestamp1++;
		memcpy(udpData, &timestamp1, sizeof(uint64_t));
		memcpy(udpData+8, emg, 8 * sizeof(uint8_t));
		for (int i = 0; i < 8; i++) {
            emgSamples[i] = emg[i];
        }
		int send_len = sendto(s, (const char*)udpData, 16 * sizeof(int8_t), 0, (struct sockaddr *) &si_other, slen);
		if (send_len == SOCKET_ERROR)
		{
			printf("sendto() failed with error code : %d", WSAGetLastError());
			//exit(0);
		}
    }

    // There are other virtual functions in DeviceListener that we could override here, like onAccelerometerData().
    // For this example, the functions overridden above are sufficient.

    // We define this function to print the current values that were updated by the on...() functions above.
    void print()
    {
        // Clear the current line
        std::cout << '\r';

        // Print out the EMG data.
        for (size_t i = 0; i < emgSamples.size(); i++) {
            std::ostringstream oss;
            oss << static_cast<int>(emgSamples[i]);
            std::string emgString = oss.str();

            std::cout << '[' << emgString << std::string(4 - emgString.size(), ' ') << ']';
        }

        std::cout << std::flush;
    }

    // The values of this array is set by onEmgData() above.
    std::array<int8_t, 8> emgSamples;
};

int main(int argc, char** argv)
{

	//Initialise winsock
	printf("\nInitialising Winsock...");
	if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0) {
		printf("Failed. Error Code : %d", WSAGetLastError());
		exit(EXIT_FAILURE);
	}

	//create socket
	s = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if (s == SOCKET_ERROR)	{
		printf("socket() failed with error code : %d", WSAGetLastError());
		exit(EXIT_FAILURE);
	}
	
	//setup address structure
	memset((char *)&si_other, 0, sizeof(si_other));
	si_other.sin_family = AF_INET;
	si_other.sin_port = htons(SERVERPORT);
	si_other.sin_addr.S_un.S_addr = inet_addr(SERVER);

    // We catch any exceptions that might occur below -- see the catch statement for more details.
    try {
		myo::Hub hub("com.example.emg-data-sample");
		std::cout << "Attempting to find a Myo..." << std::endl;

		myo::Myo* myo = hub.waitForMyo(10000);

		if (!myo) {
			throw std::runtime_error("Unable to find a Myo!");
		}

		std::cout << "Connected to a Myo armband!" << std::endl << std::endl;
		myo->setStreamEmg(myo::Myo::streamEmgEnabled);

		DataCollector collector;

		hub.addListener(&collector);

		while (1) {
			hub.run(1000/20);
			collector.print();
		}
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        std::cerr << "Press enter to continue.";
        std::cin.ignore();

		closesocket(s);
		WSACleanup();
        return 1;
    }
}
