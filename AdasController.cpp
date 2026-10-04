#define _WINSOCK_DEPRECATED_NO_WARNINGS
#include <iostream>
#include <chrono>
#include <thread>
#include <cstdint>
#include <winsock2.h>

#pragma comment(lib, "ws2_32.lib")

#pragma pack(push, 1)  // ép cấu trúc gắn chặt theo từng 1 byte
struct VehicleSensorFrame {
    uint32_t canId;  // mã định danh CAN 4 byte
    uint8_t  dlc;  // Độ dài dữ liệu (1 byte)
    float    param1;  // Khoảng cách tới xe trước (4 bytes, float IEEE 754)
    float    param2;  //  // Vận tốc tương đối (4 bytes, float IEEE 754)
};

struct VehicleActuatorFrame {
    uint32_t canId;
    uint8_t  state;   // Trạng thái FSM của ADAS (1 byte)
    float    throttle; // % Bướm ga yêu cầu [0.0 - 100.0] (4 bytes)
    float    brake;  //% Lực phanh yêu cầu [0.0 - 100.0] (4 bytes)
};
#pragma pack(pop)  //Khôi phục lại quy tắc căn chỉnh bộ nhớ chuẩn của trình biên dịch sau khi định nghĩa xong 2 struct.

// các trạng thái của adas
enum class AdasState : uint8_t {
    STANDBY = 0,  // chờ , chưa kích hoạt 
    ACC_ACTIVE = 1,   //  Ga tự động thích ứng đang bám đuôi giữ khoảng cách
    AEB_TRIGGERED = 2,  //phanh khẩn cấp tự động đã can thiệp
    FAULT_SAFE = 3  //Chế độ an toàn dự phòng khi phát hiện lỗi
};
// Lớp điều khiển trung tâm
class AccAebController {
private:
    AdasState currentState = AdasState::STANDBY;  // // Trạng thái mặc định ban đầu là chờ 
    float targetDistance = 100.0f;  //  // Khoảng cách an toàn ban đầu (100m)
    float relativeSpeed = 0.0f;  //// Vận tốc tương đối (0 m/s: 2 xe chạy cùng tốc độ)
    float setpointDistance = 30.0f;  //// Khoảng cách mục tiêu mong muốn cần duy trì (30m)
    float prevError = 0.0f;  // // Sai số ở chu kỳ trước
    float integral = 0.0f; // Biến tích lũy tổng sai số cho khâu I của PID

public:
    void updateSensors(float dist, float relSpeed) {
        targetDistance = dist;
        relativeSpeed = relSpeed;
        if (currentState == AdasState::STANDBY) currentState = AdasState::ACC_ACTIVE;
    }

    void execute10msLoop(VehicleActuatorFrame& outCmd) {
        outCmd.canId = 0x200;
        outCmd.throttle = 0.0f;
        outCmd.brake = 0.0f;
        outCmd.state = static_cast<uint8_t>(currentState);

        if (currentState == AdasState::AEB_TRIGGERED) {
            outCmd.brake = 100.0f;
            return;
        }

        if (targetDistance > 0.0f && relativeSpeed < -0.1f) {
            float ttc = targetDistance / (-relativeSpeed);
            if (ttc < 1.6f) {
                currentState = AdasState::AEB_TRIGGERED;
                outCmd.state = static_cast<uint8_t>(currentState);
                outCmd.throttle = 0.0f;
                outCmd.brake = 100.0f;
                return;
            }
        }

        if (currentState == AdasState::ACC_ACTIVE) {
            float error = targetDistance - setpointDistance;
            
            integral += error * 0.01f;
            if (integral > 20.0f) integral = 20.0f;
            if (integral < -20.0f) integral = -20.0f;

            float kp = 1.8f, ki = 0.05f, kd = 1.2f;
            float controlOut = kp * error + ki * integral + kd * relativeSpeed;

            if (controlOut > 0.0f) {
                outCmd.throttle = (controlOut > 100.0f) ? 100.0f : controlOut;
                outCmd.brake = 0.0f;
            }
            else {
                outCmd.throttle = 0.0f;
                float reqBrake = -controlOut;
                outCmd.brake = (reqBrake > 80.0f) ? 80.0f : reqBrake;
            }
            outCmd.state = static_cast<uint8_t>(currentState);
        }
    }
};

int main() {
    WSADATA wsa;
    WSAStartup(MAKEWORD(2, 2), &wsa);

    SOCKET serverSock = socket(AF_INET, SOCK_DGRAM, 0);
    sockaddr_in serverAddr{}, clientAddr{};
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(9000);
    serverAddr.sin_addr.s_addr = INADDR_ANY;

    bind(serverSock, (sockaddr*)&serverAddr, sizeof(serverAddr));

    u_long mode = 1;
    ioctlsocket(serverSock, FIONBIO, &mode);

    std::cout << "[C++ ADAS CORE] Controller running at 10ms loop on Port 9000...\n";

    AccAebController controller;
    bool running = true;

    while (running) {
        auto loopStart = std::chrono::steady_clock::now();

        VehicleSensorFrame rxFrame;
        int clientLen = sizeof(clientAddr);
        int bytes = recvfrom(serverSock, (char*)&rxFrame, sizeof(rxFrame), 0, (sockaddr*)&clientAddr, &clientLen);

        if (bytes > 0 && rxFrame.canId == 0x110) {
            controller.updateSensors(rxFrame.param1, rxFrame.param2);

            VehicleActuatorFrame txCmd{};
            controller.execute10msLoop(txCmd);

            if (clientAddr.sin_port != 0) {
                sendto(serverSock, (char*)&txCmd, sizeof(txCmd), 0, (sockaddr*)&clientAddr, sizeof(clientAddr));
            }
        }

        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - loopStart);
        if (elapsed.count() < 10) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10 - elapsed.count()));
        }
    }

    closesocket(serverSock);
    WSACleanup();
    return 0;
}