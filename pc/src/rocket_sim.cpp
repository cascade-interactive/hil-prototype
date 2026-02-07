// 2D Rocket Simulator with Thrust Vectoring
// Runs physics at 1kHz, sends IMU data to ESP32, receives actuator commands

#define NOMINMAX
#include <windows.h>
#include <setupapi.h>
#include <initguid.h>
#include <devguid.h>

#include <cmath>
#include <cstdio>
#include <cstdint>
#include <string>
#include <vector>
#include <algorithm>

#pragma comment(lib, "setupapi.lib")

// ============== PHYSICS CONSTANTS ==============
constexpr double GRAVITY = 9.81;           // m/s²
constexpr double THRUST_FORCE = 15.0;      // m/s² acceleration from thrust
constexpr double ROCKET_MASS = 1.0;        // kg (normalized)
constexpr double DT = 0.001;               // 1ms timestep (1kHz)
constexpr double DEG_TO_RAD = 3.14159265358979 / 180.0;
constexpr double RAD_TO_DEG = 180.0 / 3.14159265358979;

// ============== ROCKET STATE ==============
struct RocketState {
    double x = 0.0;           // Position X (m)
    double y = 10.0;          // Position Y (m) - start 10m up
    double vx = 0.0;          // Velocity X (m/s)
    double vy = 0.0;          // Velocity Y (m/s)
    double angle = 0.0;       // Rocket body angle (degrees, 0 = vertical)
    double thrust_tilt = 0.0; // Thrust vector tilt from actuator (degrees)
    double time = 0.0;        // Simulation time (seconds)
    bool engine_on = true;
};

// ============== SERIAL PORT ==============
static HANDLE g_serial = nullptr;

struct SerialPortInfo {
    std::string portName;
    std::string friendlyName;
    std::string hardwareId;
};

static std::vector<SerialPortInfo> list_serial_ports() {
    std::vector<SerialPortInfo> results;
    HDEVINFO info = SetupDiGetClassDevs(&GUID_DEVINTERFACE_COMPORT, nullptr, nullptr,
                                        DIGCF_PRESENT | DIGCF_DEVICEINTERFACE);
    if (info == INVALID_HANDLE_VALUE) return results;

    for (DWORD i = 0;; ++i) {
        SP_DEVICE_INTERFACE_DATA ifData{};
        ifData.cbSize = sizeof(SP_DEVICE_INTERFACE_DATA);
        if (!SetupDiEnumDeviceInterfaces(info, nullptr, &GUID_DEVINTERFACE_COMPORT, i, &ifData)) break;

        DWORD requiredSize = 0;
        SetupDiGetDeviceInterfaceDetail(info, &ifData, nullptr, 0, &requiredSize, nullptr);
        std::vector<BYTE> detailBuf(requiredSize);
        auto* detailData = reinterpret_cast<SP_DEVICE_INTERFACE_DETAIL_DATA*>(detailBuf.data());
        detailData->cbSize = sizeof(SP_DEVICE_INTERFACE_DETAIL_DATA);

        SP_DEVINFO_DATA devInfo{};
        devInfo.cbSize = sizeof(SP_DEVINFO_DATA);
        if (!SetupDiGetDeviceInterfaceDetail(info, &ifData, detailData, requiredSize, nullptr, &devInfo)) continue;

        char friendlyName[256] = {0};
        if (!SetupDiGetDeviceRegistryPropertyA(info, &devInfo, SPDRP_FRIENDLYNAME, nullptr,
                                              reinterpret_cast<PBYTE>(friendlyName), sizeof(friendlyName), nullptr)) continue;

        char hardwareId[512] = {0};
        SetupDiGetDeviceRegistryPropertyA(info, &devInfo, SPDRP_HARDWAREID, nullptr,
                                         reinterpret_cast<PBYTE>(hardwareId), sizeof(hardwareId), nullptr);

        std::string fnStr = friendlyName;
        size_t start = fnStr.find("(COM");
        if (start == std::string::npos) continue;
        size_t end = fnStr.find(')', start);
        if (end == std::string::npos) continue;

        SerialPortInfo spi;
        spi.portName = fnStr.substr(start + 1, end - start - 1);
        spi.friendlyName = friendlyName;
        spi.hardwareId = hardwareId;
        results.push_back(std::move(spi));
    }
    SetupDiDestroyDeviceInfoList(info);
    return results;
}

static bool is_probably_esp32(const SerialPortInfo& p) {
    std::string hw = p.hardwareId;
    std::transform(hw.begin(), hw.end(), hw.begin(), ::toupper);
    return hw.find("VID_10C4") != std::string::npos ||
           hw.find("VID_1A86") != std::string::npos ||
           hw.find("VID_0403") != std::string::npos ||
           hw.find("VID_303A") != std::string::npos;
}

static bool open_serial(const std::string& portName) {
    std::string device = "\\\\.\\" + portName;
    g_serial = CreateFileA(device.c_str(), GENERIC_READ | GENERIC_WRITE, 0, nullptr,
                          OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, nullptr);
    if (g_serial == INVALID_HANDLE_VALUE) {
        g_serial = nullptr;
        return false;
    }

    SetupComm(g_serial, 8192, 4096);

    DCB dcb{};
    dcb.DCBlength = sizeof(DCB);
    GetCommState(g_serial, &dcb);
    dcb.BaudRate = 921600;
    dcb.ByteSize = 8;
    dcb.Parity = NOPARITY;
    dcb.StopBits = ONESTOPBIT;
    dcb.fDtrControl = DTR_CONTROL_ENABLE;
    dcb.fRtsControl = RTS_CONTROL_ENABLE;
    dcb.fBinary = TRUE;
    SetCommState(g_serial, &dcb);

    COMMTIMEOUTS timeouts{};
    timeouts.ReadIntervalTimeout = MAXDWORD;
    timeouts.ReadTotalTimeoutConstant = 0;
    timeouts.ReadTotalTimeoutMultiplier = 0;
    SetCommTimeouts(g_serial, &timeouts);
    PurgeComm(g_serial, PURGE_RXCLEAR | PURGE_TXCLEAR);
    return true;
}

static void close_serial() {
    if (g_serial) {
        CloseHandle(g_serial);
        g_serial = nullptr;
    }
}

static int serial_read(char* buf, int maxLen) {
    if (!g_serial) return 0;
    DWORD bytesRead = 0;
    ReadFile(g_serial, buf, maxLen, &bytesRead, nullptr);
    return static_cast<int>(bytesRead);
}

static void serial_write(const char* data, int len) {
    if (!g_serial) return;
    DWORD written = 0;
    WriteFile(g_serial, data, len, &written, nullptr);
}

// ============== HIGH-RES TIMER ==============
static LARGE_INTEGER g_qpcFreq;
static inline void init_timer() { QueryPerformanceFrequency(&g_qpcFreq); }
static inline double get_time_us() {
    LARGE_INTEGER now;
    QueryPerformanceCounter(&now);
    return static_cast<double>(now.QuadPart) * 1000000.0 / static_cast<double>(g_qpcFreq.QuadPart);
}

// ============== PHYSICS UPDATE ==============
static void update_physics(RocketState& r) {
    // Thrust vector angle = rocket angle + actuator tilt
    double thrust_angle_deg = r.angle + r.thrust_tilt;
    double thrust_angle_rad = thrust_angle_deg * DEG_TO_RAD;
    
    // Thrust acceleration components (thrust points "up" relative to rocket)
    double ax = 0.0, ay = -GRAVITY;  // Start with gravity
    
    if (r.engine_on && r.y > 0) {
        // Thrust is along rocket body axis, tilted by thrust_tilt
        // Angle 0 = vertical, positive = tilted right
        ax += THRUST_FORCE * sin(thrust_angle_rad);
        ay += THRUST_FORCE * cos(thrust_angle_rad);
    }
    
    // Update velocity
    r.vx += ax * DT;
    r.vy += ay * DT;
    
    // Update position
    r.x += r.vx * DT;
    r.y += r.vy * DT;
    
    // Simple angle dynamics: rocket tilts based on lateral acceleration
    // This is very simplified - real rockets have complex dynamics
    double angular_response = 0.1; // How much angle changes per lateral accel
    r.angle += r.thrust_tilt * angular_response * DT * 10.0;
    r.angle = std::clamp(r.angle, -45.0, 45.0);
    
    // Ground collision
    if (r.y < 0) {
        r.y = 0;
        r.vy = 0;
        r.vx *= 0.8; // Friction
        r.engine_on = false;
    }
    
    r.time += DT;
}

// ============== LINE PARSER ==============
class LineParser {
public:
    void add_data(const char* data, int len) {
        for (int i = 0; i < len; ++i) {
            char ch = data[i];
            if (ch == '\n') {
                buf_[bufPos_] = '\0';
                if (bufPos_ > 0) {
                    memcpy(lastLine_, buf_, bufPos_ + 1);
                    hasLine_ = true;
                }
                bufPos_ = 0;
            } else if (ch != '\r' && bufPos_ < 63) {
                buf_[bufPos_++] = ch;
            }
        }
    }
    bool has_line() const { return hasLine_; }
    const char* get_line() { hasLine_ = false; return lastLine_; }
private:
    char buf_[64] = {};
    char lastLine_[64] = {};
    int bufPos_ = 0;
    bool hasLine_ = false;
};

// ============== CONSOLE ==============
static HANDLE hConsole = nullptr;

static void init_console() {
    hConsole = GetStdHandle(STD_OUTPUT_HANDLE);
    DWORD mode = 0;
    GetConsoleMode(hConsole, &mode);
    SetConsoleMode(hConsole, mode | ENABLE_VIRTUAL_TERMINAL_PROCESSING);
    
    // Set window title
    SetConsoleTitleA("2D Rocket Simulator - 1kHz Physics");
}

static void hide_cursor() {
    CONSOLE_CURSOR_INFO ci;
    GetConsoleCursorInfo(hConsole, &ci);
    ci.bVisible = FALSE;
    SetConsoleCursorInfo(hConsole, &ci);
}

static void home_cursor() {
    SetConsoleCursorPosition(hConsole, {0, 0});
}

// ============== MAIN ==============
int main() {
    init_console();
    init_timer();
    
    printf("=== 2D ROCKET SIMULATOR ===\n");
    printf("Searching for ESP32...\n\n");
    
    // Find ESP32
    std::string portName;
    while (portName.empty()) {
        auto ports = list_serial_ports();
        for (const auto& p : ports) {
            if (is_probably_esp32(p)) {
                portName = p.portName;
                break;
            }
        }
        if (portName.empty()) {
            printf("Waiting for ESP32...\r");
            Sleep(500);
        }
    }
    
    printf("Found ESP32 on %s\n", portName.c_str());
    if (!open_serial(portName)) {
        printf("Failed to open serial port!\n");
        return 1;
    }
    
    Sleep(500); // Let ESP32 boot
    
    // Clear screen and set up display
    system("cls");
    hide_cursor();
    
    RocketState rocket;
    LineParser parser;
    char readBuf[256];
    char sendBuf[64];
    
    uint64_t tickCount = 0;
    uint64_t sendCount = 0;
    uint64_t recvCount = 0;
    double lastActuatorCmd = 0.0;
    
    double lastPhysicsUs = get_time_us();
    double lastDisplayUs = get_time_us();
    double lastSendUs = get_time_us();
    
    printf("\n");  // Leave space for display
    
    while (true) {
        double nowUs = get_time_us();
        
        // Physics at 1kHz
        if (nowUs - lastPhysicsUs >= 1000.0) {
            lastPhysicsUs += 1000.0;
            if (nowUs - lastPhysicsUs > 5000.0) lastPhysicsUs = nowUs; // Reset if too far behind
            
            update_physics(rocket);
            tickCount++;
        }
        
        // Send IMU data at 1kHz
        if (nowUs - lastSendUs >= 1000.0) {
            lastSendUs += 1000.0;
            if (nowUs - lastSendUs > 5000.0) lastSendUs = nowUs;
            
            // Calculate velocity magnitude and direction
            double vel = sqrt(rocket.vx * rocket.vx + rocket.vy * rocket.vy);
            
            // Send: I,seq,velocity,angle\n
            int len = snprintf(sendBuf, sizeof(sendBuf), "I,%llu,%.2f,%.2f\n",
                              (unsigned long long)sendCount, vel, rocket.angle);
            serial_write(sendBuf, len);
            sendCount++;
        }
        
        // Read serial responses
        int bytesRead = serial_read(readBuf, sizeof(readBuf) - 1);
        if (bytesRead > 0) {
            parser.add_data(readBuf, bytesRead);
        }
        
        // Parse actuator commands: A,tilt\n
        while (parser.has_line()) {
            const char* line = parser.get_line();
            if (line[0] == 'A' && line[1] == ',') {
                double tilt = atof(line + 2);
                rocket.thrust_tilt = std::clamp(tilt, -15.0, 15.0);
                lastActuatorCmd = rocket.thrust_tilt;
                recvCount++;
            }
        }
        
        // Display at 10Hz
        if (nowUs - lastDisplayUs >= 100000.0) {
            lastDisplayUs = nowUs;
            
            home_cursor();
            printf("+==============================================================+\n");
            printf("|           2D ROCKET SIMULATOR (1kHz Physics)                 |\n");
            printf("+==============================================================+\n");
            printf("|  Port: %-20s  Baud: 921600                  |\n", portName.c_str());
            printf("+--------------------------------------------------------------+\n");
            printf("|  ROCKET STATE                                                |\n");
            printf("+--------------------------------------------------------------+\n");
            printf("|  Position:   X = %8.2f m    Y = %8.2f m              |\n", rocket.x, rocket.y);
            printf("|  Velocity:  Vx = %8.2f m/s  Vy = %8.2f m/s            |\n", rocket.vx, rocket.vy);
            printf("|  Speed:         %8.2f m/s                               |\n", 
                   sqrt(rocket.vx*rocket.vx + rocket.vy*rocket.vy));
            printf("|  Body Angle:    %8.2f deg   (target: %.1f -> %.1f)       |\n", 
                   rocket.angle, -5.0, 5.0);
            printf("|  Engine:        %-8s                                    |\n", 
                   rocket.engine_on ? "ON" : "OFF");
            printf("+--------------------------------------------------------------+\n");
            printf("|  THRUST VECTORING (from ESP32)                               |\n");
            printf("+--------------------------------------------------------------+\n");
            printf("|  Actuator Tilt: %8.2f deg                                |\n", lastActuatorCmd);
            printf("|  Thrust Angle:  %8.2f deg  (body + tilt)                 |\n", 
                   rocket.angle + rocket.thrust_tilt);
            printf("+--------------------------------------------------------------+\n");
            printf("|  STATISTICS                                                  |\n");
            printf("+--------------------------------------------------------------+\n");
            printf("|  Sim Time:      %8.2f sec                                |\n", rocket.time);
            printf("|  Physics Ticks: %8llu                                    |\n", (unsigned long long)tickCount);
            printf("|  IMU Sent:      %8llu      Actuator Recv: %8llu      |\n", 
                   (unsigned long long)sendCount, (unsigned long long)recvCount);
            printf("+==============================================================+\n");
            printf("\n  Press Ctrl+C to exit\n");
        }
        
        // Small sleep to prevent CPU spinning
        Sleep(0);
    }
    
    close_serial();
    return 0;
}
