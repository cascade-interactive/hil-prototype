#include <Arduino.h>

// ESP32 Rocket Controller for demo purposes
// Receives fake IMU data (velocity, angle), calculates trajectory, sends actuator commands
// It should gradually tilt from -5° to +5° over 30 seconds

// ============== PROTOCOL ==============
// Receive: I,seq,velocity,angle\n  (IMU data from simulator)
// Send:    A,tilt\n                (Actuator command back)

// =========== CONFIGURATION ===========
constexpr float START_ANGLE = -5.0f;    // Starting tilt target (degrees)
constexpr float END_ANGLE = 5.0f;       // Ending tilt target (degrees)
constexpr float TRANSITION_TIME = 30.0f; // Time to go from start to end (seconds)
constexpr float CONTROL_GAIN = 0.5f;    // How aggressively to correct

// =============== STATE ===============
static uint32_t g_startTime = 0;
static bool g_started = false;
static uint32_t g_lastRecvSeq = 0;
static uint32_t g_recvCount = 0;
static uint32_t g_sendCount = 0;

// Current IMU data
static float g_velocity = 0.0f;
static float g_bodyAngle = 0.0f;

// Line parsing buffer
static char g_lineBuf[64];
static int g_linePos = 0;

// ============== CONTROL LOGIC ==============
float calculate_actuator_command() {
    if (!g_started) return 0.0f;
    
    // Calculate elapsed time
    float elapsed = (millis() - g_startTime) / 1000.0f;
    
    // Calculate target angle based on time (linear interpolation)
    float progress = elapsed / TRANSITION_TIME;
    if (progress > 1.0f) progress = 1.0f;
    
    float targetAngle = START_ANGLE + (END_ANGLE - START_ANGLE) * progress;
    
    // Simple proportional control: tilt actuator to correct body angle toward target
    float error = targetAngle - g_bodyAngle;
    float tiltCommand = error * CONTROL_GAIN;
    
    // Clamp actuator command
    if (tiltCommand > 10.0f) tiltCommand = 10.0f;
    if (tiltCommand < -10.0f) tiltCommand = -10.0f;
    
    return tiltCommand;
}

// ============== PARSING ==============
void process_line(const char* line) {
    // Expected: I,seq,velocity,angle
    if (line[0] != 'I' || line[1] != ',') return;
    
    // Parse sequence
    const char* ptr = line + 2;
    char* end;
    uint32_t seq = strtoul(ptr, &end, 10);
    if (*end != ',') return;
    
    // Parse velocity
    ptr = end + 1;
    float velocity = strtof(ptr, &end);
    if (*end != ',') return;
    
    // Parse angle
    ptr = end + 1;
    float angle = strtof(ptr, &end);
    
    // Store values
    g_lastRecvSeq = seq;
    g_velocity = velocity;
    g_bodyAngle = angle;
    g_recvCount++;
    
    // Start timer on first valid packet
    if (!g_started) {
        g_started = true;
        g_startTime = millis();
    }
    
    // Calculate and send actuator command
    float tiltCmd = calculate_actuator_command();
    
    // Send response
    char outBuf[32];
    int len = snprintf(outBuf, sizeof(outBuf), "A,%.2f\n", tiltCmd);
    Serial.write(outBuf, len);
    g_sendCount++;
}

// ============== SETUP ==============
void setup() {
    Serial.begin(921600);
    Serial.setTxBufferSize(2048);
    delay(100);
}

// ============== MAIN LOOP ==============
void loop() {
    // Read and parse incoming data
    while (Serial.available()) {
        char ch = Serial.read();
        
        if (ch == '\n') {
            g_lineBuf[g_linePos] = '\0';
            if (g_linePos > 0) {
                process_line(g_lineBuf);
            }
            g_linePos = 0;
        } else if (ch != '\r' && g_linePos < 62) {
            g_lineBuf[g_linePos++] = ch;
        }
    }
}

