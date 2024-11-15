#include <Wire.h>
#include <math.h>

#define MPU_ADDR 0x68              // Alamat I2C untuk MPU6050
#define BAUD_RATE 9600             // Baud rate (jumlah simbol atau sinyal yang dikirimkan per detik) untuk komunikasi serial

// Var. Kalman
float angle = 0;                  // Sudut estimasi
float bias = 0;                   // Bias giroskop
float P[2][2] = {0, 0, 0, 0};     // Matriks error covariance
float Q_angle = 0.5;             // Variansi proses untuk sudut
float Q_bias = 0.00065;               // Variansi proses untuk bias
float R_measure = 0.000098;           // Variansi pengukuran

int ROLL_ALERT_THRESHOLD = 40;    // Batas untuk roll yang dianggap alert

unsigned long lastUpdateTime = 0;

// inisialisasi MPU6050
void setupMPU() {
    Wire.begin();
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x6B);             // Register Power Management
    Wire.write(0);                // Set ke 0 untuk membangunkan MPU6050
    Wire.endTransmission(true);
}

// baca data MPU6050
void readMPU(int16_t &ax, int16_t &ay, int16_t &az, int16_t &gx, int16_t &gy, int16_t &gz) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B);            // Mulai membaca dari register akselerometer
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, 14, true);

    ax = Wire.read() << 8 | Wire.read();
    ay = Wire.read() << 8 | Wire.read();
    az = Wire.read() << 8 | Wire.read();
    gx = Wire.read() << 8 | Wire.read();
    gy = Wire.read() << 8 | Wire.read();
    gz = Wire.read() << 8 | Wire.read();
}

// Kalman filter untuk stabilisasi sudut
float kalmanUpdate(float newAngle, float newRate, float dt) {
    // Prediction
    angle += dt * (newRate - bias);
    P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle);
    P[0][1] -= dt * P[1][1];
    P[1][0] -= dt * P[1][1];
    P[1][1] += Q_bias * dt;

    // Update
    float S = P[0][0] + R_measure;
    float K[2];
    K[0] = P[0][0] / S;
    K[1] = P[1][0] / S;

    float y = newAngle - angle;  // Error/Inovasi:untuk menghasilkan estimasi yang lebih akurat
    angle += K[0] * y;
    bias += K[1] * y;

    float P00_temp = P[0][0];
    float P01_temp = P[0][1];

    P[0][0] -= K[0] * P00_temp;
    P[0][1] -= K[0] * P01_temp;
    P[1][0] -= K[1] * P00_temp;
    P[1][1] -= K[1] * P01_temp;

    return angle;
}

void setup() {
    Serial2.begin(57600, SERIAL_8N1, 16, 17);  // Telemetri pada pin 16 (TX) dan 17 (RX)
    Serial.begin(BAUD_RATE);    // Inisialisasi komunikasi serial untuk monitor
    Wire.begin();
    setupMPU();                 // Inisialisasi sensor MPU6050

    lastUpdateTime = millis();
}

void loop() {
    int16_t ax, ay, az, gx, gy, gz;
    readMPU(ax, ay, az, gx, gy, gz);

    // Hitung waktu delta
    unsigned long currentTime = millis();
    float deltaTime = (currentTime - lastUpdateTime) / 1000.0;  // dalam detik
    lastUpdateTime = currentTime;

    // Hitung Roll dan Pitch dari akselerometer
    float accelRoll = atan2(ay, az) * 180 / PI;
    float accelPitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180 / PI;

    float roll = kalmanUpdate(accelRoll, (gx / 131.0), deltaTime);      // Roll dengan Kalman Filter
    float pitch = kalmanUpdate(accelPitch, (gy / 131.0), deltaTime);    // Pitch dengan Kalman Filter
    float yaw = yaw + (gz / 131.0) * deltaTime;    // Perhitungan yaw dengan integrasi gyroscope (tanpa Kalman)

    // output data Serial Monitor
    String smonitor = "Roll: " + String(roll) + "   Pitch: " + String(pitch) + "   Yaw: " + String(yaw);

    //Serial Plotter
    Serial.print(roll);
    Serial.print("\t");
    Serial.println(pitch);

    // Gabungkan data roll, pitch dan alert menjadi satu string untuk dikirim ke telemetri
    String telemetryData = String(roll) + "," + String(pitch);

    // Jika Roll melebihi batas, tambahkan pesan alert ke data telemetry
    if (abs(roll) > ROLL_ALERT_THRESHOLD) {
        telemetryData += ",ALERT: Roll melebihi " + String(ROLL_ALERT_THRESHOLD) + " derajat!";
        smonitor += "    ALERT: Roll melebihi " + String(ROLL_ALERT_THRESHOLD) + " derajat!";
        ROLL_ALERT_THRESHOLD += 10;  // Meningkatkan ambang batas setelah alert
    }

    if (abs(roll) < 40) {
        ROLL_ALERT_THRESHOLD = 40;  // Mengatur ulang ambang batas ke 40 jika roll normal
    }

    // Kirim data Roll, Pitch, dan Alert ke modul telemetri dalam satu baris
    Serial2.println(telemetryData);  // Mengirim data ke telemetri dalam format CSV atau dengan alert jika perlu

    // Serial.println(smonitor);

    delay(20);  // Jeda sebelum pembacaan berikutnya untuk pengiriman data real-time
}
