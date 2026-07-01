#imu 가속도, gps 좌표 찍기 (0503)
// [박중선] IMU + GPS 통합 읽기 테스트
#include <Wire.h>
#include <ICM_20948.h>
#include <TinyGPSPlus.h>
ICM_20948_I2C imu;
TinyGPSPlus gps;
HardwareSerial gpsSerial(2); // UART2
void setup() {
 Serial.begin(115200);
 delay(1000);
 // IMU 초기화
 Wire.begin(21, 22);
 imu.begin(Wire, 1); // AD0=HIGH → 0x69
 if (imu.status != ICM_20948_Stat_Ok) {
 Serial.println("[IMU] 연결 실패!");
 } else {
 Serial.println("[IMU] 연결 성공 (0x69)");
 }
 // GPS 초기화
 gpsSerial.begin(9600, SERIAL_8N1, 16, 17);
 Serial.println("[GPS] UART2 시작 (RX=16, TX=17)");
 Serial.println("========== 센서 통합 테스트 시작 ==========");
}
void loop() {
 // --- IMU 읽기 ---
 if (imu.dataReady()) {
 imu.getAGMT();
 Serial.print("[IMU] Ax=");
 Serial.print(imu.accX() / 1000.0, 2);
 Serial.print(" Ay=");
 Serial.print(imu.accY() / 1000.0, 2);
 Serial.print(" Az=");
 Serial.print(imu.accZ() / 1000.0, 2);
 Serial.print(" | Gx=");
 Serial.print(imu.gyrX(), 1);
 Serial.print(" Gy=");
 Serial.print(imu.gyrY(), 1);
 Serial.print(" Gz=");
 Serial.println(imu.gyrZ(), 1);
 }
 // --- GPS 읽기 ---
 while (gpsSerial.available() > 0) {
 gps.encode(gpsSerial.read());
 }
 Serial.print("[GPS] chars=");
Serial.print(gps.charsProcessed());
Serial.print(" sats=");
Serial.print(gps.satellites.value());
Serial.print(" valid=");
Serial.println(gps.location.isValid());
 if (gps.location.isUpdated()) {
 Serial.print("[GPS] 위도=");
Serial.print(gps.location.lat(), 6);
 Serial.print(" 경도=");
 Serial.print(gps.location.lng(), 6);
 Serial.print(" 위성수=");
 Serial.println(gps.satellites.value());
 }
 Serial.println("──────────────────────────────");
 delay(500);
}
