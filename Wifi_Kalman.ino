/*
회전판 위 IMU 센서와 I2C 통신을 하는 ESP32 보드에 들어간 코드
센서 값을 읽어와 각도를 추정하는 역할과 웹소켓 서버를 구축한 뒤 클라이언트에게 각도, 각속도 정보를 제공하는 역할을 수행
*/

#include <Wire.h>
#include <WiFi.h>
#include <WebSocketsServer.h>

const char* ssid = "S23";
const char* password = "dgm2025!";

WebSocketsServer webSocket = WebSocketsServer(81);

const int MPU = 0x68;
int16_t AcX, AcY, AcZ, GyX, GyY, GyZ;
double GyX_offset = 0, GyY_offset = 0, GyZ_offset = 0;

/*
칼만필터 Class
메서드 : getKalman => 오차 공분산 행렬 업데이트와 각도 정보 예측값 반환
        init => 오차공분산행렬, 노이즈 공분산 초기화 => 데이터 시트를 참조해서 값 설정할 것
엘리먼트 : Q_angle Q_gyro R_measure => 데이터 시트 참조해서 값 설정, 센서 출력값을 확인 후 세부 조정할 것
           P[2][2] K[2] => 오차 공분산 행렬과 칼만 이득,,
*/
//
class Kalman
{
public:
    double getKalman(double accAngle, double gyroRate, double dt) 
    {
        angle += dt * (gyroRate - bias);
        P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle);
        P[0][1] -= dt * P[1][1];
        P[1][0] -= dt * P[1][1];
        P[1][1] += Q_gyro * dt;

        double S = P[0][0] + R_measure;
        K[0] = P[0][0] / S;
        K[1] = P[1][0] / S;

        double y = accAngle - angle;
        angle += K[0] * y;
        bias += K[1] * y;

        double P_temp[2] = {P[0][0], P[0][1]};
        P[0][0] -= K[0] * P_temp[0];
        P[0][1] -= K[0] * P_temp[1];
        P[1][0] -= K[1] * P_temp[0];
        P[1][1] -= K[1] * P_temp[1];

        return angle;
    }

    void init(double Q_angle_, double Q_gyro_, double R_measure_) 
    {
        Q_angle = Q_angle_;
        Q_gyro = Q_gyro_;
        R_measure = R_measure_;
        angle = 0;
        bias = 0;
        P[0][0] = P[0][1] = P[1][0] = P[1][1] = 0;
    }

private:
    double Q_angle, Q_gyro, R_measure;
    double angle, bias;
    double P[2][2], K[2];
};

Kalman kalmanRoll, kalmanPitch; //Roll Pitch 값을 예측하기 위한 칼만 클래스 객체 두 개 생성

double roll, pitch, yaw; 
unsigned long previousMillis = 0;
const double dt = 0.1; //0.1초 간격

/*
setup
보드레이트 설정과 자이로스코프 민감도 설정
정지 상태에서의 Offset 계산산
칼만필터 초기화
와이파이 연결과 Ip 주소 할당, 웹소켓 서버 생성을 담당
*/
void setup() 
{
    Serial.begin(115200);
    Wire.begin();

    Wire.beginTransmission(MPU);
    Wire.write(0x6B);
    Wire.write(0); 
    Wire.endTransmission(true);

    //자이로스코프 Sensitivity 설정 (500 deg/s)
    Wire.beginTransmission(MPU);
    Wire.write(0x1B); //GYRO_CONFIG 레지스터 주소
    Wire.write(0x08); //FS_SEL = 1 설정
    Wire.endTransmission(true);

    const int numSamples = 1000;
    long GyX_sum = 0, GyY_sum = 0, GyZ_sum = 0;

    for (int i = 0; i < numSamples; i++) 
    {
        Wire.beginTransmission(MPU);
        Wire.write(0x43);
        Wire.endTransmission(false);
        Wire.requestFrom(MPU, 6, true);

        GyX = Wire.read() << 8 | Wire.read();
        GyY = Wire.read() << 8 | Wire.read();
        GyZ = Wire.read() << 8 | Wire.read();

        GyX_sum += GyX;
        GyY_sum += GyY;
        GyZ_sum += GyZ;

        delay(1);
    }

    GyX_offset = GyX_sum / numSamples;
    GyY_offset = GyY_sum / numSamples;
    GyZ_offset = GyZ_sum / numSamples;

    kalmanRoll.init(0.001, 0.003, 0.03);
    kalmanPitch.init(0.001, 0.003, 0.03);

    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) 
    {
        delay(1000);
        Serial.println("Connecting to WiFi...");
    }
    Serial.println("WiFi connected!");
    Serial.println(WiFi.localIP());

    webSocket.begin();
    webSocket.onEvent(webSocketEvent);
}

/*
클라이언트가 접속한 상태인지 확인 후
mpu6050으로부터 가속도 각속도 값을 읽어옴
가속도계로부터 읽어온 가속도 값을 통해 현재 각도 추정, 각속도 값과 함께 칼만필터에 넣어 정밀한 각도 정보 반환
Yaw는 가속도 값으로부터 추정할 수 없으니 각속도를 통해서만 추정 가능 => 드리프트가 심하게 발생
Yaw에도 칼만필터를 적용하기 위해서는 지자계 센서가 더해진 9축 IMU 센서를 사용해야함

추정한 Roll Pitch Yaw 값을 Broadcast로 웹소켓 서버에 접속해 있는 클라이언트 모두에서 발송
*/
void loop() 
{
    webSocket.loop();
    unsigned long currentMillis = millis();

    //0.1초 단위로 데이터 읽어옴
    if (currentMillis - previousMillis >= 100) 
    {
        previousMillis = currentMillis;

        Wire.beginTransmission(MPU);
        Wire.write(0x3B); //mpu6050 데이터 시트 참조 => 가속도 정보 레지스터 주소 확인
        Wire.endTransmission(false);
        Wire.requestFrom(MPU, 6, true);

        AcX = Wire.read() << 8 | Wire.read();
        AcY = Wire.read() << 8 | Wire.read();
        AcZ = Wire.read() << 8 | Wire.read();
        
        //가속도계 민감도 2g => 16384로 나눠주어야 [g]단위
        AcX /= 16384.0;
        AcY /= 16384.0;
        AcZ /= 16384.0;

        Wire.beginTransmission(MPU);
        Wire.write(0x43);
        Wire.endTransmission(false);
        Wire.requestFrom(MPU, 6, true);

        GyX = Wire.read() << 8 | Wire.read();
        GyY = Wire.read() << 8 | Wire.read();
        GyZ = Wire.read() << 8 | Wire.read();

        // Offset 제거 및 단위 변환 [deg/sec] => 센서 민감도가 500 deg/ sec 모드이니 65.5
        double X_DEL_ANG = (GyX - GyX_offset) / 65.5;
        double Y_DEL_ANG = (GyY - GyY_offset) / 65.5;
        double Z_DEL_ANG = (GyZ - GyZ_offset) / 65.5;

        double accRoll = atan2(AcY, sqrt(AcX * AcX + AcZ * AcZ)) * 180 / PI;
        double accPitch = atan2(-AcX, sqrt(AcY * AcY + AcZ * AcZ)) * 180 / PI;

        roll = kalmanRoll.getKalman(accRoll, X_DEL_ANG, dt);
        pitch = kalmanPitch.getKalman(accPitch, Y_DEL_ANG, dt);
        yaw += Z_DEL_ANG * dt;

        String data = "{\"X_DEL_ANG\":" + String(X_DEL_ANG, 2) +
                      ",\"Y_DEL_ANG\":" + String(Y_DEL_ANG, 2) +
                      ",\"Z_DEL_ANG\":" + String(Z_DEL_ANG, 2) +
                      ",\"ROLL\":" + String(roll, 2) +
                      ",\"PITCH\":" + String(pitch, 2) +
                      ",\"YAW\":" + String(yaw, 2) + "}";
        webSocket.broadcastTXT(data);
    }
}

/*
클라이언트가 서버에 접속해 있는지 확인하는 메서드드
*/
void webSocketEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t length)
{
    if (type == WStype_CONNECTED) 
    {
        Serial.printf("Client %u connected\n", num);
    } 
    else if (type == WStype_DISCONNECTED) 
    {
        Serial.printf("Client %u disconnected\n", num);
    }
}
