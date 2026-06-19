# 03 Sound UDP Receiver

Jetson 또는 PC에서 보낸 risk 명령을 Wi-Fi UDP로 받아 진동모터와 부저 패턴을 출력하는 코드입니다.

## 업로드 전 설정

`config.example.h`를 복사해서 같은 폴더에 `config.h`를 만듭니다.

```cpp
#pragma once

const char *WIFI_SSID = "YOUR_WIFI_SSID";
const char *WIFI_PASSWORD = "YOUR_WIFI_PASSWORD";
```

`config.h`는 실제 Wi-Fi 비밀번호가 들어가므로 GitHub에 올리지 않습니다.

## UDP 명령 예시

ESP32와 Jetson 또는 PC가 같은 Wi-Fi에 있어야 합니다.

```json
{"risk":0}
{"risk":1}
{"risk":2}
{"risk":3}
```

## risk별 동작

| risk | 동작 |
| --- | --- |
| 0 | 전체 OFF |
| 1 | 약한 진동 |
| 2 | 약한 진동 + 느린 부저 |
| 3 | 강한 진동 + 3연속 부저 |
