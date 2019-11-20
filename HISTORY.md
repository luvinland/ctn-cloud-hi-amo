## ChangeLog
### 1.0 - 2019-11-18

**Enhancements**

* Embed 응답 음원 재생을 ESP 단에서 처리하도록 수정함.
  * 모든 재생 음원 적용 가능.
  * `// Jace. 191118. Playback voice prompt using ESP32.` 주석 참조.
* I2S output (마이크 음성 전달) 성능 최적화.
  * `// Jace. 191118. Improve i2s output perpormance.` 주석 참조.

**API Changes**

* None.

**Bug Fixed**

* None.

**Known Issues/Improvements**

* git 배포 버전.

### 0.4 - 2019-11-07

**Enhancements**

* CTN base code 작업함.
  * I94124 - ESP32 SPI2_I2S, I2C, PWM 기능 추가. `ETN 191018 base`
* Cloud 및 Embed 기능 적용.
* `// Jace. 191107. SPI2_I2S intput/output, PWM, I2C Master working with ESP32` 주석 참조.

**API Changes**

* None.

**Bug Fixed**

* None.

**Known Issues/Improvements**

* 일정상 임시로 I94124 에서 Embed 응답 음원 재생 (2개만, ON/OFF) 함.
  * ESP 에서 재생하도록 수정 예정.

### 0.3 - 2019-10-18

**Enhancements**

* Bug Fixed.

**API Changes**

* None.

**Bug Fixed**

* 전원 인가시 Uart port 노이즈로 인한 VP 오동작 현상 발생.
  * 대략 500ms 노이즈 발생. uart init 이전 540ms delay 적용함.
  * `// Jace. 191018. Ignore uart noise when power on. 540ms.` 주석 참조.

**Known Issues/Improvements**

* None.

### 0.2 - 2019-10-14

**Enhancements**

* AMO Vent Cleaner 와 UART 통신 구현. (Vent ↔ Module 양방향).
* AMO Vent Cleaner 설계 v0.4.pdf 에 의거한 명령어 수행.
* `// Jace. 191014. Add control interface using UART0.` 주석 참조.
  * 음성인식모드 ON/OFF 음원 2개 추가됨.

**API Changes**

* None.

**Bug Fixed**

* None.

**Known Issues/Improvements**

* None.

### 0.1 - 2019-09-25

**Enhancements**

* Adaptation 진행.
  * `하이아모`, `전원켜`, `전원꺼`, `환기1단`, `환기2단`, `환기3단`, `시간예약1시간`, `시간예약4시간`, `시간예약8시간`, `인공지능모드`
  * 저음, 중음, 고음 3가지 형태로 5명 녹음

**API Changes**

* None.

**Bug Fixed**

* None.

**Known Issues/Improvements**

* 하이모비스 향 Adaptation 추후 진행 예정.
  * `하이모비스`, `트렁크열어`, `트렁크닫아`
  * 저음, 중음, 고음 3가지 형태로 5명 녹음
