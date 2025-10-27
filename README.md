# 구성

project FLP의 기기의 회로도, PCB, FW 코드 등이 담긴 파일입니다.
호스트 기기의 프레임 워크는 ESP-IDF를 이용하였고 Espressif ide를 이용하여서 CMAKE 형식의 프로젝트로 제작되었습니다.
터미널 기기의 프레임 워크는 STMCUBE를 이용하였습니다. 
host 기기의 같은경우 MCU는 ESP32-WROOM-32E를 이용하였으며 terminal기기의 경우 STM32F103C8T6을 사용하였습니다.


## 세부내용
### Hardware
- 433MHz 대역 통신을 위해 SX1278 모듈 기반의 안테나 매칭을 포함한 LoRa RF 공통 회로 설계
- MCP73831(Li-Po 충전) 및 AP2112K-3.3(LDO)을 사용하여 USB-C/배터리 입력을 받는 3.3V 전원 시스템 공통 회로 구성
- CP2104 USB-to-UART 브릿지 IC와 USB-C 커넥터를 사용한 PC 시리얼 통신 및 펌웨어 다운로드 공통 회로 구현
- L86-M33 GPS 모듈을 탑재하여 위치 정보 수집을 위한 공통 회로 및 안테나 인터페이스 설계
- Host(ESP32) 보드에 펌웨어의 효율적인 디버깅을 위한 JTAG 인터페이스 회로 추가 설계

### Firmware
- Host(ESP32) 펌웨어는 C 언어와 ESP-IDF(FreeRTOS)를 기반으로, Terminal(STM32F103) 펌웨어는 C 언어와 HAL 라이브러리를 기반으로 개발하여 LoRa 통신 시스템을 구축
- Terminal은 UART2로 GPS 모듈의 NMEA 센텐스를 파싱하며, 5초 주기 또는 '호출'/'위험' 버튼 입력 시 고유 ID, GPS 좌표, Target ID, 위험 상태를 "ID,Lat,Lon,TargetID,Danger" 형식의 문자열 패킷으로 LoRa 전송
- Host는 SPI를 통해 이 LoRa 패킷을 수신, split_lora_response 함수로 파싱하여 Target ID를 확인하고, TerminalDevice 구조체 배열로 관리되는 단말기 목록의 최신 GPS 좌표와 위험 상태를 갱신
- Host는 Wi-Fi(AP+STA 모드)로 인터넷에 연결하며, WSS 클라이언트를 통해 원격 서버로 모든 단말기 정보와 자체 GPS 수신 정보를 통합하여 주기적으로 JSON 전송
- Wi-Fi AP 모드에서 Captive Portal 기능을 포함한 웹 서버를 구동하여 사용자에게 Wi-Fi 연결 정보 및 AP 정보 설정 페이지를 제공하며, 이 설정값들과 등록된 단말기 ID 목록은 NVS에 영속 저장
- 특정 GPIO 버튼(CTRL + FIND_HOST) 조합을 통해 10초간 '단말기 등록 모드'로 진입하며, 이 동안 수신된 "REG,<ID>,<TARGET_ID>" 형식의 LoRa 등록 요청 패킷을 처리하여 새로운 단말기를 시스템에 추가하고 NVS에 저장
