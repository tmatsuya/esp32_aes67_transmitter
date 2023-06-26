# ESP32用 AES67 Transmitter

## 必要ハードウェア環境

 1-1. ESP32

  [検証済みESP32](https://akizukidenshi.com/catalog/g/gM-13628/)


 1-2. 外付けADC

  [検証済みADC] Pmod I2S2 (https://digilent.com/reference/pmod/pmodi2s2/start)


## 必要ソフトウェア環境

 2-1. ESP-IDF

  [ESP-IDF v5.1](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/get-started/index.html)


## 制限事項(2023/02/22 14:00現在)

 3-1. 16(/24)Bit、2Channel、(1/)5ms単位対応

 3-2. 音声入力のみ

 3-3. SDP送信のみ対応



## コンパイルと実行方法

 4-1. ESP-IDFをインストール

 4-2. . ~/esp/esp-idf/export.sh

 4-3. git clone https://github.com/tmatsuya/esp32_aes67_transmitter.git

 4-4. cd esp32_aes67_transmitter

 4-5. idf.py  menuconfig  ... Menuより"Example Connection Config"からWiFi SSIDとWiFi Passwordも設定すること。さらにソース上の UDP_PORT と MULTICAST_IPV4_ADDR マクロに初期値のマルチキャストアドレスを設定すること。

 4-6. idf.py -p /dev/ESP32シリアルデバイス flash






## ESP32と外付けADC(Pmod I2S2)の配線図
```
  ESP32                ADC(I2S2)
  IO0   ------   PIN7  A/D MCLK
  IO26  ------   PIN8  A/D SCLK
  IO25  ------   PIN9  A/D SDOUT
  IO22  ------   PIN10 A/D LRCLK
  GND   ------   PIN11 GND
  3.3V  ------   PIN12 VCC (3.3V only !)
```

![配線写真 Pmod I2S2](/photo_pmodi2s2.jpg)



## ESP32とボタン(入力切り替え)の配線図(IO2とGNDの間にタクトスイッチを挟む形です)
```
  ESP32          
  IO2   ------   BUTTON(TACT SWITCH connect to GND)  ------ GND
```

