/**
 ******************************************************************************
 * @file    X_NUCLEO_NFC03A1_HelloWorld.ino
 * @author  AST
 * @version V1.0.0
 * @date    6 December 2017
 * @brief   Arduino test application for the STMicrolectronics X-NUCLEO-NFC03A1
 *          NFC reader/writer expansion board.
 *          This application makes use of C++ classes obtained from the C
 *          components' drivers.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2017 STMicroelectronics</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without
 *modification, are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright
 *notice, this list of conditions and the following disclaimer in the
 *documentation and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 *LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
//#include "lib_NDEF_Text.h"
#include "lib_NDEF.h"
#include "lib_iso15693pcd.h"
#include "lib_nfctype5pcd.h"

#define SerialPort Serial

/* Exported define -----------------------------------------------------------*/
#define BULK_MAX_PACKET_SIZE 0x00000040

/* Regarding board antenna (and matching) appropriate
value may be modified to optimized RF performances */
/* Analogue configuration register
 ARConfigB	bits  7:4	MOD_INDEX	Modulation index to modulator
                      3:0	RX_AMP_GAIN	Defines receiver amplifier gain
For type A you can also adjust the Timer Window
*/

/******************  PICC  ******************/
// /* ISO14443A */
// #define PICC_TYPEA_ACConfigA 0x27 /* backscaterring */

// /* ISO14443B */
// #define PICC_TYPEB_ARConfigD 0x0E /* card demodulation gain */
// #define PICC_TYPEB_ACConfigA 0x17 /* backscaterring */

// /* Felica */
// #define PICC_TYPEF_ACConfigA 0x17 /* backscaterring */

/* Private variables ---------------------------------------------------------*/

// /* TT1 (PCD only)*/
// uint8_t TT1Tag[NFCT1_MAX_TAGMEMORY];

// /* TT2 */
// uint8_t TT2Tag[NFCT2_MAX_TAGMEMORY];

// /* TT3 */
// uint8_t TT3Tag[NFCT3_MAX_TAGMEMORY];
// uint8_t *TT3AttribInfo = TT3Tag, *TT3NDEFfile =
// &TT3Tag[NFCT3_ATTRIB_INFO_SIZE];

// /* TT4 */
// uint8_t CardCCfile[NFCT4_MAX_CCMEMORY];
// uint8_t CardNDEFfileT4A[NFCT4_MAX_NDEFMEMORY];
// uint8_t CardNDEFfileT4B[NFCT4_MAX_NDEFMEMORY];

/* TT5 (PCD only)*/
uint8_t TT5Tag[NFCT5_MAX_TAGMEMORY];

extern DeviceMode_t devicemode;

int8_t TagType = TRACK_NOTHING;
bool TagDetected = false;
uint8_t status = ERRORCODE_GENERIC;
// char dataOut[512];

sRecordInfo_t info;
sRecordInfo_t info2;

#define X_NUCLEO_NFC03A1_LED1 7
#define X_NUCLEO_NFC03A1_LED2 6
#define X_NUCLEO_NFC03A1_LED3 5
#define X_NUCLEO_NFC03A1_LED4 4

void setup() {
  // 95HF HW Init
  ConfigManager_HWInit();

  // LED1
  pinMode(X_NUCLEO_NFC03A1_LED1, OUTPUT);

  // LED2
  pinMode(X_NUCLEO_NFC03A1_LED2, OUTPUT);

  // LED3
  pinMode(X_NUCLEO_NFC03A1_LED3, OUTPUT);

  // LED4
  pinMode(X_NUCLEO_NFC03A1_LED4, OUTPUT);

  // Configure USB serial interface
  SerialPort.begin(115200);

  SerialPort.println("Please bring an NFC");

  digitalWrite(X_NUCLEO_NFC03A1_LED1, HIGH);
}

/* Loop ----------------------------------------------------------------------*/

void loop() {
  devicemode = PCD;
  TagType = ConfigManager_TagHunting(TRACK_NFCTYPE5);

  switch (TagType) {
  case TRACK_NFCTYPE5: {
    TagDetected = true;
    SerialPort.println("Trouve");

    digitalWrite(X_NUCLEO_NFC03A1_LED2, HIGH);
  } break;

  default: {
    TagDetected = false;
    digitalWrite(X_NUCLEO_NFC03A1_LED3, LOW);
  } break;
  }
  digitalWrite(X_NUCLEO_NFC03A1_LED2, LOW);
  delay(500);
  digitalWrite(X_NUCLEO_NFC03A1_LED2, HIGH);
  if (TagDetected == true) {
    TagDetected = false;

    status = PCDNFCT5_ReadNDEF();
    if (status == PCDNFCT5_OK) {
      Serial.println("Debut");
      for (size_t i = 0; i < NFCT5_MAX_TAGMEMORY; i++) {
        SerialPort.println(TT5Tag[i]);
      }
      Serial.println("Fin");
    } else if (status == PCDNFCT5_ERROR_LOCKED) {
      SerialPort.println("Tag bloque");
    } else if (status == PCDNFCT5_ERROR_NOT_FORMATED) {
      SerialPort.println("Not formated");
    } else if (status == PCDNFCT5_ERROR_MEMORY_INTERNAL) {
      SerialPort.println("Memory");
    } else {
      SerialPort.println("Erreur");
    }

    digitalWrite(X_NUCLEO_NFC03A1_LED3, HIGH);
  }
}
