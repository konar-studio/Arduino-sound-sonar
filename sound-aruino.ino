
#include <ArduinoJson.h>

// FHT, http://wiki.openmusiclabs.com/wiki/ArduinoFHT
#define LOG_OUT 1  // use the log output function
#define LIN_OUT8 1 // use the linear byte output function
#define FHT_N 64 // set to 256 point fht
#include <FHT.h>   // include the library


// consts
#define AmpMax (1024 / 2)
#define MicSamples (1024 * 2) // Three of these time-weightings have been internationally standardised, 'S' (1 s) originally called Slow, 'F' (125 ms) originally called Fast and 'I' (35 ms) originally called Impulse.

// modes
#define Use3 .3    // use 3.3 voltage. the 5v voltage from usb is not regulated. this is much more stable.

#define FreqLog // use log scale for FHT frequencies
#ifdef FreqLog
#define FreqOutData fht_log_out
#define FreqGainFactorBits 0
#else
#define FreqOutData fht_lin_out8
#define FreqGainFactorBits 3
#endif
#define FreqSerialBinary

#define VolumeGainFactorBits 0

// macros
// http://yaab-arduino.blogspot.co.il/2015/02/fast-sampling-from-analog-input.html
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))

// Emulate Serial1 on pins 7/6 if not present
#ifndef HAVE_HWSERIAL1
#include "SoftwareSerial.h"
SoftwareSerial Serial1(6, 7); // RX, TX
#endif

byte Pins[] ={ A0, A1, A2, A3 };

int wsConected = 2; // from ESPI 01
int PowerLED = 3;
void setup()
{
    delay(3000);
    Serial1.begin(115200);    // ESP01 port used for debugging
    Serial.begin(9600); // Debuger
    // while (!Serial1); // Wait untilSerial is ready - Leonardo
    // digital Pin for connected devices
    pinMode(wsConected, INPUT);
    pinMode(PowerLED, OUTPUT);

}

void loop()
{
    if (digitalRead(wsConected) == HIGH) {
        digitalWrite(PowerLED, HIGH);
        for (int x = 0; x < 4; x++) {
            SignalProccecing(x);
            delay(10);
        }
    }
    else if (digitalRead(wsConected) == LOW)
    {
        digitalWrite(PowerLED, LOW);
    }
    delay(100);
}

void SignalProccecing(int inputPin)
{
    long soundVolAvg = 0, soundVolMax = 0, soundVolRMS = 0;
    long t0 = micros();


    for (int i = 0; i < MicSamples; i++)
    {

        int k = analogRead(Pins[inputPin]);
        int amp = abs(k - AmpMax);
        amp <<= VolumeGainFactorBits;
        soundVolMax = max(soundVolMax, amp);
        soundVolAvg += amp;
        soundVolRMS += ((long)amp*amp);
    }
    soundVolAvg /= MicSamples;
    soundVolRMS /= MicSamples;
    float soundVolRMSflt = sqrt(soundVolRMS);
    float dB = 20.0*log10(soundVolRMSflt/AmpMax);


    StaticJsonDocument<500> msg;


    msg["max"] = 100 * soundVolMax / AmpMax;;
    msg["avg"] = 100 * soundVolAvg / AmpMax;;
    msg["rms"] = 10 * soundVolRMSflt / 7; // RMS to estimate peak (RMS is 0.7 of the peak in sin);
    msg["rmsflt"] = 100 * soundVolRMSflt / AmpMax;;
    msg["db"] = String(dB, 3);
    msg["mic"] = inputPin;

    // calculate frequencies in the signal and print to serial
    for (int i = 0; i < FHT_N; i++)
    { // save 256 samples

        int k = analogRead(Pins[inputPin]);
        k -= 0x0200; // form into a signed int
        k <<= 6;     // form into a 16b signed int
        k <<= FreqGainFactorBits;
        fht_input[i] = k; // put real data into bins
    }

    fht_window();  // window the data for better frequency response
    fht_reorder(); // reorder the data before doing the fht
    fht_run();     // process the data in the fht
    #ifdef FreqLog
    fht_mag_log();
    #else
    fht_mag_lin8(); // take the output of the fht
    #endif

    long dt = micros() - t0;
    msg["t0"] = t0;
    msg["dt"] = dt;

    long s_rate = FHT_N * 1000000l / dt;
    msg["s_rate"] = s_rate;

    for (int i = 0; i < FHT_N / 2; i++)
    {
        msg["bin"][i] = FreqOutData[i];
    }


    serializeJson(msg, Serial1);
    Serial1.println("\r");
    // Serial.println("\r");
    // serializeJson(msg, Serial);
}


