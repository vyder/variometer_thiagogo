#include <Wire.h>                   // Biblioteca de interface I2C

#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>                     // Biblioteca Piezo Tone

#define tone_out1 8
#define tone_out2 9

Adafruit_BMP280 sensor_bmp;

short speaker_pin1 = 8;                //arduino speaker output -
short speaker_pin2 = 9;                //arduino speaker output +

float vario_down = -1.1;               // Definir a redução
float vario_up;
float alt[51];
float tim[51];
float beep;
float Beep_period;

float vario = 0;

float Altitude = 0;
float Temperature = 0;

unsigned char samples = 10;
unsigned char maxsamples = 50;

// relogio
unsigned char segundo = 0, minuto = 0, hora = 0;
unsigned long UtlTime;
unsigned long lastMillis;

boolean thermalling = false;

// Saudações de áudio
void play_welcome_beep() {
    for (int aa=100; aa<=800; aa=aa+100) {
        tone(tone_out1,aa,200);
        tone(tone_out2,aa+3,200);
        delay(50);
    }
}

void setup() {
    // Inicializa i2c
    Wire.begin();

    pinMode(4, INPUT);
    digitalWrite(4, HIGH);

    pinMode(tone_out1, OUTPUT);  // Dinâmica pin8 de saída -
    pinMode(tone_out2, OUTPUT);  // Speaker pin9 saída +

    // Sensibilidade do sensor de pressão
    sensor_bmp.begin();

    play_welcome_beep();
}

void loop(void) {

    float tempo = millis();
    float N1 = 0;
    float N2 = 0;
    float N3 = 0;
    float D1 = 0;
    float D2 = 0;

    Altitude = (sensor_bmp.readAltitude(1013.25));
    Temperature = (sensor_bmp.readTemperature());

    // averager
    for (int cc = 1; cc <= maxsamples; cc++) {
        int prev = cc - 1
        alt[prev] = alt[cc];
        tim[prev] = tim[cc];
    }

    alt[maxsamples] = Altitude;
    tim[maxsamples] = tempo;

    float stime = tim[maxsamples - samples];

    for(int cc = (maxsamples-samples); cc < maxsamples; cc++) {
        N1 += (tim[cc] - stime) * alt[cc];
        N2 += (tim[cc] - stime);
        N3 += alt[cc];
        D1 += (tim[cc] - stime) * (tim[cc] - stime);
        D2 += (tim[cc] - stime);
    };

    // Cálculo de som
    vario = 1000 * ((samples*N1) - N2*N3) / (samples*D1 - D2*D2);

    if ((tempo - beep) > Beep_period) {
        beep = tempo;

        if ((vario > vario_up) && (vario < 15)) {
            float scaledVario = (vario * 5)
            Beep_period = 350 - scaledVario;

            float inverseVario = 300 - scaledVario;

            // Som em ascensão
            tone(tone_out1, (1000 + (100 * vario)), inverseVario);
            tone(tone_out2, (1003 + (100 * vario)), inverseVario);

            thermalling = true;

        } else if ((vario < 0 ) && (thermalling == true)) {
            thermalling = false;
            // tone_out2.play(200, 800); Predpotok // Som (é opcional)

        // Som a afundar
        } else if (vario< vario_down) {
            Beep_period = 200;
            tone(tone_out1, (300 - vario), 340);
            tone(tone_out2, (303 - vario), 340);
            thermalling = false;
        }
    }

    // relogio
    if ((millis() - UtlTime) < 0) {
        UtlTime = millis();

    } else {
        segundo = int((millis() - UtlTime) / 1000);
    }

    if (segundo > 59) {
        segundo = 0;
        minuto++;
        UtlTime = millis();

        if (minuto > 59) {
            hora++;
            minuto = 0;
        }
    }
}
