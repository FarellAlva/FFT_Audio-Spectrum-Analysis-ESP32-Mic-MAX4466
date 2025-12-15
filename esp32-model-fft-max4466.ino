/*
 * TUGAS 4: Visualisasi Spectrum Analysis (Peak Freq & Magnitude)
 * Board: ESP32 + MAX4466
 * Output: Serial Plotter (2 Garis Grafik)
 */

#include <Arduino.h>
#include <arduinoFFT.h>

// --- 1. Konfigurasi ---
#define MIC_ADC_PIN 34
#define SAMPLE_BUFFER_SIZE 512
#define SAMPLE_RATE 8000

// --- Variabel Global ---
unsigned long sampling_period_us;
unsigned long last_sample_micros;

double vReal[SAMPLE_BUFFER_SIZE];
double vImag[SAMPLE_BUFFER_SIZE];
int16_t wave[SAMPLE_BUFFER_SIZE];

// Inisialisasi FFT
arduinoFFT FFT = arduinoFFT(vReal, vImag, SAMPLE_BUFFER_SIZE, SAMPLE_RATE);

// =================================================================
// SETUP
// =================================================================
void setup() {
  // 4. Kirimkan data ke PC via serial (Baudrate tinggi agar lancar)
  Serial.begin(115200); 
  
  // Memberi label pada grafik di Serial Plotter
  Serial.println("Frekuensi(Hz),Magnitudo(Scaled)"); 

  sampling_period_us = round(1000000.0 / SAMPLE_RATE);
  pinMode(MIC_ADC_PIN, INPUT);
  analogSetAttenuation(ADC_11db);
}

// =================================================================
// LOOP UTAMA
// =================================================================
void loop() {
  // 2. Baca sinyal menggunakan ADC + Sampling
  getWave();

  // 3. Terapkan FFT
  performFFT();

  // 5. Visualisasikan data (Peak Freq & Magnitudo)
  plotPeakAndMagnitude();
}

// =================================================================
// FUNGSI - FUNGSI
// =================================================================

void getWave() {
  last_sample_micros = micros();
  for (int i = 0; i < SAMPLE_BUFFER_SIZE; i++) {
    while ((micros() - last_sample_micros) < sampling_period_us) { /* Wait */ }
    last_sample_micros += sampling_period_us;
    wave[i] = analogRead(MIC_ADC_PIN);
  }
}

void performFFT() {
  // DC Removal
  long signalSum = 0;
  for (int i = 0; i < SAMPLE_BUFFER_SIZE; i++) signalSum += wave[i];
  double dcOffset = signalSum / SAMPLE_BUFFER_SIZE;

  // Isi Buffer
  for (int i = 0; i < SAMPLE_BUFFER_SIZE; i++) {
    vReal[i] = (double)(wave[i] - dcOffset);
    vImag[i] = 0;
  }

  // Komputasi FFT
  FFT.Windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.Compute(FFT_FORWARD);
  FFT.ComplexToMagnitude();
}

void plotPeakAndMagnitude() {
  // A. Cari Peak Frequency (Nada Dominan)
  double peakFreq = FFT.MajorPeak();

  // B. Cari Peak Magnitude (Kekerasan Suara di frekuensi tsb)
  double peakMag = 0;
  // Cari manual nilai tertinggi di array vReal
  // Kita mulai dari index 2 untuk hindari noise frekuensi rendah
  for (int i = 2; i < (SAMPLE_BUFFER_SIZE / 2); i++) {
    if (vReal[i] > peakMag) {
      peakMag = vReal[i];
    }
  }

  // Filter Noise: Jika suara terlalu pelan, anggap frekuensi 0
  if (peakMag < 500) {
    peakFreq = 0;
    peakMag = 0;
  }

  // --- KIRIM KE SERIAL PLOTTER ---
  
  // 1. Plot Frekuensi (Biru)
  Serial.print(peakFreq); 
  Serial.print(","); 
  
  // 2. Plot Magnitudo (Merah/Oranye)
  // Kita bagi 50 agar skalanya mirip dengan Frekuensi (agar muat di satu layar)
  // Karena Magnitudo bisa sampai puluhan ribu, sedangkan Freq cuma max 4000.
  Serial.println(peakMag / 50.0); 
  
  delay(20); // Delay stabilitas
}