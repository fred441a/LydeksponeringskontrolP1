// bluetooth, config, discover and audio
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gap_bt_api.h"
#include "esp_a2dp_api.h"
#include "freertos/queue.h"

// the audio DAC and amp configuration.
#include "driver/i2s.h"

//globals
const int avrlength = 200;

int AmplitydeArray[avrlength];
int Amplitude;
int indx;



float Avarage(int* samples /*skal være et array med avrlength som længde*/) {
  float gennem = 0;
  for (int i = 0; i <= avrlength - 1; i++) {
    gennem += (float) samples[i];
  }
  return gennem / avrlength;
}

void addRolling(int sample) {
  AmplitydeArray[indx] = sample;
  indx++;
  if (indx >= avrlength) {
    indx = 0;
  }

}


//Denne funktion køres hver gang et sample bliver sendt over bluetooth
void bt_data_cb(const uint8_t *data, uint32_t len) {
  // number of 16 bit samples
  int n = len / 2;

  // point to a 16bit sample
  int16_t* data16 = (int16_t*)data;

  // create a variable (potentially processed) that we'll pass via I2S
  int16_t fy;

  // Records number of bytes written via I2S
  size_t i2s_bytes_write = 0;

  for (int i = 0; i < n; i++) {
    // put the current sample in fy
    fy = *data16;

    //making this value larger will decrease the volume(Very simple DSP!).
    fy /= 1;

    // write data to I2S buffer
    i2s_write(I2S_NUM_0, &fy, 2, &i2s_bytes_write,  100 );

    //move to next memory address housing 16 bit data
    data16++;
  }
  //Serial.print("sample:");
  //Serial.println(abs((int)fy));
  //Serial.print(",Volume:");
  //Serial.println(Avarage(AmplitydeArray));
  //addRolling(abs((int)fy));
}


void I2SSetup() {
  // i2s configuration
  static const i2s_config_t i2s_config = {
    .mode = static_cast<i2s_mode_t>(I2S_MODE_MASTER | I2S_MODE_TX),
    .sample_rate = 44100,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_24BIT,
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
    .communication_format = static_cast<i2s_comm_format_t>(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB),
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1, // default interrupt priority
    .dma_buf_count = 8,
    .dma_buf_len = 64,
    .use_apll = false,
    .tx_desc_auto_clear = true
  };

  // i2s pinout
  static const i2s_pin_config_t pin_config = {
    .bck_io_num = 26,//26
    .ws_io_num = 27,
    .data_out_num = 25,, //
    .data_in_num = I2S_PIN_NO_CHANGE
  };

  // now configure i2s with constructed pinout and config
  i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_NUM_0, &pin_config);
  //i2s_set_dac_mode(I2S_DAC_CHANNEL_LEFT_EN);
  i2s_set_clk(I2S_NUM_0, 44100, I2S_BITS_PER_SAMPLE_16BIT, I2S_CHANNEL_STEREO);
  i2s_set_sample_rates(I2S_NUM_0, 44100);

  REG_WRITE(PIN_CTRL, 0b111111110000);
  PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO0_U, FUNC_GPIO0_CLK_OUT1);

}

void BluetoothSetup() {

  // set up bluetooth classic via bluedroid
  btStart();
  esp_bluedroid_init();
  esp_bluedroid_enable();


  // set up device name
  const char *dev_name = "Bluetooth_Test";
  esp_bt_dev_set_device_name(dev_name);

  // initialize A2DP sink and set the data callback(A2DP is bluetooth audio)
  esp_a2d_sink_register_data_callback(bt_data_cb);
  esp_a2d_sink_init();

  // set discoverable and connectable mode, wait to be connected
  esp_bt_gap_set_scan_mode(ESP_BT_SCAN_MODE_CONNECTABLE_DISCOVERABLE);



}


void setup() {
  Serial.begin(115200);
  BluetoothSetup();
  I2SSetup();

}

void loop() {
  delay(1000);
}
