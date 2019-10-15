#include <Wire.h>
#include <ArduCAM.h>
#include <SPI.h>
#include <Bounce2.h>

#define DEBUG 0
#define DEBUG_PRINT_GREYSCALE 0

#define EXPOSURE (2 << 10)

#define CAM_CS_PIN 9
#define SAVE_REF_PIN 8
#define RXLED 17
#define INDICATOR_LED 5

const int IMG_BUFFER_H = 240;
const int IMG_BUFFER_W = 320;

#define DOWNSAMPLE_FACTOR 16

#define COMPARISON_TOLERANCE 0.1
#define TOLERANCE_PXL 1

#define REF_IMG_H (IMG_BUFFER_H / DOWNSAMPLE_FACTOR)
#define REF_IMG_W (IMG_BUFFER_W / DOWNSAMPLE_FACTOR)

#define DEBUG_PRINT(X) ((DEBUG ? Serial.print(X) : 0))
#define DEBUG_PRINTLN(X) ((DEBUG ? Serial.println(X) : 0))

byte REFERNCE_IMG[REF_IMG_H][REF_IMG_W] = {};
uint16_t DOWNSAMP_IMG[REF_IMG_H][REF_IMG_W] = {};

Bounce debouncer = Bounce();
ArduCAM arduCAM(OV2640, CAM_CS_PIN);

bool has_reference_img = false;
bool save_img_as_ref = false;

struct RGB
{
  byte r;
  byte g;
  byte b;
};

uint8_t read_fifo_to_buffer_and_downsample(ArduCAM &cam);
void initialize_clpd(ArduCAM &cam);
void loop_until_spi_ready(ArduCAM &cam);
void detect_camera(ArduCAM &cam);
void to_rgb(byte vh, byte vl, RGB &rgb);
void clear_img_buf();
void blink_rxlex();
void set_exposure(ArduCAM &cam, int32_t exp_val);
bool new_image_different_from_ref();

// the setup routine runs once when you press reset:
void setup()
{
  pinMode(RXLED, OUTPUT);
  pinMode(INDICATOR_LED, OUTPUT);
  pinMode(SAVE_REF_PIN, INPUT_PULLUP);
  debouncer.attach(SAVE_REF_PIN);
  debouncer.interval(5); // interval in ms

  uint8_t vid, pid;
  uint8_t temp;

  // Initialize I2C
  Wire.begin();
  // Start the Serial port with baud rate 9600
  Serial.begin(115200);

  // Set ChipSelect to High, signaling the camera to be active
  pinMode(CAM_CS_PIN, OUTPUT);
  digitalWrite(CAM_CS_PIN, HIGH);

  // Initialize SPI
  SPI.begin();

  // Check that camera is present and works
  initialize_clpd(arduCAM);
  loop_until_spi_ready(arduCAM);
  detect_camera(arduCAM);

  // Initialize to export RGB565 BMP format
  // Resolution is fixed to 320x240x2
  arduCAM.set_format(BMP);
  arduCAM.OV2640_set_Light_Mode(Office);
  arduCAM.OV2640_set_Brightness(Brightness2);
  arduCAM.OV2640_set_Contrast(Contrast_1);

  arduCAM.InitCAM();
  delay(1000);
  arduCAM.clear_fifo_flag();

  set_exposure(arduCAM, EXPOSURE);
}

void set_exposure(ArduCAM &cam, int32_t exp_val)
{
  if (exp_val == -1)
    return;

  uint8_t R13, R4, R10, R45;

  cam.rdSensorReg8_8(0x13, &R13);
  cam.rdSensorReg8_8(0x04, &R4);
  cam.rdSensorReg8_8(0x10, &R10);
  cam.rdSensorReg8_8(0x45, &R45);
  DEBUG_PRINT("Register 4: ");
  DEBUG_PRINTLN(R4, BIN);
  DEBUG_PRINT("Register 10: ");
  DEBUG_PRINT(R10, BIN);
  DEBUG_PRINT("Register 45: ");
  DEBUG_PRINT(R45, BIN);
  DEBUG_PRINT("Register 13: ");
  DEBUG_PRINT(R13, BIN);

  uint16_t a = round(exp_val / 53.39316);

  uint16_t r4shift = 14;
  uint16_t r10leftshift = 6;
  uint16_t r10rightshift = 8;
  uint16_t r45leftshift = 10;
  uint16_t b = a << r4shift;
  uint8_t r4 = b >> r4shift;                          //Extracts bits 0 and 1
  uint8_t r10 = (a << r10leftshift) >> r10rightshift; // Extracts bits 2-9
  uint8_t r45 = a >> r45leftshift;                    // Extracts bits 10-15

  for (int i = 0; i < 2; i++)
  {
    bitWrite(R4, i, bitRead(r4, i));
  }

  for (int i = 0; i < 8; i++)
  {
    bitWrite(R10, i, bitRead(r10, i));
  }

  for (int i = 0; i < 6; i++)
  {
    bitWrite(R45, i, bitRead(r45, i));
  }

  bitClear(R13, 0);
  bitClear(R13, 2);

  cam.wrSensorReg8_8(0x13, (int)R13);
  cam.wrSensorReg8_8(0x04, (int)R4);
  cam.wrSensorReg8_8(0x10, (int)R10);
  cam.wrSensorReg8_8(0x45, (int)R45);

  cam.rdSensorReg8_8(0x13, &R13);
  cam.rdSensorReg8_8(0x04, &R4);
  cam.rdSensorReg8_8(0x10, &R10);
  cam.rdSensorReg8_8(0x45, &R45);

  DEBUG_PRINT("Register 4: ");
  DEBUG_PRINTLN(R4, BIN);
  DEBUG_PRINT("Register 10: ");
  DEBUG_PRINT(R10, BIN);
  DEBUG_PRINT("Register 45: ");
  DEBUG_PRINT(R45, BIN);
  DEBUG_PRINT("Register 13: ");
  DEBUG_PRINT(R13, BIN);
}

// the loop routine runs over and over again forever:
void loop()
{
  bool is_different = false;
  debouncer.update();

  if (debouncer.read() == LOW)
  {
    save_img_as_ref = true;
    blink_rxlex();
    return;
  }

  uint8_t temp, temp_last;
  int mode = 1;
  temp = 0xff;

  DEBUG_PRINTLN(F(">>> Start single capture. <<<"));

  arduCAM.flush_fifo();
  arduCAM.clear_fifo_flag();

  arduCAM.start_capture();
  while (!arduCAM.get_bit(ARDUCHIP_TRIG, CAP_DONE_MASK))
  {
    delay(20);
  }

  // Read image data into BUFFERED_IMG
  read_fifo_to_buffer_and_downsample(arduCAM);

#if DEBUG_PRINT_GREYSCALE == 1
  Serial.println("**START");
  for (int i = 0; i < REF_IMG_H; ++i)
  {
    for (int j = 0; j < REF_IMG_W; ++j)
    {
      if (DOWNSAMP_IMG[i][j] < 10)
        Serial.print("   ");
      else if (DOWNSAMP_IMG[i][j] < 100)
        Serial.print("  ");
      else
        Serial.print(" ");
      DEBUG_GREYSCALE(DOWNSAMP_IMG[i][j]);
    }
    Serial.println(" ");
  }
  Serial.println("**END");

  return;
#endif

  if (!has_reference_img || save_img_as_ref)
  {
    delay(300);
    DEBUG_PRINTLN(F("Reference: "));
    // Copy into reference
    for (int i = 0; i < REF_IMG_H; ++i)
    {
      for (int j = 0; j < REF_IMG_W; ++j)
      {
        REFERNCE_IMG[i][j] = DOWNSAMP_IMG[i][j];
        if (REFERNCE_IMG[i][j] < 10)
          DEBUG_PRINT("   ");
        else if (REFERNCE_IMG[i][j] < 100)
          DEBUG_PRINT("  ");
        else
          DEBUG_PRINT(" ");
        DEBUG_PRINT(REFERNCE_IMG[i][j]);
      }
      DEBUG_PRINTLN(" ");
    }
    has_reference_img = true;
    save_img_as_ref = false;

    blink_rxlex();
  }
  else
  {
    // Compare to REF_IMG
    is_different = new_image_different_from_ref();
    if (is_different)
    {
      DEBUG_PRINTLN("Different");
      digitalWrite(RXLED, LOW);
    }
    else
    {
      DEBUG_PRINTLN("Same");
      digitalWrite(RXLED, HIGH);
    }
  }
}

uint8_t read_fifo_to_buffer_and_downsample(ArduCAM &cam)
{
  //DEBUG_PRINTLN(F("ACK CMD CAM Capture Done. END"));
  delay(50);
  uint8_t temp, temp_last;
  uint32_t length = 0;
  length = cam.read_fifo_length();
  if (length >= MAX_FIFO_SIZE)
  {
    DEBUG_PRINTLN(F("ACK CMD Over size. END"));
    cam.clear_fifo_flag();
    return 0;
  }
  if (length == 0) //0 kb
  {
    DEBUG_PRINTLN(F("ACK CMD Size is 0. END"));
    cam.clear_fifo_flag();
    return 0;
  }
  cam.CS_LOW();
  cam.set_fifo_burst(); //Set fifo burst mode

  char VH, VL;
  int base_pixel_x;
  int base_pixel_y;
  float avg_pixels = 0.0;
  unsigned long time1;
  unsigned long time2;

  clear_img_buf();

  int checked = 0;
  // Height first, therefore this is read
  for (int i = 0; i < IMG_BUFFER_H; ++i)
  {
    for (int j = 0; j < IMG_BUFFER_W; ++j)
    {
      VL = SPI.transfer(0x00);
      VH = SPI.transfer(0x00);

      RGB rgb = {0,
                 0,
                 0};

      to_rgb(VH, VL, rgb);

      base_pixel_x = i / DOWNSAMPLE_FACTOR;
      base_pixel_y = j / DOWNSAMPLE_FACTOR;

      DOWNSAMP_IMG[base_pixel_x][base_pixel_y] += rgb_to_intensity(rgb);

      ++checked;
    }
  }

  for (byte i = 0; i < REF_IMG_H; ++i)
  {
    for (byte j = 0; j < REF_IMG_W; ++j)
    {
      DOWNSAMP_IMG[i][j] = (DOWNSAMP_IMG[i][j] >> 8);
    }
  }
  cam.CS_HIGH();
  //Clear the capture done flag
  cam.clear_fifo_flag();
  return 1;
}

void initialize_clpd(ArduCAM &cam)
{
  //Reset the CPLD, i.e., the main controller of the camera board
  cam.write_reg(0x07, 0x80);
  delay(100);
  cam.write_reg(0x07, 0x00);
  delay(100);
}

void loop_until_spi_ready(ArduCAM &cam)
{
  uint8_t temp;
  while (1)
  {
    cam.write_reg(ARDUCHIP_TEST1, 0x55);
    temp = cam.read_reg(ARDUCHIP_TEST1);
    if (temp != 0x55)
    {
      DEBUG_PRINTLN(F(">>> SPI interface Error! <<<"));
      delay(1000);
      continue;
    }
    else
    {
      DEBUG_PRINTLN(F(">>> SPI interface OK! <<<"));
      break;
    }
  }
}

void detect_camera(ArduCAM &cam)
{
  uint8_t vid, pid;
  while (1)
  {
    cam.wrSensorReg8_8(0xff, 0x01);
    cam.rdSensorReg8_8(OV2640_CHIPID_HIGH, &vid);
    cam.rdSensorReg8_8(OV2640_CHIPID_LOW, &pid);
    if ((vid != 0x26) && ((pid != 0x41) || (pid != 0x42)))
    {
      DEBUG_PRINTLN(F(">>> Can't find OV2640 module! <<<"));
      delay(1000);
      continue;
    }
    else
    {
      DEBUG_PRINTLN(F(">>> Camera detected. <<<"));
      break;
    }
  }
}

void to_rgb(char vh, char vl, RGB &rgb)
{
  uint16_t data = ((vh << 8) | vl);
  // According to official
  rgb.r = ((((data >> 11) & 0x1F) * 527) + 23) >> 6;
  rgb.g = ((((data >> 5) & 0x3F) * 259) + 33) >> 6;
  rgb.b = (((data & 0x1F) * 527) + 23) >> 6;
}

byte rgb_to_intensity(RGB &rgb)
{
  // http://poynton.ca/notes/colour_and_gamma/ColorFAQ.html#RTFToC9
  // This is too slow
  // return (byte)((int) (0.2989 * rgb.r)) + ((int)(0.5870 * rgb.g)) + ((int)(0.1140 * rgb.b));

  // This is too noisy. *170 >> 9 is almost equivalent to * 1/3
  // uint32_t a = (rgb.r + rgb.g + rgb.b) * 170;
  // return (a >> 9);

  return (byte)(rgb.r >> 2) + (rgb.g >> 1) + (rgb.b >> 3);
}

bool new_image_different_from_ref()
{
  int tol_value = (int)(255 * COMPARISON_TOLERANCE);
  DEBUG_PRINT("Comparison with tolerance: ");
  DEBUG_PRINTLN(tol_value);
  int i = 0, j = 0;
  int count_diff = 0;
  int cur_difference = 0;

  for (int i = 0; i < REF_IMG_H; ++i)
  {
    for (int j = 0; j < REF_IMG_W; ++j)
    {
      if (REFERNCE_IMG[i][j] < 10)
        DEBUG_PRINT("   ");
      else if (REFERNCE_IMG[i][j] < 100)
        DEBUG_PRINT("  ");
      else
        DEBUG_PRINT(" ");
      DEBUG_PRINT(REFERNCE_IMG[i][j]);
    }
    DEBUG_PRINTLN(" ");
  }

  DEBUG_PRINTLN("======= ^ ref ^ ======== v new v =======");
  for (int i = 0; i < REF_IMG_H; ++i)
  {
    for (int j = 0; j < REF_IMG_W; ++j)
    {
      if (DOWNSAMP_IMG[i][j] < 10)
        DEBUG_PRINT("   ");
      else if (DOWNSAMP_IMG[i][j] < 100)
        DEBUG_PRINT("  ");
      else
        DEBUG_PRINT(" ");
      DEBUG_PRINT(DOWNSAMP_IMG[i][j]);
    }
    DEBUG_PRINTLN(" ");
  }

  for (i = 1; i < REF_IMG_H - 1; ++i)
  {
    for (j = 1; j < REF_IMG_W - 1; ++j)
    {
      cur_difference = (int)abs(REFERNCE_IMG[i][j] - DOWNSAMP_IMG[i][j]);
      if (cur_difference > tol_value)
      {
        if (++count_diff > TOLERANCE_PXL)
        {
          return true;
        }
      }
    }
  }
  DEBUG_PRINTLN("");
  return false;
}

void clear_img_buf()
{
  for (int i = 0; i < REF_IMG_H; ++i)
  {
    for (int j = 0; j < REF_IMG_W; ++j)
    {
      DOWNSAMP_IMG[i][j] = 0;
    }
  }
}

void blink_rxlex()
{
  for (int i = 0; i < 4; ++i)
  {
    digitalWrite(RXLED, LOW);
    delay(300);
    digitalWrite(RXLED, HIGH);
    delay(300);
  }
  delay(3000);
}
