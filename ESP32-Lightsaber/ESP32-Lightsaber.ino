//---------------------------------------------------------------------------------------------------------------------------
// 
// Title: SD Card Wav Player
//
// Description:
//    Simple example to demonstrate the fundementals of playing WAV files (digitised sound) from an SD Card via the I2S 
//    interface of the ESP32. Plays WAV file from SD card. To keep this simple the WAV must be stereo and 16bit samples. 
//    The Samples Per second can be anything. On the SD Card the wav file must be in root and called wavfile.wav
//    Libraries are available to play WAV's on ESP32, this code does not use these so that we can see what is happening.
//    This is part 3 in a tutorial series on using I2S on ESP32. See the accompanying web page (which will also include
//    a tutorial video).
//
// Boring copyright/usage information:
//    (c) XTronical, www.xtronical.com
//    Use as you wish for personal or monatary gain, or to rule the world (if that sort of thing spins your bottle)
//    However you use it, no warrenty is provided etc. etc. It is not listed as fit for any purpose you perceive
//    It may damage your house, steal your lover, drink your beers and more.
//
//    http://www.xtronical.com/i2s-ep3
//
//---------------------------------------------------------------------------------------------------------------------------

//------------------------------------------------------------ Includes -----------------------------------------------------
    #include "SD.h"                         // SD Card library, usually part of the standard install
    #include "driver/i2s.h"                 // Library of I2S routines, comes with ESP32 standard install
    #include "Wire.h"                       // Library for wire inputs/outputs
    #include <Adafruit_MPU6050.h>           // Library for MPU-6050 Accelerometer
    #include <Adafruit_Sensor.h>            // Library for Sensors

//------------------------------------------------------------ Buttons ------------------------------------------------------
    #define PUSH_BUTTON   32                // Momentary Push Button
    #define PUSH_BUTTON_2 26          // Momentary Push Button 2
 
//------------------------------------------------------------ SD Card ------------------------------------------------------
    #define SD_CS         5          // SD Card chip select
   
//------------------------------------------------------------ I2S ----------------------------------------------------------
          #define I2S_DOUT      33          // i2S Data out oin
          #define I2S_BCLK      27          // Bit clock
          #define I2S_LRC       25          // Left/Right clock, also known as Frame clock or word select
          #define I2S_NUM       0           // i2s port number

//------------------------------------------------------------ Wav File reading ---------------------------------------------
          #define NUM_BYTES_TO_READ_FROM_FILE 1024    // How many bytes to read from wav file at a time

//------------------------------------------------------------ Sound Buffer -------------------------------------------------
          #define NUM_BYTES_TO_USE_AS_BUFFER 256      // How many bytes to use as the audio buffer for playing audio

//------------------------------------------------------------ MPU-6050 accelerometer ---------------------------------------
//          const int MPU_ADDR = 0x68;        // i2c address of the MPU-6050
//          int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;
          Adafruit_MPU6050 mpu;


//------------------------------------------------------------ Built-in LED -------------------------------------------------
          #define INTERNAL_LED  2

//------------------------------------------------------------ I2S configuration --------------------------------------------

      static const i2s_config_t i2s_config = 
      {
          .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
          .sample_rate = 48000,                                 // Note, this will be changed later
          .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
          .channel_format = I2S_CHANNEL_FMT_ONLY_RIGHT,
          .communication_format = (i2s_comm_format_t)(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB),
          .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,             // high interrupt priority
          .dma_buf_count = 8,                                   // 8 buffers
          .dma_buf_len = 64,                                    // 64 bytes per buffer, so 8K of buffer space
          .use_apll=0,
          .tx_desc_auto_clear= true, 
          .fixed_mclk=-1    
      };
      
      // These are the physical wiring connections to our I2S decoder board/chip from the esp32, there are other connections
      // required for the chips mentioned at the top (but not to the ESP32), please visit the page mentioned at the top for
      // further information regarding these other connections.
      
      static const i2s_pin_config_t pin_config = 
      {
          .bck_io_num = I2S_BCLK,                           // The bit clock connection, goes to pin 27 of ESP32
          .ws_io_num = I2S_LRC,                             // Word select, also known as word select or left right clock
          .data_out_num = I2S_DOUT,                         // Data out from the ESP32, connect to DIN on 38357A
          .data_in_num = I2S_PIN_NO_CHANGE                  // we are not interested in I2S data into the ESP32
      };

      float Volume = 1;

      bool VolUp = 0;
      bool VolDown = 0;
        
      struct WavHeader_Struct
      {
          //   RIFF Section    
          char RIFFSectionID[4];      // Letters "RIFF"
          uint32_t Size;              // Size of entire file less 8
          char RiffFormat[4];         // Letters "WAVE"
          
          //   Format Section    
          char FormatSectionID[4];    // letters "fmt"
          uint32_t FormatSize;        // Size of format section less 8
          uint16_t FormatID;          // 1=uncompressed PCM
          uint16_t NumChannels;       // 1=mono,2=stereo
          uint32_t SampleRate;        // 44100, 16000, 8000 etc.
          uint32_t ByteRate;          // =SampleRate * Channels * (BitsPerSample/8)
          uint16_t BlockAlign;        // =Channels * (BitsPerSample/8)
          uint16_t BitsPerSample;     // 8,16,24 or 32
        
          // Data Section
          char DataSectionID[4];      // The letters "data"
          uint32_t DataSize;          // Size of the data that follows
      }WavHeader;

//----------------------------------------------------------- Global Variables/objects --------------------------------------
    File WavFile;                                 // Object for root of SD card directory
    static const i2s_port_t i2s_num = I2S_NUM_0;  // i2s port number    

//----------------------------------------------------------- Tasks ---------------------------------------------------------
    TaskHandle_t task1;               // task 1
    TaskHandle_t task2;               // task 2


//----------------------------------------------------------- Multi Core Task 1 ---------------------------------------------
void task_1code(void * pvParameters)
{
  for(;;) {
//    Serial.print("task1 running on core ");
//    Serial.print(xPortGetCoreID());
//    Serial.print("\n");
////    digitalWrite(INTERNAL_LED, HIGH);
////    delay(1000);
////    digitalWrite(INTERNAL_LED, LOW);
////    delay(1000);

    PlayWav();
  }
}

//----------------------------------------------------------- Multi Core Task 2 ---------------------------------------------
void task_2code(void * pvParameters)
{
  for(;;) {
//    Serial.print("task2 running on core ");
//    Serial.print(xPortGetCoreID());
//    Serial.print("\n");
////    delay(300);

    GetGyro();
  }
}

//----------------------------------------------------------- Setup ---------------------------------------------------------
void setup() {    
    Wire.begin();
    Serial.begin(115200);                               // Used for info/debug
    Serial.print("setup is running on: Core_");
    Serial.print(xPortGetCoreID());
    
    SDCardInit();
    i2s_driver_install(i2s_num, &i2s_config, 0, NULL);
    i2s_set_pin(i2s_num, &pin_config);
    // get the wav file from the SD card
    WavFile = SD.open("/wavfile.wav");                   // Open the wav file
    if(WavFile==false)
      Serial.println("Could not open 'wavfile.wav'");
    else
    {
      WavFile.read((byte *) &WavHeader,44);               // Read in the WAV header, which is first 44 bytes of the file. 
                                                          // We have to typecast to bytes for the "read" function
      DumpWAVHeader(&WavHeader);                          // Dump the header data to serial, optional!
      if(ValidWavData(&WavHeader))                        // optional if your sure the WAV file will be valid.
        i2s_set_sample_rates(i2s_num, WavHeader.SampleRate);      //set sample rate 
    }
    
    pinMode(PUSH_BUTTON, INPUT);
    pinMode(PUSH_BUTTON_2, INPUT);   
                      
    pinMode(INTERNAL_LED, OUTPUT);                                // on board LED

//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! MPU-6050 Accelerometer Manual attempt !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//    Wire.begin(21, 22, 100000);                           // sda, scl, clock speed
//    Wire.beginTransmission(MPU_ADDR);
//    Wire.write(0x6B);                                    // PWR_MGMT_1 register
//    Wire.write(0);                                       // set to zero (wakes up the MPU-6050)
//    Wire.endTransmission(true);
//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! MPU-6050 Accelerometer Manual attempt !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

//------------------------------------------------------------------- MPU-6050 Accelerometer Setup --------------------------
    if (!mpu.begin()) {
      Serial.println("Failed to find MPU6050 chip");
      while (1) {
        delay(10);
      }
    }
    Serial.println("MPU6050 Found!");

    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    Serial.print("Accelerometer range set to: ");
    switch (mpu.getAccelerometerRange()) {
      case MPU6050_RANGE_2_G:
        Serial.println("+-2G");
        break;
      case MPU6050_RANGE_4_G:
        Serial.println("+-4G");
        break;
      case MPU6050_RANGE_8_G:
        Serial.println("+-8G");
        break;
      case MPU6050_RANGE_16_G:
        Serial.println("+-16G");
        break;
    }

    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    Serial.print("Gyro range set to: ");
    switch (mpu.getGyroRange()) {
      case MPU6050_RANGE_250_DEG:
        Serial.println("+- 250 deg/s");
        break;
      case MPU6050_RANGE_500_DEG:
        Serial.println("+- 500 deg/s");
        break;
      case MPU6050_RANGE_1000_DEG:
        Serial.println("+- 1000 deg/s");
        break;
      case MPU6050_RANGE_2000_DEG:
        Serial.println("+- 2000 deg/s");
        break;
    }

    mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
    Serial.print("Filter Bandwidth set to: ");
    switch (mpu.getFilterBandwidth()) {
      case MPU6050_BAND_260_HZ:
        Serial.println("260 Hz");
        break;
      case MPU6050_BAND_184_HZ:
        Serial.println("184 Hz");
        break;
      case MPU6050_BAND_94_HZ:
        Serial.println("94 Hz");
        break;
      case MPU6050_BAND_44_HZ:
        Serial.println("44 Hz");
        break;
      case MPU6050_BAND_21_HZ:
        Serial.println("21 Hz");
        break;
      case MPU6050_BAND_10_HZ:
        Serial.println("10 Hz");
        break;
      case MPU6050_BAND_5_HZ:
        Serial.println("5 Hz");
        break;
    }

    Serial.println("");
    delay(100);

//---------------------------------------------------------------- Dual Core Tasks Setup ------------------------------------

    xTaskCreatePinnedToCore(task_1code, // Task function
                            "Task1",    // name of task
                            10000,      // Stack size of the task
                            NULL,       // parameter of the task
                            1,          // priority of the task
                            &task1,      // Task handle to keep track of created task
                            1);         // pin task to core 1
    delay(1000);
    
    xTaskCreatePinnedToCore(task_2code, // Task function
                            "Task2",    // name of task
                            10000,      // Stack size of the task
                            NULL,       // parameter of the task
                            1,          // priority of the task
                            &task2,      // Task handle to keep track of created task
                            0);         // pin task to core 1
    delay(1000);
}


//---------------------------------------------------------------- Main Loop ------------------------------------------------
void loop()
{    
//  Serial.print( " loop() is running on: Core " );
//  Serial.println( xPortGetCoreID() );
//  delay(1000);

//task_1code();
//task_2code();

//   PlayWav();                                            // Have to keep calling this to keep the wav file playing
//   GetGyro();

//    Wire.beginTransmission(MPU_ADDR);
//    Wire.write(0x3B);                            // starting with register 0x3B (ACCEL_XOUT_H)
//    Wire.endTransmission(true);
//    Wire.beginTransmission(MPU_ADDR);
//    Wire.requestFrom(MPU_ADDR, 14, true);        // request a total of 14 registers
//    AcX = Wire.read()<<8 | Wire.read();       // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
//    AcY = Wire.read()<<8 | Wire.read();       // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
//    AcZ = Wire.read()<<8 | Wire.read();       // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
//    Tmp = Wire.read()<<8 | Wire.read();       // 0x41 (TEMP_OUT_H) &  0x42 (TEMP_OUT_L)
//    GyX = Wire.read()<<8 | Wire.read();       // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
//    GyY = Wire.read()<<8 | Wire.read();       // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
//    GyZ = Wire.read()<<8 | Wire.read();       // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
//    
//    Serial.print(AcX); Serial.print(" , ");
//    Serial.print(AcY); Serial.print(" , ");
//    Serial.print(AcZ); Serial.print(" , ");
//    Serial.print(GyX); Serial.print(" , ");
//    Serial.print(GyY); Serial.print(" , ");
//    Serial.print(GyZ); Serial.print("\n");
}

//------------------------------------------------------------ Play Wav File ------------------------------------------------
void PlayWav()
{
  static bool ReadingFile=true;                       // True if reading file from SD. false if filling I2S buffer
  static byte Samples[NUM_BYTES_TO_READ_FROM_FILE];   // Memory allocated to store the data read in from the wav file
  static uint16_t BytesRead;                          // Num bytes actually read from the wav file which will either be
                                                      // NUM_BYTES_TO_READ_FROM_FILE or less than this if we are very
                                                      // near the end of the file. i.e. we can't read beyond the file.

  if(ReadingFile)                                     // Read next chunk of data in from file if needed
  {
    BytesRead=ReadFile(Samples);                      // Read data into our memory buffer, return num bytes read in
    ReadingFile=false;                                // Switch to sending the buffer to the I2S
  }
  else
    ReadingFile=FillI2SBuffer(Samples,BytesRead);        // We keep calling this routine until it returns true, at which point
                                                      // this will swap us back to Reading the next block of data from the file.
                                                      // Reading true means it has managed to push all the data to the I2S 
                                                      // Handler, false means there still more to do and you should call this
                                                      // routine again and again until it returns true.
}

//------------------------------------------------------------ Read Wav File ------------------------------------------------
uint16_t ReadFile(byte* Samples)
{
    static uint32_t BytesReadSoFar=0;                   // Number of bytes read from file so far
    uint16_t BytesToRead;                               // Number of bytes to read from the file
    uint16_t i;                                         // loop counter
    int16_t SignedSample;                               // Single Signed Sample

    
    if(BytesReadSoFar+NUM_BYTES_TO_READ_FROM_FILE>WavHeader.DataSize)   // If next read will go past the end then adjust the 
      BytesToRead=WavHeader.DataSize-BytesReadSoFar;                    // amount to read to whatever is remaining to read
    else
      BytesToRead=NUM_BYTES_TO_READ_FROM_FILE;                          // Default to max to read
      
    WavFile.read(Samples,BytesToRead);                  // Read in the bytes from the file
    BytesReadSoFar+=BytesToRead;                        // Update the total bytes read in so far
    
    if(BytesReadSoFar>=WavHeader.DataSize)              // Have we read in all the data?
    {
      WavFile.seek(44);                                 // Reset to start of wav data  
      BytesReadSoFar=0;                                 // Clear to no bytes read in so far                            
    }

//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! This is for Button controls !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//
//    if (digitalRead(PUSH_BUTTON) && (Volume < 1) && (VolUp == 0))
//    {
//      Volume += 0.1;
//      VolUp = 1;
//      Serial.print("Volume Up Button Pressed\n");
//    }
//
//    if ((digitalRead(PUSH_BUTTON) == 0) && (VolUp == 1))
//    {
//      VolUp = 0;
//    }
//
//    if (digitalRead(PUSH_BUTTON_2) && (Volume > 0) && (VolDown == 0))
//    {
//      Volume -= 0.1;
//      VolDown = 1;
//      Serial.print("Volume Down Button Pressed\n");
//    }
//
//    if ((digitalRead(PUSH_BUTTON_2) == 0) && (VolDown == 1))
//    {
//      VolDown = 0;
//    }
//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! This is for Button controls !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

    for (i = 0; i < BytesToRead; i += 2)                // We step two bytes at a time as we're using 16 bits per channel
    {
      SignedSample =* ((int16_t *)(Samples + i));       // Get the Byte address, convert to an int pointer, then get contents of that address as an int
      
//---------------------------------------------- This next line is where we can edit the sound byte -------------------------
      SignedSample = SignedSample * Volume;             // Multiply by the volume - a value that will be between 0 and 1, where 1 would be full volume
      
      *((int16_t *)(Samples + i)) = SignedSample;       // Store back in the memory location we got the sample from

      // The previous 3 lines of code could be written in this one line, would be marginally quicker but harder to follow
      // *((int16_t *)(Samples+i))=(*((int16_t *)(Samples+i)))*Volume; 
    }
    
    return BytesToRead;                                 // return the number of bytes read into buffer
}

//-------------------------------------------------------- Fill Audio Buffer ------------------------------------------------
bool FillI2SBuffer(byte* Samples,uint16_t BytesInBuffer)
{
    // Writes bytes to buffer, returns true if all bytes sent else false, keeps track itself of how many left
    // to write, so just keep calling this routine until returns true to know they've all been written, then
    // you can re-fill the buffer
    
    size_t BytesWritten;                        // Returned by the I2S write routine, 
    static uint16_t BufferIdx=0;                // Current pos of buffer to output next
    uint8_t* DataPtr;                           // Point to next data to send to I2S
    uint16_t BytesToSend;                       // Number of bytes to send to I2S
    
    // To make the code eaier to understand I'm using to variables to some calculations, normally I'd write this calcs
    // directly into the line of code where they belong, but this make it easier to understand what's happening
    
    DataPtr=Samples+BufferIdx;                               // Set address to next byte in buffer to send out
    BytesToSend=BytesInBuffer-BufferIdx;                     // This is amount to send (total less what we've already sent)
    i2s_write(i2s_num,DataPtr,BytesToSend,&BytesWritten,1);  // Send the bytes, wait 1 RTOS tick to complete
    BufferIdx+=BytesWritten;                                 // increasue by number of bytes actually written
    
    if(BufferIdx>=BytesInBuffer)                 
    {
      // sent out all bytes in buffer, reset and return true to indicate this
      BufferIdx=0; 
      return true;                             
    }
    else
      return false;       // Still more data to send to I2S so return false to indicate this
}

//------------------------------------------------------- Initialize SD Card ------------------------------------------------
void SDCardInit()
{        
    pinMode(SD_CS, OUTPUT); 
    digitalWrite(SD_CS, HIGH); // SD card chips select, must use GPIO 5 (ESP32 SS)
    if(!SD.begin(SD_CS))
    {
        Serial.println("Error talking to SD card!");
        while(true);                  // end program
    }
}

//-------------------------------------------------------- Validate Wav Data ------------------------------------------------
bool ValidWavData(WavHeader_Struct* Wav)
{
  
  if(memcmp(Wav->RIFFSectionID,"RIFF",4)!=0) 
  {    
    Serial.print("Invalid data - Not RIFF format");
    return false;        
  }
  if(memcmp(Wav->RiffFormat,"WAVE",4)!=0)
  {
    Serial.print("Invalid data - Not Wave file");
    return false;           
  }
  if(memcmp(Wav->FormatSectionID,"fmt",3)!=0) 
  {
    Serial.print("Invalid data - No format section found");
    return false;       
  }
  if(memcmp(Wav->DataSectionID,"data",4)!=0) 
  {
    Serial.print("Invalid data - data section not found");
    return false;      
  }
  if(Wav->FormatID!=1) 
  {
    Serial.print("Invalid data - format Id must be 1");
    return false;                          
  }
  if(Wav->FormatSize!=16) 
  {
    Serial.print("Invalid data - format section size must be 16.");
    return false;                          
  }
  if((Wav->NumChannels!=1)&(Wav->NumChannels!=2))
  {
    Serial.print("Invalid data - only mono or stereo permitted.");
    return false;   
  }
  if(Wav->SampleRate>48000) 
  {
    Serial.print("Invalid data - Sample rate cannot be greater than 48000");
    return false;                       
  }
  if((Wav->BitsPerSample!=8)& (Wav->BitsPerSample!=16)) 
  {
    Serial.print("Invalid data - Only 8 or 16 bits per sample permitted.");
    return false;                        
  }
  return true;
}


//---------------------------------------------------------- Dump Wav Header ------------------------------------------------
void DumpWAVHeader(WavHeader_Struct* Wav)
{
  if(memcmp(Wav->RIFFSectionID,"RIFF",4)!=0)
  {
    Serial.print("Not a RIFF format file - ");    
    PrintData(Wav->RIFFSectionID,4);
    return;
  } 
  if(memcmp(Wav->RiffFormat,"WAVE",4)!=0)
  {
    Serial.print("Not a WAVE file - ");  
    PrintData(Wav->RiffFormat,4);  
    return;
  }  
  if(memcmp(Wav->FormatSectionID,"fmt",3)!=0)
  {
    Serial.print("fmt ID not present - ");
    PrintData(Wav->FormatSectionID,3);      
    return;
  } 
  if(memcmp(Wav->DataSectionID,"data",4)!=0)
  {
    Serial.print("data ID not present - "); 
    PrintData(Wav->DataSectionID,4);
    return;
  }  
  // All looks good, dump the data
  Serial.print("Total size :");Serial.println(Wav->Size);
  Serial.print("Format section size :");Serial.println(Wav->FormatSize);
  Serial.print("Wave format :");Serial.println(Wav->FormatID);
  Serial.print("Channels :");Serial.println(Wav->NumChannels);
  Serial.print("Sample Rate :");Serial.println(Wav->SampleRate);
  Serial.print("Byte Rate :");Serial.println(Wav->ByteRate);
  Serial.print("Block Align :");Serial.println(Wav->BlockAlign);
  Serial.print("Bits Per Sample :");Serial.println(Wav->BitsPerSample);
  Serial.print("Data Size :");Serial.println(Wav->DataSize);
}

//--------------------------------------------------------------- Print Data ------------------------------------------------
void PrintData(const char* Data,uint8_t NumBytes)
{
    for(uint8_t i=0;i<NumBytes;i++)
      Serial.print(Data[i]); 
      Serial.println();  
}

//--------------------------------------------------- Get Accelerometer Data ------------------------------------------------
void GetGyro()
{
//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! MPU-6050 Accelerometer Manual attempt !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//    Wire.beginTransmission(MPU_ADDR);
//    Wire.write(0x3B);                            // starting with register 0x3B (ACCEL_XOUT_H)
//    Wire.endTransmission(true);
//    Wire.beginTransmission(MPU_ADDR);
//    Wire.requestFrom(MPU_ADDR, 14, true);        // request a total of 14 registers
//    
//    AcX = Wire.read()<<8 | Wire.read();       // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
//    AcY = Wire.read()<<8 | Wire.read();       // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
//    AcZ = Wire.read()<<8 | Wire.read();       // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
//    Tmp = Wire.read()<<8 | Wire.read();       // 0x41 (TEMP_OUT_H) &  0x42 (TEMP_OUT_L)
//    GyX = Wire.read()<<8 | Wire.read();       // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
//    GyY = Wire.read()<<8 | Wire.read();       // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
//    GyZ = Wire.read()<<8 | Wire.read();       // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
//    
//    
//    Serial.print(AcX); Serial.print(" , ");
//    Serial.print(AcY); Serial.print(" , ");
//    Serial.print(AcZ); Serial.print(" , ");
//    Serial.print(GyX); Serial.print(" , ");
//    Serial.print(GyY); Serial.print(" , ");
//    Serial.print(GyZ); Serial.print("\n");
//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! MPU-6050 Accelerometer Manual attempt !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

  /*  Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  /*  Print out the values  */
  Serial.print("Acceleration(m/s^2) X: ");
  Serial.print(a.acceleration.x);
  Serial.print("Y: ");
  Serial.print(a.acceleration.y);
  Serial.print("Z: ");
  Serial.print(a.acceleration.z);
  Serial.print("     ");

  Serial.print("Rotation(rad/s) X: ");
  Serial.print(g.gyro.x);
  Serial.print("Y: ");
  Serial.print(g.gyro.y);
  Serial.print("Z: ");
  Serial.print(g.gyro.z);
  Serial.print("     ");

  Serial.print("Temperature(C): ");
  Serial.print(temp.temperature);
  Serial.print("     ");

  Serial.println("");
  delay(10);
}
