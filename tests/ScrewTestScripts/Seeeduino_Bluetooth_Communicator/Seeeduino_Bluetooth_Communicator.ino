#include <bluefruit.h>
#include <ctype.h> 

BLEDis bledis;
BLEHidAdafruit blehid;

bool hasKeyPressed = false;

String comdata;
int blinktime;
int integerdata;

void setup() 
{
  Serial.begin(115200);
  //  while ( !Serial ) delay(10);   // for nrf52840 with native usb

  pinMode(A0, INPUT_PULLUP);

  Bluefruit.begin();
  Bluefruit.setTxPower(4);    // Check bluefruit.h for supported values
  Bluefruit.setName("ARCSnekTestBED"); //<- You have to put it here for some reason

  // Configure and Start Device Information Service
  bledis.setManufacturer("Adafruit Industries");
  bledis.setModel("Bluefruit Feather 52");
  bledis.begin();

  blehid.begin();

  // Set callback for set LED from central
  blehid.setKeyboardLedCallback(set_keyboard_led);

  /* Set connection interval (min, max) to your perferred value.
   * Note: It is already set by BLEHidAdafruit::begin() to 11.25ms - 15ms
   * min = 9*1.25=11.25 ms, max = 12*1.25= 15 ms 
   */
  /*  */
  Bluefruit.Periph.setConnInterval(15, 15); 

  startAdv();
}

void startAdv(void)
{  
  // Advertising packet
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();
  Bluefruit.Advertising.addAppearance(BLE_APPEARANCE_HID_KEYBOARD);
  
  // Include BLE HID service
  Bluefruit.Advertising.addService(blehid);

  // There is enough room for the dev name in the advertising packet
  Bluefruit.Advertising.addName();
  
  /* Start Advertising
   * - Enable auto advertising if disconnected
   * - Interval:  fast mode = 20 ms, slow mode = 152.5 ms
   * - Timeout for fast mode is 30 seconds
   * - Start(timeout) with timeout = 0 will advertise forever (until connected)
   * 
   * For recommended advertising interval
   * https://developer.apple.com/library/content/qa/qa1931/_index.html   
   */
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);    // in unit of 0.625 ms
  Bluefruit.Advertising.setFastTimeout(30);      // number of seconds in fast mode
  Bluefruit.Advertising.start(0);                // 0 = Don't stop advertising after n seconds
}

void loop() 
{
  SerialChecker();
  if (comdata != "") { 
    char input = toupper(comdata[0]);
    int ans = input-61;
    // int hex = "0x"+String(ans);
    
    uint8_t keycodes[6] = { HID_KEY_CONTROL_LEFT, HID_KEY_SHIFT_LEFT , ans , HID_KEY_NONE , HID_KEY_NONE , HID_KEY_NONE }; //<-- https://github.com/adafruit/Adafruit_nRF52_Arduino/blob/200b3aaefb3256ac26df82ebc9b5b58923d9c37c/cores/nRF5/Adafruit_TinyUSB_Core/tinyusb/src/class/hid/hid.h#L212
    blehid.keyboardReport( 0 , keycodes );  

    blehid.keyRelease(); // <-- Needed or else will become "stuck"
  }
}

void SerialChecker() 
{
  //read string from serial monitor
  comdata = ""; //reset the old values
  if (Serial.available() > 0) // if we get a valid byte, read analog ins:
  {
    while (Serial.available() > 0)
    {
      comdata += char(Serial.read()); //Add the character value to comdata
      delay(2);//so that whole message is sent before the arduino reads it
      integerdata = comdata.toInt(); //converts our character data into a number
      blinktime = integerdata;
    }
    Serial.println(comdata);//prints out what you are typing
  }
}

/**
 * Callback invoked when received Set LED from central.
 * Must be set previously with setKeyboardLedCallback()
 *
 * The LED bit map is as follows: (also defined by KEYBOARD_LED_* )
 *    Kana (4) | Compose (3) | ScrollLock (2) | CapsLock (1) | Numlock (0)
 */
void set_keyboard_led(uint16_t conn_handle, uint8_t led_bitmap)
{
  (void) conn_handle;
  
  // light up Red Led if any bits is set
  if ( led_bitmap )
  {
    ledOn( LED_RED );
  }
  else
  {
    ledOff( LED_RED );
  }
}
