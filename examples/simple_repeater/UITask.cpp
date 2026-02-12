#include "UITask.h"
#include <Arduino.h>
#include <helpers/CommonCLI.h>
#include <RTClib.h>

#define AUTO_OFF_MILLIS      20000  // 20 seconds
#define BOOT_SCREEN_MILLIS   4000   // 4 seconds

// 'meshcore', 128x13px
static const uint8_t meshcore_logo [] PROGMEM = {
    0x3c, 0x01, 0xe3, 0xff, 0xc7, 0xff, 0x8f, 0x03, 0x87, 0xfe, 0x1f, 0xfe, 0x1f, 0xfe, 0x1f, 0xfe, 
    0x3c, 0x03, 0xe3, 0xff, 0xc7, 0xff, 0x8e, 0x03, 0x8f, 0xfe, 0x3f, 0xfe, 0x1f, 0xff, 0x1f, 0xfe, 
    0x3e, 0x03, 0xc3, 0xff, 0x8f, 0xff, 0x0e, 0x07, 0x8f, 0xfe, 0x7f, 0xfe, 0x1f, 0xff, 0x1f, 0xfc, 
    0x3e, 0x07, 0xc7, 0x80, 0x0e, 0x00, 0x0e, 0x07, 0x9e, 0x00, 0x78, 0x0e, 0x3c, 0x0f, 0x1c, 0x00, 
    0x3e, 0x0f, 0xc7, 0x80, 0x1e, 0x00, 0x0e, 0x07, 0x1e, 0x00, 0x70, 0x0e, 0x38, 0x0f, 0x3c, 0x00, 
    0x7f, 0x0f, 0xc7, 0xfe, 0x1f, 0xfc, 0x1f, 0xff, 0x1c, 0x00, 0x70, 0x0e, 0x38, 0x0e, 0x3f, 0xf8, 
    0x7f, 0x1f, 0xc7, 0xfe, 0x0f, 0xff, 0x1f, 0xff, 0x1c, 0x00, 0xf0, 0x0e, 0x38, 0x0e, 0x3f, 0xf8, 
    0x7f, 0x3f, 0xc7, 0xfe, 0x0f, 0xff, 0x1f, 0xff, 0x1c, 0x00, 0xf0, 0x1e, 0x3f, 0xfe, 0x3f, 0xf0, 
    0x77, 0x3b, 0x87, 0x00, 0x00, 0x07, 0x1c, 0x0f, 0x3c, 0x00, 0xe0, 0x1c, 0x7f, 0xfc, 0x38, 0x00, 
    0x77, 0xfb, 0x8f, 0x00, 0x00, 0x07, 0x1c, 0x0f, 0x3c, 0x00, 0xe0, 0x1c, 0x7f, 0xf8, 0x38, 0x00, 
    0x73, 0xf3, 0x8f, 0xff, 0x0f, 0xff, 0x1c, 0x0e, 0x3f, 0xf8, 0xff, 0xfc, 0x70, 0x78, 0x7f, 0xf8, 
    0xe3, 0xe3, 0x8f, 0xff, 0x1f, 0xfe, 0x3c, 0x0e, 0x3f, 0xf8, 0xff, 0xfc, 0x70, 0x3c, 0x7f, 0xf8, 
    0xe3, 0xe3, 0x8f, 0xff, 0x1f, 0xfc, 0x3c, 0x0e, 0x1f, 0xf8, 0xff, 0xf8, 0x70, 0x3c, 0x7f, 0xf8, 
};

// 'logo', 32x32px
const unsigned char bremesh_logo [] PROGMEM = {
	0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 0xc0, 0x00, 0x00, 0xff, 0xf0, 0x00, 0x01, 0xc0, 0x3c, 0x00, 
	0x07, 0x00, 0x0e, 0x00, 0x0e, 0x00, 0x07, 0x00, 0x0c, 0x1f, 0x81, 0x80, 0x18, 0x7f, 0xe0, 0x80, 
	0x30, 0xe0, 0x30, 0x00, 0x31, 0x80, 0x18, 0x00, 0x23, 0x80, 0x0c, 0x00, 0x63, 0x0f, 0x8e, 0x00, 
	0x63, 0x1f, 0xc0, 0x1c, 0x66, 0x30, 0x80, 0x3e, 0x66, 0x30, 0x7f, 0xfe, 0x66, 0x30, 0xff, 0xfe, 
	0x66, 0x39, 0xc0, 0x3e, 0x63, 0x1b, 0x00, 0x38, 0x63, 0x03, 0x00, 0x60, 0x21, 0x83, 0x00, 0x60, 
	0x31, 0xc3, 0x00, 0xc0, 0x30, 0xe3, 0x00, 0xc0, 0x18, 0x7b, 0x01, 0x80, 0x0c, 0x03, 0x03, 0x00, 
	0x06, 0x03, 0x07, 0x00, 0x03, 0x83, 0x1c, 0x00, 0x01, 0xe3, 0x78, 0x00, 0x00, 0x7f, 0xe0, 0x00, 
	0x00, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

void UITask::renderBatteryIndicator(DisplayDriver* display, uint16_t batteryMilliVolts) {
  // Convert millivolts to percentage
  const int minMilliVolts = 3000; // Minimum voltage (e.g., 3.0V)
  const int maxMilliVolts = 4200; // Maximum voltage (e.g., 4.2V)
  int batteryPercentage = ((batteryMilliVolts - minMilliVolts) * 100) / (maxMilliVolts - minMilliVolts);
  if (batteryPercentage < 0) batteryPercentage = 0; // Clamp to 0%
  if (batteryPercentage > 100) batteryPercentage = 100; // Clamp to 100%

  // battery icon
  int iconWidth = 24;
  int iconHeight = 10;
  int iconX = display->width() - iconWidth - 5; // Position the icon near the top-right corner
  int iconY = 0;
  display->setColor(DisplayDriver::GREEN);

  // battery outline
  display->drawRect(iconX, iconY, iconWidth, iconHeight);

  // battery "cap"
  display->fillRect(iconX + iconWidth, iconY + (iconHeight / 4), 3, iconHeight / 2);

  // fill the battery based on the percentage
  int fillWidth = (batteryPercentage * (iconWidth - 4)) / 100;
  display->fillRect(iconX + 2, iconY + 2, fillWidth, iconHeight - 4);
}

void UITask::begin(mesh::RTCClock* clock, mesh::MainBoard* board, NodePrefs* node_prefs, const char* build_date, const char* firmware_version) {
  _prevBtnState = HIGH;
  _auto_off = millis() + AUTO_OFF_MILLIS;
  _node_prefs = node_prefs;
  _display->turnOn();
  _board = board;
  _clock = clock;

  // strip off dash and commit hash by changing dash to null terminator
  // e.g: v1.2.3-abcdef -> v1.2.3
  char *version = strdup(firmware_version);
  char *dash = strchr(version, '-');
  if(dash){
    *dash = 0;
  }

  // v1.2.3 (1 Jan 2025)
  sprintf(_version_info, "%s (%s)", version, build_date);
}

void UITask::renderCurrScreen() {
  char tmp[80];
  if (millis() < BOOT_SCREEN_MILLIS) { // boot screen
    // meshcore logo
    _display->setColor(DisplayDriver::BLUE);
    int logoWidth = 32;
    _display->drawXbm((_display->width() - logoWidth) / 2, 0, bremesh_logo, logoWidth, 32);


    // node type
    const char* node_type = "BreMesh RePeter";
    uint16_t typeWidth = _display->getTextWidth(node_type);
    _display->setCursor((_display->width() - typeWidth) / 2, 35);
    _display->print(node_type);

    // version info
    _display->setColor(DisplayDriver::LIGHT);
    _display->setTextSize(1);
    uint16_t versionWidth = _display->getTextWidth(_version_info);
    _display->setCursor((_display->width() - versionWidth) / 2, 55);
    _display->print(_version_info);
  } else {  // home screen
    // time
    _display->setCursor(0, 2);
    uint32_t now = _clock->getCurrentTime();
    DateTime dt = DateTime(now);
    char buffer[32];
    sprintf(buffer, "%02d.%02d.%d %02d:%02d",
            dt.day(), dt.month(), dt.year(), dt.hour(), dt.minute());
    _display->print(buffer);

    // freq / sf
    _display->setCursor(0, 20);
    _display->setColor(DisplayDriver::YELLOW);
    sprintf(tmp, "FREQ: %06.3f SF%d", _node_prefs->freq, _node_prefs->sf);
    _display->print(tmp);

    // bw / cr
    _display->setCursor(0, 30);
    sprintf(tmp, "BW: %03.2f CR: %d", _node_prefs->bw, _node_prefs->cr);
    _display->print(tmp);

    // vbattery
    _display->setCursor(0, 40);
    sprintf(tmp, "BAT: %03.2f Volt", _board->getBattMilliVolts() / 1000.0);
    _display->print(tmp);

    // node name
    _display->setCursor(0, 55);
    _display->setTextSize(1);
    _display->setColor(DisplayDriver::GREEN);
    _display->print(_node_prefs->node_name);

    // battery icon
    renderBatteryIndicator(_display, _board->getBattMilliVolts());
  }
}

void UITask::loop() {
#ifdef PIN_USER_BTN
  if (millis() >= _next_read) {
    int btnState = digitalRead(PIN_USER_BTN);
    if (btnState != _prevBtnState) {
      if (btnState == LOW) {  // pressed?
        if (_display->isOn()) {
          // TODO: any action ?
        } else {
          _display->turnOn();
        }
        _auto_off = millis() + AUTO_OFF_MILLIS;   // extend auto-off timer
      }
      _prevBtnState = btnState;
    }
    _next_read = millis() + 200;  // 5 reads per second
  }
#endif

  if (_display->isOn()) {
    if (millis() >= _next_refresh) {
      _display->startFrame();
      renderCurrScreen();
      _display->endFrame();

      _next_refresh = millis() + 1000;   // refresh every second
    }
    if (millis() > _auto_off) {
      _display->turnOff();
    }
  }
}
