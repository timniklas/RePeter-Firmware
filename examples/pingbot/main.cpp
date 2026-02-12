#include <Arduino.h>   // needed for PlatformIO
#include <Mesh.h>

#if defined(NRF52_PLATFORM)
  #include <InternalFileSystem.h>
#elif defined(RP2040_PLATFORM)
  #include <LittleFS.h>
#elif defined(ESP32)
  #include <SPIFFS.h>
#endif

#include <helpers/ArduinoHelpers.h>
#include <helpers/StaticPoolPacketManager.h>
#include <helpers/SimpleMeshTables.h>
#include <helpers/IdentityStore.h>
#include <RTClib.h>
#include <target.h>

/* ---------------------------------- CONFIGURATION ------------------------------------- */

#ifndef LORA_FREQ
  #define LORA_FREQ   915.0
#endif
#ifndef LORA_BW
  #define LORA_BW     250
#endif
#ifndef LORA_SF
  #define LORA_SF     10
#endif
#ifndef LORA_CR
  #define LORA_CR      5
#endif
#ifndef LORA_TX_POWER
  #define LORA_TX_POWER  20
#endif

#ifndef MAX_CONTACTS
  #define MAX_CONTACTS         100
#endif

#include <helpers/BaseChatMesh.h>

#include <stdlib.h> // für rand()

// Witzige Sprüche für das Display
const char* funny_quotes[] = {
  "Ping kam an. Welt noch da.",
  "Mesh lebt. Keiner weiss warum.",
  "Ping bestaetigt. Zweifel bleibt.",
  "Noch nicht kaputt.",
  "Signal da. Vertrauen nein.",
  "Antwort kam durch. Realitaet wackelt.",
  "Alles okay. Vermutlich.",
  "Es reagiert. Ueberraschend.",
  "Noch funktionsfaehig.",
  "Ich sach mal so.",
  "Das laeuft hier alles.",
  "Is wie es is.",
  "Das ist jetzt nicht optimal.",
  "Lassen wir das mal so stehen.",
  "Da koennte man jetzt diskutieren.",
  "Da bin ich raus.",
  "Das ist hier kein Ponyhof.",
  "Man kann nicht alles haben.",
  "Das war so nicht geplant.",
  "Ich wuerde sagen: passt."
};
const size_t funny_quotes_count = sizeof(funny_quotes) / sizeof(funny_quotes[0]);

// Zeitstempel des letzten empfangenen Pings (Millis)
volatile uint32_t last_ping_display_ms = 0;
volatile int last_ping_quote_idx = -1;
#define PING_DISPLAY_DURATION_MS 5000
#define SEND_TIMEOUT_BASE_MILLIS          500
#define FLOOD_SEND_TIMEOUT_FACTOR         16.0f
#define DIRECT_SEND_PERHOP_FACTOR         6.0f
#define DIRECT_SEND_PERHOP_EXTRA_MILLIS   250

#define  PUBLIC_GROUP_PSK  "PK4W/QZ7qcMqmL4i6bmFJQ=="

// ------------------ Echo / Jitter Configuration ----------------------------------------
#define ECHO_MIN_DELAY_MS          2000
#define ECHO_MAX_DELAY_MS          6000
#define ECHO_BACKOFF_MIN_MS        300
#define ECHO_BACKOFF_MAX_MS        1500
#define ECHO_MAX_TOTAL_DELAY_MS    30000
#define ECHO_MIN_INTERVAL_MS       15000
/* -------------------------------------------------------------------------------------- */

// Persistenz-Datei für die Uhrzeit
#define RTC_FILE_PATH "/rtc_time"

// Believe it or not, this std C function is busted on some platforms!
static uint32_t _atoi(const char* sp) {
  uint32_t n = 0;
  while (*sp && *sp >= '0' && *sp <= '9') {
    n *= 10;
    n += (*sp++ - '0');
  }
  return n;
}

static bool startsWithIgnoreCase(const char* text, const char* prefix) {
  if (!text || !prefix) return false;
  while (*prefix && *text) {
    char ca = *text;
    char cb = *prefix;
    if (ca >= 'A' && ca <= 'Z') ca += 32;
    if (cb >= 'A' && cb <= 'Z') cb += 32;
    if (ca != cb) return false;
    ++text;
    ++prefix;
  }
  return *prefix == 0; // true, wenn prefix komplett übereinstimmt
}

/* ----------------------- Additional helpers for jitter / matching -------------------- */

// uniform random zwischen lo..hi inkl. (nutzt Arduino random())
static uint32_t randBetween(uint32_t lo, uint32_t hi) {
  if (hi <= lo) return lo;
  uint32_t span = hi - lo + 1;
  long r = random((long)span);
  if (r < 0) r = -r;
  return lo + (uint32_t)r;
}

// case-insensitive substring (ASCII)
static bool ciContains(const char* haystack, const char* needle) {
  if (!haystack || !needle || !*needle) return false;
  size_t nlen = strlen(needle);
  for (const char* p = haystack; *p; ++p) {
    size_t i = 0;
    while (i < nlen && p[i]) {
      char a = p[i], b = needle[i];
      if (a >= 'A' && a <= 'Z') a += 32;
      if (b >= 'A' && b <= 'Z') b += 32;
      if (a != b) break;
      ++i;
    }
    if (i == nlen) return true;
  }
  return false;
}

/* ------------------------- Zensur-Helfer für "ping" -> "p*ng" ------------------------ */

// ersetzt jedes (case-insensitive) "ping" im String durch "p*ng" / "P*ng" etc.
static void censorPingInplace(char* s) {
  if (!s) return;
  for (char* p = s; p[0] && p[1] && p[2] && p[3]; ++p) {
    char c0 = p[0], c1 = p[1], c2 = p[2], c3 = p[3];
    if ((c0 == 'p' || c0 == 'P') &&
        (c1 == 'i' || c1 == 'I') &&
        (c2 == 'n' || c2 == 'N') &&
        (c3 == 'g' || c3 == 'G')) {
      // wir behalten das P/p bei und machen p*ng / P*ng
      p[1] = '*';
      p[2] = 'n';
      p[3] = 'g';
    }
  }
}

/* ------------------------- Converts route to comma-separated hex string ------------------------
   out: pointer to target string
   out_size: max size of target string
   route: array containing the route
   route_len: number of used entries in route array
   Hint: in any case, there is a \0 written.
*/
void routeToHexString(char *out, uint8_t out_size, const uint8_t* route, uint8_t route_len) {
  char* p = out;
  size_t remaining = (size_t)out_size;
  int written = 0;

  if (out_size < 1) {
    // out is too small for anything
    return;
  }

  for (uint8_t i = 0; i < route_len; ++i) {
    if (remaining < 4) { // check if we have space for "XX,\0" or "XX\0"
      break;
    }

    if (i < route_len - 1) {
      written = snprintf(p, remaining, "%02X,", route[i]);
    } else {
      written = snprintf(p, remaining, "%02X", route[i]);
    }

    if (written < 0 || written >= (int)remaining) {
      // Error or buffer overflow
      break;
    }
    p += written;
    remaining -= written;
  }
  *p = '\0'; // ensure null termination
}

/* --------------------------------------------------------------------------------------
   WICHTIGER FIX:
   Statt VolatileRTCClock (die oft NICHT tickt) verwenden wir eine RTCClock,
   die auf millis() basiert und zwischen set/get korrekt weiterläuft.
-------------------------------------------------------------------------------------- */
class MillisRTCClock : public mesh::RTCClock {
  uint32_t base_epoch = 0;
  uint32_t base_ms = 0;

public:
  uint32_t getCurrentTime() override {
    if (base_epoch == 0) return 0;
    uint32_t now_ms = millis();
    uint32_t delta_s = (now_ms - base_ms) / 1000U;
    return base_epoch + delta_s;
  }

  void setCurrentTime(uint32_t t) override {
    base_epoch = t;
    base_ms = millis();
  }
};

/* -------------------------------------------------------------------------------------- */

struct NodePrefs {  // persisted to file
  float airtime_factor;
  char node_name[32];
  double node_lat, node_lon;
  float freq;
  uint8_t tx_power_dbm;
  uint8_t unused[3];
  uint8_t echo_mode; // 0 = echo, 1 = funny
};

class MyMesh : public BaseChatMesh, ContactVisitor {
  FILESYSTEM* _fs;
  NodePrefs _prefs;
  uint32_t expected_ack_crc;
  ChannelDetails* _public;
  unsigned long last_msg_sent;
  ContactInfo* curr_recipient;
  char command[512+10];
  uint8_t tmp_buf[256];
  char hex_buf[512];

  // Pending echo state (ein Pending reicht hier)
  bool has_pending_echo;
  uint32_t pending_send_at_ms;                 // millis() Deadline
  uint32_t pending_created_ms;                 // millis() Planungszeit
  char pending_ping_body[MAX_TEXT_LEN + 1];    // Ping-Body (ohne Senderpräfix)
  char pending_sender_name[33];                // Absendername für Erwähnung @[Name]
  float pending_snr;                           // SNR aus dem empfangenen Ping
  uint8_t pending_path_len;                    // Pfadlänge aus dem empfangenen Ping
  uint8_t pending_path[64];                    // Pfad
  uint32_t pending_total_delay_ms;             // aufsummierter Delay inkl. Backoffs
  bool pending_has_direct_snr;                 // true, wenn Route direkt ist

  // Sicherheitsnetz gegen zu häufige Echos
  uint32_t last_echo_sent_ms;

  // ---- Zeit-Persistenz (nur beim Setzen / Advert) ----
  void saveRTCToFS(uint32_t epoch) {
#if defined(NRF52_PLATFORM)
    _fs->remove(RTC_FILE_PATH);
    File f = _fs->open(RTC_FILE_PATH, FILE_O_WRITE);
#elif defined(RP2040_PLATFORM)
    File f = _fs->open(RTC_FILE_PATH, "w");
#else
    File f = _fs->open(RTC_FILE_PATH, "w", true);
#endif
    if (!f) {
      Serial.println("   (WARN: cannot open RTC file for write)");
      return;
    }
    const uint32_t magic = 0x52544331; // "RTC1"
    f.write((const uint8_t*)&magic, sizeof(magic));
    f.write((const uint8_t*)&epoch, sizeof(epoch));
    f.close();
  }

  uint32_t loadRTCFromFS() {
    if (!_fs->exists(RTC_FILE_PATH)) return 0;
#if defined(RP2040_PLATFORM)
    File f = _fs->open(RTC_FILE_PATH, "r");
#else
    File f = _fs->open(RTC_FILE_PATH);
#endif
    if (!f) return 0;
    uint32_t magic = 0, epoch = 0;
    bool ok = (f.read((uint8_t*)&magic, sizeof(magic)) == sizeof(magic)) &&
              (f.read((uint8_t*)&epoch, sizeof(epoch)) == sizeof(epoch));
    f.close();
    if (!ok || magic != 0x52544331) return 0;
    return epoch;
  }

  const char* getTypeName(uint8_t type) const {
    if (type == ADV_TYPE_CHAT) return "Chat";
    if (type == ADV_TYPE_REPEATER) return "Repeater";
    if (type == ADV_TYPE_ROOM) return "Room";
    return "??";  // unknown
  }

  void loadContacts() {
    if (_fs->exists("/contacts")) {
    #if defined(RP2040_PLATFORM)
      File file = _fs->open("/contacts", "r");
    #else
      File file = _fs->open("/contacts");
    #endif
      if (file) {
        bool full = false;
        while (!full) {
          ContactInfo c;
          uint8_t pub_key[32];
          uint8_t unused;
          uint32_t reserved;

          bool success = (file.read(pub_key, 32) == 32);
          success = success && (file.read((uint8_t *) &c.name, 32) == 32);
          success = success && (file.read(&c.type, 1) == 1);
          success = success && (file.read(&c.flags, 1) == 1);
          success = success && (file.read(&unused, 1) == 1);
          success = success && (file.read((uint8_t *) &reserved, 4) == 4);
          success = success && (file.read((uint8_t *) &c.out_path_len, 1) == 1);
          success = success && (file.read((uint8_t *) &c.last_advert_timestamp, 4) == 4);
          success = success && (file.read(c.out_path, 64) == 64);
          c.gps_lat = c.gps_lon = 0;   // not yet supported

          if (!success) break;  // EOF

          c.id = mesh::Identity(pub_key);
          c.lastmod = 0;
          if (!addContact(c)) full = true;
        }
        file.close();
      }
    }
  }

  void saveContacts() {
#if defined(NRF52_PLATFORM)
    _fs->remove("/contacts");
    File file = _fs->open("/contacts", FILE_O_WRITE);
#elif defined(RP2040_PLATFORM)
    File file = _fs->open("/contacts", "w");
#else
    File file = _fs->open("/contacts", "w", true);
#endif
    if (file) {
      ContactsIterator iter;
      ContactInfo c;
      uint8_t unused = 0;
      uint32_t reserved = 0;

      while (iter.hasNext(this, c)) {
        bool success = (file.write(c.id.pub_key, 32) == 32);
        success = success && (file.write((uint8_t *) &c.name, 32) == 32);
        success = success && (file.write(&c.type, 1) == 1);
        success = success && (file.write(&c.flags, 1) == 1);
        success = success && (file.write(&unused, 1) == 1);
        success = success && (file.write((uint8_t *) &reserved, 4) == 4);
        success = success && (file.write((uint8_t *) &c.out_path_len, 1) == 1);
        success = success && (file.write((uint8_t *) &c.last_advert_timestamp, 4) == 4);
        success = success && (file.write(c.out_path, 64) == 64);

        if (!success) break;  // write failed
      }
      file.close();
    }
  }

  void setClock(uint32_t timestamp) {
    uint32_t curr = getRTCClock()->getCurrentTime();
    getRTCClock()->setCurrentTime(timestamp);
    // Nur beim Setzen der Zeit dauerhaft speichern
    saveRTCToFS(timestamp);
    Serial.println("   (OK - clock set & saved!)");
  }

protected:
  float getAirtimeBudgetFactor() const override {
    return _prefs.airtime_factor;
  }

  int calcRxDelay(float score, uint32_t air_time) const override {
    return 0;  // disable rxdelay
  }

  bool allowPacketForward(const mesh::Packet* packet) override {
    return false;
  }

  void onDiscoveredContact(ContactInfo& contact, bool is_new, uint8_t path_len, const uint8_t* path) override {
    Serial.printf("ADVERT from -> %s\n", contact.name);
    Serial.printf("  type: %s\n", getTypeName(contact.type));
    Serial.print("   public key: "); mesh::Utils::printHex(Serial, contact.id.pub_key, PUB_KEY_SIZE); Serial.println();

    saveContacts();
  }

  void onContactPathUpdated(const ContactInfo& contact) override {
    Serial.printf("PATH to: %s, path_len=%d\n", contact.name, (int32_t) contact.out_path_len);
    saveContacts();
  }

  ContactInfo* processAck(const uint8_t *data) override {
    if (memcmp(data, &expected_ack_crc, 4) == 0) {     // got an ACK from recipient
      Serial.printf("   Got ACK! (round trip: %d millis)\n", _ms->getMillis() - last_msg_sent);
      expected_ack_crc = 0;
      return NULL;
    }
    return NULL;
  }

  void onMessageRecv(const ContactInfo& from, mesh::Packet* pkt, uint32_t sender_timestamp, const char *text) override {
    Serial.printf("(%s) MSG -> from %s\n", pkt->isRouteDirect() ? "DIRECT" : "FLOOD", from.name);
    Serial.printf("   %s\n", text);

    if (strcmp(text, "clock sync") == 0) {  // special text command
      setClock(sender_timestamp + 1);
    }
  }

  void onCommandDataRecv(const ContactInfo& from, mesh::Packet* pkt, uint32_t sender_timestamp, const char *text) override {
  }
  void onSignedMessageRecv(const ContactInfo& from, mesh::Packet* pkt, uint32_t sender_timestamp, const uint8_t *sender_prefix, const char *text) override {
  }

  void onChannelMessageRecv(const mesh::GroupChannel& channel, mesh::Packet* pkt, uint32_t timestamp, const char *text) override {
    if (pkt->isRouteDirect()) {
      Serial.printf("PUBLIC CHANNEL MSG -> (Direct!)\n");
    } else {
      Serial.printf("PUBLIC CHANNEL MSG -> (Flood) hops %d\n", pkt->path_len);
    }
    Serial.printf("   %s\n", text);

    const char* body = text;
    const char* sep = strchr(text, ':');
    char bodybuf[MAX_TEXT_LEN + 1];
    char senderName[33] = {0};
    if (sep) {
      size_t nameLen = (size_t)(sep - text);
      if (nameLen > 0 && nameLen < sizeof(senderName)) {
        memcpy(senderName, text, nameLen);
        senderName[nameLen] = 0;
      }
      sep++;
      while (*sep == ' ') sep++;
      strncpy(bodybuf, sep, MAX_TEXT_LEN);
      bodybuf[MAX_TEXT_LEN] = 0;
      body = bodybuf;
    }

    // Eigene Nachrichten ignorieren (falls Namepräfix vorhanden)
    if (senderName[0] && strcmp(senderName, _prefs.node_name) == 0) {
      return;
    }

    // Heuristik: sehr wahrscheinliches lokales Loopback
    if (pkt->isRouteDirect() && pkt->getSNR() == 0.0f && pkt->path_len == 0) {
      return;
    }

    auto isPingTrigger = [](const char* b) -> bool {
      if (!b) return false;
      if (!startsWithIgnoreCase(b, "ping")) return false;
      char c = b[4];
      return (c == 0 || c == ' ');
    };

    bool hasPrefix = (sep != nullptr && senderName[0] != 0);

    // Ping erkannt -> öffentliches Echo planen (nur wenn strenger Trigger erfüllt und Präfix vorhanden)
    if (hasPrefix && isPingTrigger(body)) {

#if ECHO_MIN_INTERVAL_MS > 0
      if (_ms->getMillis() - last_echo_sent_ms < ECHO_MIN_INTERVAL_MS) {
        Serial.printf("   (rate-limited: skipping echo; next in ~%lu ms)\n",
                      (unsigned long)(ECHO_MIN_INTERVAL_MS - (_ms->getMillis() - last_echo_sent_ms)));
        return;
      }
#endif

      char snr_str[16];
      snprintf(snr_str, sizeof(snr_str), "%.1f", pkt->getSNR());

      char route_str[64];  // space for first 21 hops
      bool route_is_direct = (pkt->isRouteDirect() || pkt->path_len == 0);
      if (route_is_direct) {
        strcpy(route_str, "direct");
      } else {
        routeToHexString(route_str, sizeof(route_str), pkt->path, (unsigned)pkt->path_len);
      }

      Serial.printf("   Received public ping -> scheduling echo for \"%s\"\n", body);

      StrHelper::strncpy(pending_ping_body, body, sizeof(pending_ping_body));
      StrHelper::strncpy(pending_sender_name, senderName, sizeof(pending_sender_name)); // Absender merken
      pending_snr = pkt->getSNR();
      pending_path_len = pkt->path_len;
      if (pkt->path_len > 0) {
        memcpy(pending_path, pkt->path, pkt->path_len);
      }
      pending_has_direct_snr = route_is_direct;

      uint32_t base_delay = randBetween(ECHO_MIN_DELAY_MS, ECHO_MAX_DELAY_MS);
      uint32_t now_ms = _ms->getMillis();
      pending_send_at_ms = now_ms + base_delay;
      pending_created_ms = now_ms;
      pending_total_delay_ms = base_delay;
      has_pending_echo = true;

      if (pending_has_direct_snr) {
        Serial.printf("   (echo scheduled in ~%lu ms, SNR: %s dB, route: %s)\n",
                      (unsigned long)base_delay, snr_str, route_str);
      } else {
        Serial.printf("   (echo scheduled in ~%lu ms, route: %s)\n",
                      (unsigned long)base_delay, route_str);
      }
      return;
    }

    if (has_pending_echo) {
      if (startsWithIgnoreCase(body, "echo") || ciContains(body, "echo")) {
        if (ciContains(body, pending_ping_body)) {
          uint32_t extra = randBetween(ECHO_BACKOFF_MIN_MS, ECHO_BACKOFF_MAX_MS);
          uint32_t new_total = pending_total_delay_ms + extra;
          if (new_total > ECHO_MAX_TOTAL_DELAY_MS) new_total = ECHO_MAX_TOTAL_DELAY_MS;

          uint32_t now_ms = _ms->getMillis();
          uint32_t elapsed = now_ms - pending_created_ms;
          if (new_total > elapsed) {
            pending_send_at_ms = now_ms + (new_total - elapsed);
          } else {
            pending_send_at_ms = now_ms + extra / 2;
          }
          pending_total_delay_ms = new_total;

          Serial.printf("   (heard other echo -> +%lu ms backoff, send in ~%lu ms, total ~%lu ms)\n",
                        (unsigned long)extra,
                        (unsigned long)(pending_send_at_ms - now_ms),
                        (unsigned long)pending_total_delay_ms);
        }
      }
    }
  }

  uint8_t onContactRequest(const ContactInfo& contact, uint32_t sender_timestamp, const uint8_t* data, uint8_t len, uint8_t* reply) override {
    return 0;  // unknown
  }

  void onContactResponse(const ContactInfo& contact, const uint8_t* data, uint8_t len) override {
    // not supported
  }

  uint32_t calcFloodTimeoutMillisFor(uint32_t pkt_airtime_millis) const override {
    return SEND_TIMEOUT_BASE_MILLIS + (FLOOD_SEND_TIMEOUT_FACTOR * pkt_airtime_millis);
  }
  uint32_t calcDirectTimeoutMillisFor(uint32_t pkt_airtime_millis, uint8_t path_len) const override {
    return SEND_TIMEOUT_BASE_MILLIS +
         ( (pkt_airtime_millis*DIRECT_SEND_PERHOP_FACTOR + DIRECT_SEND_PERHOP_EXTRA_MILLIS) * (path_len + 1));
  }

  void onSendTimeout() override {
    Serial.println("   ERROR: timed out, no ACK.");
  }

public:
  MyMesh(mesh::Radio& radio, StdRNG& rng, mesh::RTCClock& rtc, SimpleMeshTables& tables)
     : BaseChatMesh(radio, *new ArduinoMillis(), rng, rtc, *new StaticPoolPacketManager(16), tables)
  {
    memset(&_prefs, 0, sizeof(_prefs));
    _prefs.airtime_factor = 2.0;    // one third
    strcpy(_prefs.node_name, "PingBot HB");
    _prefs.freq = LORA_FREQ;
    _prefs.tx_power_dbm = LORA_TX_POWER;
    _prefs.echo_mode = 0; // default: echo

    command[0] = 0;
    curr_recipient = NULL;

    has_pending_echo = false;
    pending_send_at_ms = 0;
    pending_created_ms = 0;
    pending_ping_body[0] = 0;
    pending_sender_name[0] = 0;
    pending_snr = 0.0f;
    pending_path_len = 0;
    pending_total_delay_ms = 0;
    pending_has_direct_snr = false;

    last_echo_sent_ms = 0;
  }

  float getFreqPref() const { return _prefs.freq; }
  uint8_t getTxPowerPref() const { return _prefs.tx_power_dbm; }
  const char* getNodeName() const { return _prefs.node_name; }

  void begin(FILESYSTEM& fs) {
    _fs = &fs;

    BaseChatMesh::begin();

  #if defined(NRF52_PLATFORM)
    IdentityStore store(fs, "");
  #elif defined(RP2040_PLATFORM)
    IdentityStore store(fs, "/identity");
    store.begin();
  #else
    IdentityStore store(fs, "/identity");
  #endif
    if (!store.load("_main", self_id, _prefs.node_name, sizeof(_prefs.node_name))) {  // legacy: node_name was from identity file
      ((StdRNG *)getRNG())->begin(millis());

      self_id = mesh::LocalIdentity(getRNG());  // create new random identity
      int count = 0;
      while (count < 10 && (self_id.pub_key[0] == 0x00 || self_id.pub_key[0] == 0xFF)) {  // reserved id hashes
        self_id = mesh::LocalIdentity(getRNG()); count++;
      }
      store.save("_main", self_id);
    }

    // Persistierte Zeit laden (falls vorhanden)
    uint32_t persisted = loadRTCFromFS();
    if (persisted > 0) {
      uint32_t curr = getRTCClock()->getCurrentTime();
      if (persisted > curr) {
        getRTCClock()->setCurrentTime(persisted);
        Serial.printf("   (RTC restored from FS: %lu)\n", (unsigned long)persisted);
      }
    }

    // load persisted prefs
    if (_fs->exists("/node_prefs")) {
    #if defined(RP2040_PLATFORM)
      File file = _fs->open("/node_prefs", "r");
    #else
      File file = _fs->open("/node_prefs");
    #endif
      if (file) {
        size_t sz = file.read((uint8_t *) &_prefs, sizeof(_prefs));
        // fallback: falls alte Datei ohne echo_mode, Standard setzen
        if (sz < sizeof(_prefs)) {
          _prefs.echo_mode = 0; // default: echo
        }
        file.close();
      }
    } else {
      _prefs.echo_mode = 0; // default: echo
    }

    loadContacts();
    _public = addChannel("#ping", PUBLIC_GROUP_PSK); // pre-configure Andy's public channel
  }

  void savePrefs() {
#if defined(NRF52_PLATFORM)
    _fs->remove("/node_prefs");
    File file = _fs->open("/node_prefs", FILE_O_WRITE);
#elif defined(RP2040_PLATFORM)
    File file = _fs->open("/node_prefs", "w");
#else
    File file = _fs->open("/node_prefs", "w", true);
#endif
    if (file) {
      file.write((const uint8_t *)&_prefs, sizeof(_prefs));
      file.close();
    }
  }

  void showWelcome() {
    Serial.println("===== MeshCore Chat Terminal =====");
    Serial.println();
    Serial.printf("WELCOME  %s\n", _prefs.node_name);
    mesh::Utils::printHex(Serial, self_id.pub_key, PUB_KEY_SIZE);
    Serial.println();
    Serial.println("   (enter 'help' for basic commands)");
    Serial.println();
  }

  void sendSelfAdvert(int delay_millis) {
    // NEU: bei jedem Advert aktuelle Uhrzeit persistieren
    uint32_t now_ts = getRTCClock()->getCurrentTime();
    saveRTCToFS(now_ts);
    Serial.printf("   (RTC saved on advert: %lu)\n", (unsigned long)now_ts);

    uint8_t app_data[MAX_ADVERT_DATA_SIZE];
    uint8_t app_data_len;
    {
      AdvertDataBuilder builder(ADV_TYPE_NONE, _prefs.node_name, _prefs.node_lat, _prefs.node_lon);
      app_data_len = builder.encodeTo(app_data);
    }

    auto pkt = createAdvert(self_id, app_data, app_data_len);
    if (pkt) {
      sendFlood(pkt, delay_millis);
    }
  }

  // ContactVisitor
  void onContactVisit(const ContactInfo& contact) override {
    Serial.printf("   %s - ", contact.name);
    char tmp[40];
    int32_t secs = (int32_t)(getRTCClock()->getCurrentTime() - contact.last_advert_timestamp);
    AdvertTimeHelper::formatRelativeTimeDiff(tmp, secs, false);
    Serial.println(tmp);
  }

  void handleCommand(const char* command) {
    while (*command == ' ') command++;  // skip leading spaces

    if (strcmp(command, "advert") == 0) {
      sendSelfAdvert(0);
      Serial.println("   (advert sent, flood).");
    } else if (strcmp(command, "clock") == 0) {    // show current time
      uint32_t now = getRTCClock()->getCurrentTime();
      DateTime dt = DateTime(now);
      Serial.printf("%02d:%02d - %d/%d/%d UTC\n", dt.hour(), dt.minute(), dt.day(), dt.month(), dt.year());
    } else if (memcmp(command, "time ", 5) == 0) {  // set time (to epoch seconds)
      uint32_t secs = _atoi(&command[5]);
      setClock(secs);
    } else if (memcmp(command, "set ", 4) == 0) {
      const char* config = &command[4];
      if (memcmp(config, "name ", 5) == 0) {
        StrHelper::strncpy(_prefs.node_name, &config[5], sizeof(_prefs.node_name));
        savePrefs();
        Serial.println("  OK");
      } else if (memcmp(config, "lat ", 4) == 0) {
        _prefs.node_lat = atof(&config[4]);
        savePrefs();
        Serial.println("  OK");
      } else if (memcmp(config, "lon ", 4) == 0) {
        _prefs.node_lon = atof(&config[4]);
        savePrefs();
        Serial.println("  OK");
      } else if (memcmp(config, "tx ", 3) == 0) {
        _prefs.tx_power_dbm = atoi(&config[3]);
        savePrefs();
        Serial.println("  OK - reboot to apply");
      } else if (memcmp(config, "echomode ", 9) == 0) {
        const char* mode = &config[9];
        if (strcmp(mode, "echo") == 0) {
          _prefs.echo_mode = 0;
          savePrefs();
          Serial.println("  OK - echo mode set to echo");
        } else if (strcmp(mode, "fun") == 0) {
          _prefs.echo_mode = 1;
          savePrefs();
          Serial.println("  OK - echo mode set to fun");
        } else {
          Serial.println("  ERROR: unknown echomode (use 'echo' or 'fun')");
        }
      } else {
        Serial.printf("  ERROR: unknown config: %s\n", config);
      }
    } else if (memcmp(command, "help", 4) == 0) {
      Serial.println("Commands:");
      Serial.println("   set {name|lat|lon|tx|echomode} {value}");
      Serial.println("      echomode: echo | fun");
      Serial.println("   advert");
      Serial.println("   clock");
      Serial.println("   time <epoch_seconds>");
    } else {
      Serial.print("   ERROR: unknown command: "); Serial.println(command);
    }
  }

  void loop() {
    BaseChatMesh::loop();

    int len = strlen(command);
    while (Serial.available() && len < (int)sizeof(command)-1) {
      char c = Serial.read();
      if (c != '\n') {
        command[len++] = c;
        command[len] = 0;
      }
      Serial.print(c);
    }
    if (len == (int)sizeof(command)-1) {  // command buffer full
      command[sizeof(command)-1] = '\r';
    }

    if (len > 0 && command[len - 1] == '\r') {  // received complete line
      command[len - 1] = 0;  // replace newline with C string null terminator

      handleCommand(command);
      command[0] = 0;  // reset command buffer
    }

    // --- Geplante Echo-Sendungen ausführen ---
    if (has_pending_echo) {
      uint32_t now_ms = _ms->getMillis();
      if ((int32_t)(now_ms - pending_send_at_ms) >= 0) {
        char snr_str[16];
        snprintf(snr_str, sizeof(snr_str), "%.1f", pending_snr);

        char route_str[64];  // space for first 21 hops
        if (pending_path_len == 0) {
          strcpy(route_str, "direct");
        } else {
          routeToHexString(route_str, sizeof(route_str), pending_path, (unsigned)pending_path_len);
        }

        char replyText[MAX_TEXT_LEN];
        const char* mention = (pending_sender_name[0] != 0) ? pending_sender_name : "unknown";

        int quote_idx = -1;
        const char* funny = nullptr;

        if (_prefs.echo_mode == 1) {
          // Witzigen Spruch auswählen
          quote_idx = random(funny_quotes_count);
          funny = funny_quotes[quote_idx];
        }

        // Antwort zusammenbauen je nach Modus
        if (_prefs.echo_mode == 1) {
          // Fun Spruch
          if (pending_has_direct_snr) {
            snprintf(replyText, sizeof(replyText),
                     "@[%s] %s (SNR: %s dB, Route: %s)",
                     mention, funny, snr_str, route_str);
          } else {
            snprintf(replyText, sizeof(replyText),
                     "@[%s] %s (Route: %s)",
                     mention, funny, route_str);
          }
        } else {
          // Klassisches Echo
          if (pending_has_direct_snr) {
            snprintf(replyText, sizeof(replyText),
                     "@[%s] ECHO: %s (SNR: %s dB, Route: %s)",
                     mention, pending_ping_body, snr_str, route_str);
          } else {
            snprintf(replyText, sizeof(replyText),
                     "@[%s] ECHO: %s (Route: %s)",
                     mention, pending_ping_body, route_str);
          }
        }

        Serial.printf("   Sending delayed public echo -> \"%s\"\n", replyText);

        uint8_t temp[5 + MAX_TEXT_LEN + 32];
        uint32_t now = getRTCClock()->getCurrentTime();
        memcpy(temp, &now, 4);
        temp[4] = 0;

        // finaler Klartext inkl. Node-Namen
        snprintf((char*)&temp[5], MAX_TEXT_LEN,
                 "%s: %s", _prefs.node_name, replyText);
        ((char*)&temp[5])[MAX_TEXT_LEN] = 0;

        // HIER: "ping" -> "p*ng" im finalen Text (inkl. Namen)
        censorPingInplace((char*)&temp[5]);

        int outlen = strlen((char*)&temp[5]);
        auto pkt2 = createGroupDatagram(PAYLOAD_TYPE_GRP_TXT,
                                        _public->channel, temp, 5 + outlen);
        if (pkt2) {
          sendFlood(pkt2);
          Serial.println("   (public echo sent - delayed)");
#if ECHO_MIN_INTERVAL_MS > 0
          last_echo_sent_ms = _ms->getMillis();
#endif
        } else {
          Serial.println("   ERROR: unable to send public echo (delayed)");
        }

        // Reset pending state
        has_pending_echo = false;

        // Nach Ping: Zeitstempel und Quote merken (nur für Anzeige)
        last_ping_display_ms = millis();
        last_ping_quote_idx = (quote_idx >= 0) ? quote_idx : 0;
        pending_ping_body[0] = 0;
        pending_sender_name[0] = 0;
        pending_snr = 0.0f;
        pending_path_len = 0;
        pending_total_delay_ms = 0;
        pending_has_direct_snr = false;
      }
    }

// Displayanzeige ins loop() verschoben
  }
};

StdRNG fast_rng;
SimpleMeshTables tables;

// FIX: tickende RTC statt VolatileRTCClock
MillisRTCClock tick_rtc;

MyMesh the_mesh(radio_driver, fast_rng, tick_rtc, tables);

void halt() {
  while (1) ;
}

void setup() {
  Serial.begin(115200);

  board.begin();

#ifdef DISPLAY_CLASS
  if (display.begin()) {
    display.startFrame();
    display.setCursor(0, 0);
    display.print("Please wait...");
    display.endFrame();
  }
#endif

  if (!radio_init()) { halt(); }

  fast_rng.begin(radio_get_rng_seed());

  // Seed für Arduino random(): mische Funk-Zufall mit Zeit
  randomSeed((uint32_t)(micros() ^ radio_get_rng_seed()));

#if defined(NRF52_PLATFORM)
  InternalFS.begin();
  the_mesh.begin(InternalFS);
#elif defined(RP2040_PLATFORM)
  LittleFS.begin();
  the_mesh.begin(LittleFS);
#elif defined(ESP32)
  SPIFFS.begin(true);
  the_mesh.begin(SPIFFS);
#else
  #error "need to define filesystem"
#endif

  radio_set_params(the_mesh.getFreqPref(), LORA_BW, LORA_SF, LORA_CR);
  radio_set_tx_power(the_mesh.getTxPowerPref());

  the_mesh.showWelcome();

  // send out initial Advertisement to the mesh
  the_mesh.sendSelfAdvert(1200);   // add slight delay
}

void loop() {
  the_mesh.loop();
#ifdef DISPLAY_CLASS
  static uint32_t last_display_update = 0;
  uint32_t now_ms = millis();
  if (now_ms - last_display_update > 200) { // max 5x pro Sekunde aktualisieren
    last_display_update = now_ms;
    display.startFrame();
    display.setCursor(0, 0);
    display.print("< Ping Server >");
    display.setCursor(0, 10);
    display.print(the_mesh.getNodeName());
    if (last_ping_display_ms && (now_ms - last_ping_display_ms < PING_DISPLAY_DURATION_MS) && last_ping_quote_idx >= 0) {
      display.setCursor(0, 30);
      display.print("Ping empfangen!");
      display.setCursor(0, 40);
      display.print(funny_quotes[last_ping_quote_idx]);
    }
    display.setCursor(0, 50);
    uint32_t now = the_mesh.getRTCClock()->getCurrentTime();
    DateTime dt = DateTime(now);
    char buffer[32];
    sprintf(buffer, "%02d.%02d.%d %02d:%02d UTC",
            dt.day(), dt.month(), dt.year(), dt.hour(), dt.minute());
    display.print(buffer);
    display.endFrame();
  }
#endif
}
