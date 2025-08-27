
#include <Arduino.h>
#include <SPI.h>
#include <Arduino_GFX_Library.h>
#include <XPT2046_Touchscreen.h>
#include <SoftwareSerial.h>

#define TFT_CS    10
#define TFT_DC     8
#define TFT_RST    7
#define TFT_BL     9
#define SD_CS      6
#define TOUCH_CS   5
#define TOUCH_IRQ  4

#define TFT_ROTATION 1
#define TFT_WIDTH   480
#define TFT_HEIGHT  320

#define TS_MINX  200
#define TS_MAXX  3800
#define TS_MINY  200
#define TS_MAXY  3800

#define C_BLACK gfx->color565(0,0,0)
#define C_WHITE gfx->color565(255,255,255)
#define C_ACC   gfx->color565(180,220,255)
#define C_HDR   gfx->color565(30,30,30)
#define C_DIM   gfx->color565(70,70,70)
#define C_RED   gfx->color565(255,0,0)

Arduino_DataBus *bus = new Arduino_HWSPI(TFT_DC, TFT_CS);
Arduino_GFX *gfx = new Arduino_ST7796(bus, TFT_RST, TFT_ROTATION, true);

XPT2046_Touchscreen ts(TOUCH_CS, TOUCH_IRQ);
int16_t mapTouchX(int16_t rx, int16_t ry){ return map(ry, TS_MINY, TS_MAXY, 0, TFT_WIDTH); }
int16_t mapTouchY(int16_t rx, int16_t ry){ return map(rx, TS_MINX, TS_MAXX, 0, TFT_HEIGHT); }

#define ESP_RX_PIN 2
#define ESP_TX_PIN 3
SoftwareSerial esp(ESP_RX_PIN, ESP_TX_PIN);

const char *SSID = "JM notebook";
const char *PASS = "1234567890a";

uint32_t lastUIBlink = 0;
bool waitingBlink = false;

String up(String s){ for(size_t i=0;i<s.length();++i){ char c=s[i]; if(c>='a'&&c<='z') s[i]=c-'a'+'A'; } return s; }
String trimAll(String s){ s.trim(); return s; }

void flushESP(uint16_t ms){
  uint32_t t0 = millis();
  while (millis() - t0 < ms){
    while (esp.available()) (void)esp.read();
    delay(1);
  }
}

void atLog(const String &cmd, uint16_t wait=3000){
  esp.println(cmd);
  uint32_t t0=millis();
  while(millis()-t0<wait){ while(esp.available()) Serial.write(esp.read()); yield(); }
}
String atCap(const String &cmd, uint16_t wait=3000){
  esp.println(cmd);
  String out; uint32_t t0=millis();
  while(millis()-t0<wait){ while(esp.available()) out += char(esp.read()); yield(); }
  return out;
}
String parseSTAIP(const String &cif){
  int p=cif.indexOf("STAIP,\""); if(p<0) return "";
  p+=7; int q=cif.indexOf('"',p); if(q<0) return "";
  return cif.substring(p,q);
}

/* ---------- UI ---------- */
void drawHeader(const String &ip){
  gfx->fillRect(0,0,TFT_WIDTH,40, C_HDR);
  gfx->setTextWrap(false);
  gfx->setTextColor(C_WHITE);
  gfx->setFont(nullptr);
  gfx->setTextSize(1);
  gfx->setCursor(10,12);  gfx->print("APP DISPLAY  |  480x320  |  TCP 8080");
  gfx->setCursor(330,12); gfx->print(ip.length()? ("IP: "+ip) : "(IP N/A)");
}
void clearMain(){ gfx->fillRect(0,40,TFT_WIDTH,TFT_HEIGHT-40, C_BLACK); }

void showHelp(){
  clearMain();
  gfx->setTextWrap(false);
  gfx->setTextColor(C_WHITE); gfx->setFont(nullptr); gfx->setTextSize(2);
  gfx->setCursor(20,70);  gfx->print("Send CSV:");
  gfx->setCursor(20,105); gfx->print("count,part,color,sleeve");
  gfx->setTextSize(1);
  gfx->setCursor(20,140); gfx->print("(legacy index,count,... also OK)");
}

/* 데이터 표시(라벨/값) — index 제거 */
void showData(int total,const String& part,const String& color,const String& sleeve){
  gfx->startWrite();
  clearMain();

  int xL=20, xV=150, y=70, dy=34;
  gfx->setTextWrap(false);
  gfx->setTextSize(2); gfx->setFont(nullptr);

  gfx->setTextColor(C_ACC);   gfx->setCursor(xL,y); gfx->print("Total:");
  gfx->setTextColor(C_WHITE); gfx->setCursor(xV,y); gfx->print(total);

  y+=dy; gfx->setTextColor(C_ACC);   gfx->setCursor(xL,y); gfx->print("PART:");
        gfx->setTextColor(C_WHITE); gfx->setCursor(xV,y); gfx->print(part);

  y+=dy; gfx->setTextColor(C_ACC);   gfx->setCursor(xL,y); gfx->print("COLOR:");
        gfx->setTextColor(C_WHITE); gfx->setCursor(xV,y); gfx->print(color);

  y+=dy; gfx->setTextColor(C_ACC);   gfx->setCursor(xL,y); gfx->print("SLEEVE:");
        gfx->setTextColor(C_WHITE); gfx->setCursor(xV,y); gfx->print(sleeve);

  // 하단 CLEAR 버튼
  gfx->fillRoundRect(10, 260, 120, 36, 8, C_DIM);
  gfx->drawRoundRect(10, 260, 120, 36, 8, gfx->color565(200,200,200));
  gfx->setCursor(28,272); gfx->setTextSize(1); gfx->setTextColor(C_WHITE); gfx->print("CLEAR");

  gfx->endWrite();
}

/* ---------- color+sleeve 복원 ---------- */
bool splitColorSleeve(String tail, String &color, String &sleeve){
  tail.trim();
  if (tail.length()==0){ color=""; sleeve=""; return false; }

  // 1) 정상 콤마
  int c = tail.indexOf(',');
  if (c>=0){
    color  = trimAll(tail.substring(0,c));
    sleeve = trimAll(tail.substring(c+1));
    return (color.length()>0 && sleeve.length()>0);
  }

  // 2) sleeve 접미어
  const char* suffixes[] = {"short","long","sleeveless","pants","skirt","jeans","slacks","half","quarter"};
  String lower=tail; lower.toLowerCase();
  for (auto sfx: suffixes){
    String S=sfx;
    if (lower.endsWith(S)){
      int cut = lower.length() - S.length();
      color  = trimAll(tail.substring(0, cut));
      sleeve = tail.substring(cut); sleeve.trim();
      if (color.length()>0 && sleeve.length()>0) return true;
    }
  }

  // 3) 마지막 공백
  int sp = tail.lastIndexOf(' ');
  if (sp>0 && sp<tail.length()-1){
    color  = trimAll(tail.substring(0, sp));
    sleeve = trimAll(tail.substring(sp+1));
    if (color.length()>0 && sleeve.length()>0) return true;
  }

  // 4) 실패: color만
  color  = tail; sleeve = "";
  return false;
}

/* ---------- 도움: 콤마 개수 세기 ---------- */
int countCommas(const String &s){
  int cnt=0;
  for (size_t i=0;i<s.length();++i) if (s[i]==',') cnt++;
  return cnt;
}

/* ---------- 앞쪽 필드 파싱 (index 무시 로직 포함) ----------
   결과: total, part, tail(color+옵션sleeve) */
bool parseHeadIgnoreIndex(const String &line, String &sTot, String &sPart, String &tail){
  int comm = countCommas(line);

  if (comm >= 3) {
    // 아마도 legacy 5필드(index,count,part,...) 또는 5필드 정상
    // -> 첫 3필드 추출 후 index 버리고 count/part 사용
    int p0 = line.indexOf(',');
    int p1 = line.indexOf(',', p0+1);
    int p2 = line.indexOf(',', p1+1);
    if (p0<0 || p1<0) return false;

    String sIdx = line.substring(0, p0);           // 버림
    sTot  = line.substring(p0+1, p1);  sTot.trim();
    sPart = (p2<0) ? line.substring(p1+1) : line.substring(p1+1, p2);
    sPart.trim();
    tail  = (p2<0) ? "" : line.substring(p2+1); tail.trim();
    return true;
  }

  if (comm == 2) {
    // 신 포맷 4필드(count,part,color,sleeve) 이거나 3필드+tail
    int p0 = line.indexOf(',');
    int p1 = line.indexOf(',', p0+1);
    if (p0<0 || p1<0) return false;
    sTot  = line.substring(0, p0);          sTot.trim();
    sPart = line.substring(p0+1, p1);       sPart.trim();
    tail  = line.substring(p1+1);           tail.trim();
    return true;
  }

  // 콤마 부족
  return false;
}

/* ---------- +IPD (능동 대기형) ---------- */
bool readIPD(int &outId, String &outPayload, uint16_t timeout=800){
  uint32_t t0 = millis();
  esp.setTimeout(200);
  while (millis() - t0 < timeout){
    if (esp.find((char*)"+IPD,")){
      int id = esp.parseInt();
      if (esp.read()!=',') return false;
      int len = esp.parseInt();
      if (esp.read()!=':') return false;

      outPayload.reserve(len);
      outPayload="";
      uint32_t t1 = millis();
      while ((int)outPayload.length()<len && (millis()-t1) < 1200){
        if (esp.available()) outPayload += char(esp.read());
        yield();
      }
      if ((int)outPayload.length()!=len) return false;
      outId = id;
      return true;
    }
    delay(1);
  }
  return false;
}

void sendAck(int id, const String &msg){
  if (id<0) return;
  String body = msg + "\r\n";
  esp.println("AT+CIPSEND=" + String(id) + "," + String(body.length()));
  delay(15);
  esp.print(body);
  delay(5);
  esp.println("AT+CIPCLOSE=" + String(id));
}

/* ---------- CLEAR 버튼 ---------- */
const int CLEAR_X=10, CLEAR_Y=260, CLEAR_W=120, CLEAR_H=36;
void drawClearBtn(){
  gfx->fillRoundRect(CLEAR_X,CLEAR_Y,CLEAR_W,CLEAR_H,8, C_DIM);
  gfx->drawRoundRect(CLEAR_X,CLEAR_Y,CLEAR_W,CLEAR_H,8, gfx->color565(200,200,200));
  gfx->setCursor(CLEAR_X+18, CLEAR_Y+12); gfx->setTextColor(C_WHITE); gfx->setFont(nullptr); gfx->setTextSize(1); gfx->print("CLEAR");
}
bool inClearBtn(int16_t x,int16_t y){
  return (x>=CLEAR_X && x<=CLEAR_X+CLEAR_W && y>=CLEAR_Y && y<=CLEAR_Y+CLEAR_H);
}

/* ---------- setup ---------- */
void setup(){
  pinMode(TFT_CS,OUTPUT);   digitalWrite(TFT_CS,HIGH);
  pinMode(TFT_BL,OUTPUT);   digitalWrite(TFT_BL,HIGH);
  pinMode(TOUCH_CS,OUTPUT); digitalWrite(TOUCH_CS,HIGH);
  pinMode(SD_CS,OUTPUT);    digitalWrite(SD_CS,HIGH);
  pinMode(TOUCH_IRQ,INPUT_PULLUP);

  Serial.begin(9600);
  esp.begin(9600);
  esp.setTimeout(2000);

  if (!gfx->begin()){ Serial.println(F("TFT init failed!")); while(1) delay(1); }
  gfx->fillScreen(C_BLACK);
  ts.begin();

  atLog("ATE0");
  atLog("AT");
  atLog("AT+CWMODE=1");
  atLog("AT+CWJAP=\"" + String(SSID) + "\",\"" + String(PASS) + "\"", 12000);

  String ip = parseSTAIP(atCap("AT+CIFSR", 2000));
  drawHeader(ip);
  drawClearBtn();
  showHelp();

  atLog("AT+CIPMUX=1");
  atLog("AT+CIPSERVER=1,8080");
  atLog("AT+CIPSTO=180");

  flushESP(150); // 첫 패킷 동기화
}

/* ---------- loop ---------- */
void loop(){
  int linkId; String payload;

  // 1) 수신 처리
  if (readIPD(linkId, payload, 800)){
    gfx->drawPixel(2, 42, C_RED); // 수신 피드백
    Serial.print("[RX] payload=\""); Serial.print(payload); Serial.println("\"");

    // 브라우저 GET 허용
    if (payload.startsWith("GET ")){
      int q = payload.indexOf("?d=");
      int sp = payload.indexOf(' ', 4);
      if (q>0 && sp>q) payload = payload.substring(q+3, sp);
      else { sendAck(linkId, "ERR HTTP"); return; }
    }

    // 헤드 파싱 (index 무시)
    String sTot, sPart, tail;
    if (!parseHeadIgnoreIndex(payload, sTot, sPart, tail)){
      sendAck(linkId, "ERR HEAD");
      return;
    }

    int total = sTot.toInt();
    String part = up(sPart);
    if (part=="BOTTOM") part="BTM";
    if (part=="TOPS")   part="TOP";

    // color+sleeve 복원
    String color, sleeve;
    bool okTail = splitColorSleeve(tail, color, sleeve);
    color = up(color); sleeve = up(sleeve);

    // SPI 상태 확정 후 표시
    digitalWrite(TOUCH_CS, HIGH);
    digitalWrite(TFT_CS, HIGH); delayMicroseconds(2);
    digitalWrite(TFT_CS, LOW);  delayMicroseconds(2);
    gfx->startWrite();
    showData(total, part, color, sleeve);
    gfx->endWrite();

    // 시리얼 로깅(총개수만)
    Serial.print("Total="); Serial.println(total);

    sendAck(linkId, okTail ? "OK SHOW" : "OK SHOW*");
  }

  // 2) 터치 → CLEAR
  if (ts.touched()){
    TS_Point p = ts.getPoint();
    digitalWrite(TOUCH_CS, HIGH);
    digitalWrite(TFT_CS, HIGH); delayMicroseconds(2);
    digitalWrite(TFT_CS, LOW);  delayMicroseconds(2);

    int16_t sx = mapTouchX(p.x,p.y), sy = mapTouchY(p.x,p.y);
    if (sx<0) sx=0; if (sx>=TFT_WIDTH) sx=TFT_WIDTH-1;
    if (sy<0) sy=0; if (sy>=TFT_HEIGHT) sy=TFT_HEIGHT-1;

    if (inClearBtn(sx,sy)){
      clearMain(); drawClearBtn(); gfx->fillRoundRect(CLEAR_X,CLEAR_Y,CLEAR_W,CLEAR_H,8, C_DIM);
      delay(120); drawClearBtn(); showHelp();
    }
    delay(8);
  }

  // 3) 헤더 상태 토글(대기)
  if (millis() - lastUIBlink > 3000){
    lastUIBlink = millis();
    waitingBlink = !waitingBlink;
    gfx->fillRect(250, 12, 70, 16, C_HDR);
    gfx->setCursor(250,12); gfx->setTextColor(waitingBlink?C_ACC:C_WHITE); gfx->setTextSize(1); gfx->setFont(nullptr);
    gfx->print(waitingBlink ? "Waiting" : "        ");
  }
}
