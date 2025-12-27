#include <DHT.h>
#include <LiquidCrystal_I2C.h>
#include <EEPROM.h>

// ---------------------- Pinout ----------------------
#define DHTPIN_IN       7   // Inside
#define DHTPIN_OUT      8   // Outside
#define FAN1_PWM        9
#define FAN1_TACH       2
#define FAN2_PWM       10
#define FAN2_TACH       3
#define BTN_MENUE      A1
#define BTN_DOWN       A2
#define BTN_UP         A3
#define FORCE_FAN_PIN   4   // Force 100% PWM
#define PREHEAT_PIN     5
#define LED_PIN         6   

DHT dht_in(DHTPIN_IN, DHT22);
DHT dht_out(DHTPIN_OUT, DHT22);
LiquidCrystal_I2C lcd(0x27,16,2);

// ------------------- Button handling (edge based) -------------------
struct Button {
  bool stableState;
  bool lastReading;
  unsigned long lastChange;
  unsigned long pressStart;
  bool longHandled;
  unsigned long lastRepeat;   // NEW: for repeat timing
};


Button btnUp   = {HIGH, HIGH, 0, 0, false, 0};
Button btnDown = {HIGH, HIGH, 0, 0, false, 0};
Button btnMenu = {HIGH, HIGH, 0, 0, false, 0};

const unsigned long DEBOUNCE_TIME = 40;

#define REPEAT_DELAY 500   // ms until repeat starts
#define REPEAT_RATE  120   // ms between repeats

// ------------------- Global sensor variables -------------------
float tin=NAN, tout=NAN, hin=NAN, hout=NAN;

int hinLow=0, hinHigh=0, houtLow=0, houtHigh=0;
float hinSlope=1.0, hinOffset=0.0;
float houtSlope=1.0, houtOffset=0.0;

// ------------------- LED Parameters -------------------
int ledMax = 10;
int ledMin = 0;
unsigned long blinkSlowOn = 50, blinkSlowOff = 5000;
unsigned long blinkFastOn = 100, blinkFastOff = 200;

unsigned long pulsePeriod = 5000;

float ledBrightness = 0;
bool ledIncreasing = true;
unsigned long lastBlink = 0;
bool ledState = false;

// ----------------- Control Parameters -----------------
float Ki = 0.2, Kd = 1.0, Kp = 5.0;
volatile unsigned long tach1Counter = 0;
volatile unsigned long tach2Counter = 0;
unsigned long lastRPMcalc = 0;
int rpm1 = 0, rpm2 = 0;

int offset = 5;
int minPWM_percent = 20;
int maxPWM_percent = 100;
int fanOffset = 0;

int FanOffHumidity = 40;
int MinHumidity = 50;
int PreheatTemp = 0;

bool sensorReady = false;
bool fansStopped = false;

// ------------------ PID State ----------------
float integral = 0.0;
float lastError = 0.0;
unsigned long lastPID = 0;

// ------------------- Menu / Display -------------------
bool displayOn = true;
unsigned long lastInputTime = 0;
bool inMenu = false;
int menuIndex = 0;
bool menuNeedsRedraw = true;

// ------------------- EEPROM Addresses -------------------
#define ADDR_OFFSET     0
#define ADDR_MINPWM     2
#define ADDR_MAXPWM     4
#define ADDR_KP         6
#define ADDR_FANOFFSET  10
#define ADDR_KI         12
#define ADDR_KD         16
#define ADDR_FANOFFH    20
#define ADDR_MINHUM     22
#define ADDR_PREHEAT    24

#define LONG_PRESS_TIME 1500

#define ADDR_HIN_LOW     26
#define ADDR_HIN_HIGH    28
#define ADDR_HOUT_LOW    30
#define ADDR_HOUT_HIGH   32

// ------------------- ISR -------------------
void tach1ISR() { tach1Counter++; }
void tach2ISR() { tach2Counter++; }

// ------------------- Setup -------------------
void setup() {
  pinMode(FAN1_PWM, OUTPUT);
  pinMode(FAN2_PWM, OUTPUT);
  pinMode(FAN1_TACH, INPUT_PULLUP);
  pinMode(FAN2_TACH, INPUT_PULLUP);
  pinMode(BTN_UP, INPUT_PULLUP);
  pinMode(BTN_DOWN, INPUT_PULLUP);
  pinMode(BTN_MENUE, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);
  pinMode(PREHEAT_PIN, OUTPUT);
  pinMode(FORCE_FAN_PIN, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(FAN1_TACH), tach1ISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(FAN2_TACH), tach2ISR, FALLING);

  lcd.init();
  lcd.backlight();

  dht_in.begin();
  dht_out.begin();

  // Load EEPROM
  EEPROM.get(ADDR_OFFSET, offset);
  EEPROM.get(ADDR_MINPWM, minPWM_percent);
  EEPROM.get(ADDR_MAXPWM, maxPWM_percent);
  EEPROM.get(ADDR_KP, Kp);
  EEPROM.get(ADDR_FANOFFSET, fanOffset);
  EEPROM.get(ADDR_KI, Ki);
  EEPROM.get(ADDR_KD, Kd);
  EEPROM.get(ADDR_FANOFFH, FanOffHumidity);
  EEPROM.get(ADDR_MINHUM, MinHumidity);
  EEPROM.get(ADDR_PREHEAT, PreheatTemp);
  EEPROM.get(ADDR_HIN_LOW, hinLow);
  EEPROM.get(ADDR_HIN_HIGH, hinHigh);
  EEPROM.get(ADDR_HOUT_LOW, houtLow);
  EEPROM.get(ADDR_HOUT_HIGH, houtHigh);

  // Steigung/Offset berechnen
if(hinHigh - hinLow != 0) hinSlope = 75.0 / (hinHigh - hinLow);
else hinSlope = 1.0;
hinOffset = -hinLow * hinSlope;

if(houtHigh - houtLow != 0) houtSlope = 75.0 / (houtHigh - houtLow);
else houtSlope = 1.0;
houtOffset = -houtLow * houtSlope;

  // Timer1 PWM setup
  TCCR1A = 0; TCCR1B = 0;
  TCCR1A |= (1<<COM1A1)|(1<<COM1B1)|(1<<WGM11);
  TCCR1B |= (1<<WGM13)|(1<<WGM12)|(1<<CS10);
  ICR1 = 640;
  OCR1A = 0;
  OCR1B = 0;

  digitalWrite(FAN1_PWM, LOW);
  digitalWrite(FAN2_PWM, LOW);
}

// ------------------- Main Loop -------------------
void loop() {
  unsigned long now = millis();

  // ------------------- Read Sensors -------------------
  static unsigned long lastRead = 0;
  if (now - lastRead >= 3000) {
    lastRead = now;
    float hInNew = dht_in.readHumidity();
    float hOutNew = dht_out.readHumidity();
    float tInNew = dht_in.readTemperature();
    float tOutNew = dht_out.readTemperature();

    if (!isnan(hInNew)) hin = hInNew * hinSlope + hinOffset;
    if (!isnan(hOutNew)) hout = hOutNew * houtSlope + houtOffset;
    if (!isnan(tInNew)) tin = tInNew;
    if (!isnan(tOutNew)) tout = tOutNew;

    // Sensor is ready only if both humidity sensors deliver valid data
    if (!isnan(hInNew) && !isnan(hOutNew)) {
      sensorReady = true;
  } else {
      sensorReady = false;
}
  }

  // ------------------- Calculate RPM -------------------
  if (now - lastRPMcalc > 3000) {
    noInterrupts();
    unsigned long count1 = tach1Counter; tach1Counter=0;
    unsigned long count2 = tach2Counter; tach2Counter=0;
    interrupts();

    rpm1 = (count1*60000UL)/(2*(now - lastRPMcalc));
    rpm2 = (count2*60000UL)/(2*(now - lastRPMcalc));
    lastRPMcalc = now;
    menuNeedsRedraw = true;
  }

  handleButtons(now);

// ------------------- Display Timeout + Auto-Exit Menu -------------------
if(displayOn) {
    // Menü automatisch verlassen kurz vor Display-Off
    if(inMenu && now - lastInputTime > 10000){ // 10 Sekunden
        inMenu = false;
        EEPROM.put(ADDR_MINPWM,minPWM_percent);
        EEPROM.put(ADDR_MAXPWM,maxPWM_percent);
        EEPROM.put(ADDR_KP,Kp);
        EEPROM.put(ADDR_KI,Ki);
        EEPROM.put(ADDR_KD,Kd);
        EEPROM.put(ADDR_OFFSET,offset);
        EEPROM.put(ADDR_FANOFFSET,fanOffset);
        EEPROM.put(ADDR_FANOFFH,FanOffHumidity);
        EEPROM.put(ADDR_MINHUM,MinHumidity);
        EEPROM.put(ADDR_PREHEAT,PreheatTemp);
        EEPROM.put(ADDR_HIN_LOW, hinLow);
        EEPROM.put(ADDR_HIN_HIGH, hinHigh);
        EEPROM.put(ADDR_HOUT_LOW, houtLow);
        EEPROM.put(ADDR_HOUT_HIGH, houtHigh);
        menuNeedsRedraw = true;
    }

    // Display ausschalten nach 30 Sekunden
    if(now - lastInputTime > 30000){
        lcd.noDisplay();
        lcd.noBacklight();
        displayOn = false;
    }
}

  // ------------------- Preheat -------------------
  static bool preheatActive = false;
  if (!preheatActive && tout <= PreheatTemp) preheatActive = true;
  else if (preheatActive && tout >= PreheatTemp + 1) preheatActive = false;
  digitalWrite(PREHEAT_PIN, preheatActive ? HIGH : LOW);

  // ------------------- Force Fan -------------------
  bool forceFanActive = (digitalRead(FORCE_FAN_PIN) == LOW);
  if (forceFanActive) {
    int pwmVal = map(maxPWM_percent, 0, 100, 0, ICR1);
    TCCR1A |= (1<<COM1A1)|(1<<COM1B1);
    OCR1A = pwmVal;
    OCR1B = pwmVal;

    if (now - lastBlink >= (ledState ? blinkFastOn : blinkFastOff)) {
      ledState = !ledState;
      analogWrite(LED_PIN, ledState ? ledMax : 0);
      lastBlink = now;
    }
    return;
  }

  // ------------------- FanOff Condition -------------------
  bool fanOffCondition = sensorReady && !isnan(hin) && (hin < FanOffHumidity);
  if (fanOffCondition) {
    if(!fansStopped) { // Only stop once
      fansStopped = true;
      stopFans();
    }
  } else if (fansStopped && hin > MinHumidity) {
    fansStopped = false;
    integral = 0;
    lastError = 0;
  }

  // ------------------- PID Fan Control -------------------
  if (!fansStopped) {
    float dt = max((now - lastPID)/1000.0, 0.001);
    float error = (hin - hout) - offset;

    float predictedOut = Kp*error + Ki*integral;
    if (predictedOut > 0 && predictedOut < 255) integral += error*dt;
    integral = constrain(integral, -300, 300);

    float derivative = (error - lastError)/dt;
    float output = Kp*error + Ki*integral + Kd*derivative;

    float pwm1 = constrain(minPWM_percent + output, minPWM_percent, maxPWM_percent);
    float pwm2 = constrain(pwm1 + fanOffset, 0, 100);

    OCR1A = map((int)pwm1,0,100,0,ICR1);
    OCR1B = map((int)pwm2,0,100,0,ICR1);
    TCCR1A |= (1<<COM1A1)|(1<<COM1B1);

    lastError = error;
    lastPID = now;
  }

  // ------------------- LED Logic -------------------
  updateLED(now);

  // ------------------- Display Update -------------------
  if(menuNeedsRedraw && displayOn) updateDisplay();
}

// ------------------- Functions -------------------
void stopFans() {
  TCCR1A &= ~((1<<COM1A1)|(1<<COM1B1));
  OCR1A = 0;
  OCR1B = 0;
  digitalWrite(FAN1_PWM, LOW);
  digitalWrite(FAN2_PWM, LOW);
}

void updateLED(unsigned long now) {
  if (inMenu) { 
    analogWrite(LED_PIN, 0); 
    return; 
  }

  if (!sensorReady) { // Fast blink
    if (now - lastBlink >= (ledState ? blinkFastOn : blinkFastOff)) {
      ledState = !ledState;
      analogWrite(LED_PIN, ledState ? ledMax : ledMin); // Max/Min statt full PWM
      lastBlink = now;
    }
  } 
  else if (fansStopped) { // Slow blink
    if (now - lastBlink >= (ledState ? blinkSlowOn : blinkSlowOff)) {
      ledState = !ledState;
      analogWrite(LED_PIN, ledState ? ledMax : ledMin); // Max/Min statt full PWM
      lastBlink = now;
    }
  } 
  else { // Pulsieren
    float t = (float)(now % pulsePeriod) / pulsePeriod; // 0..1 innerhalb Periode
    // Sinusförmig zwischen ledMin und ledMax
    ledBrightness = ledMin + (ledMax - ledMin) * 0.5 * (1 + sin(2 * PI * t - PI/2)); 
    analogWrite(LED_PIN, (int)ledBrightness);
  }
}

// Returns true ONLY on clean HIGH->LOW transition
bool buttonEdge(Button &b, int pin, unsigned long now){
  bool reading = digitalRead(pin);

  if(reading != b.lastReading){
    b.lastChange = now;
    b.lastReading = reading;
  }

  if(now - b.lastChange > DEBOUNCE_TIME){
    // HIGH -> LOW (press)
    if(b.stableState == HIGH && reading == LOW){
      b.stableState = LOW;
      b.pressStart = now;
      b.longHandled = false;
      return true;
    }

    // LOW -> HIGH (release)
    if(b.stableState == LOW && reading == HIGH){
      b.stableState = HIGH;
      b.longHandled = false;
    }
  }
  return false;
}


void updateDisplay() {
  lcd.clear();
  if(!inMenu) {
    // ------------------- Normal Display -------------------
    lcd.setCursor(0,0);
    lcd.print((int)tin); lcd.print("C|");
    lcd.print((int)tout); lcd.print("C  ");
    lcd.print((int)hin); lcd.print("%|");
    lcd.print((int)hout); lcd.print("%");

    lcd.setCursor(0,1);
    lcd.print("In:"); lcd.print(rpm2);
    lcd.setCursor(8,1); lcd.print("Out:"); lcd.print(rpm1);
  } else {
    // ------------------- Menu Display -------------------
    lcd.setCursor(0,0);
    if(menuIndex==0) {
        lcd.print(">MinPWM:"); 
        lcd.print(minPWM_percent);
    } else if(menuIndex==1) {
        lcd.print(">MaxPWM:"); 
        lcd.print(maxPWM_percent);
    } else if(menuIndex==2) {
        lcd.print(">Kp:"); 
        lcd.print(Kp,1);
    } else if(menuIndex==3) {
        lcd.print(">Ki:"); 
        lcd.print(Ki,2);
    } else if(menuIndex==4) {
        lcd.print(">Kd:"); 
        lcd.print(Kd,2);
    } else if(menuIndex==5) {
        lcd.print(">Offset:"); 
        lcd.print(offset);
    } else if(menuIndex==6) {
        lcd.print(">FanOffset:"); 
        lcd.print(fanOffset);
    } else if(menuIndex==7) {
        lcd.print(">FanOffHum:"); 
        lcd.print(FanOffHumidity);
    } else if(menuIndex==8) {
        lcd.print(">MinHum:"); 
        lcd.print(MinHumidity);
    } else if(menuIndex==9) {
        lcd.print(">Preheat:"); 
        lcd.print(PreheatTemp);
    } else if(menuIndex==10){
        lcd.print(">Cal HinL:");
        lcd.print((int)hinLow);
    } else if(menuIndex==11){
        lcd.print(">Cal HinH:");
        lcd.print((int)hinHigh);
    } else if(menuIndex==12){
        lcd.print(">Cal HoutL:");
        lcd.print((int)houtLow);
    } else if(menuIndex==13){
        lcd.print(">Cal HoutH:");
        lcd.print((int)houtHigh);
}

    lcd.setCursor(0,1);
    lcd.print("In:"); lcd.print(rpm2);
    lcd.setCursor(8,1); lcd.print("Out:"); lcd.print(rpm1);
  }
  menuNeedsRedraw = false;
}


void handleButtons(unsigned long now){

  // -------- Display wake --------
  if(!displayOn){
    if(digitalRead(BTN_UP)==LOW || digitalRead(BTN_DOWN)==LOW || digitalRead(BTN_MENUE)==LOW){
      lcd.display();
      lcd.backlight();
      displayOn = true;
      lastInputTime = now;
      delay(200);
    }
    return;
  }

// -------- UP --------
if(buttonEdge(btnUp, BTN_UP, now)){
  lastInputTime = now;
  incrementMenuValue();              // first step
  btnUp.lastRepeat = now;
}

// Repeat
if(btnUp.stableState == LOW &&
   now - btnUp.pressStart >= REPEAT_DELAY &&
   now - btnUp.lastRepeat >= REPEAT_RATE){

  btnUp.lastRepeat = now;
  lastInputTime = now;
  incrementMenuValue();
}

// -------- DOWN --------
if(buttonEdge(btnDown, BTN_DOWN, now)){
  lastInputTime = now;
  decrementMenuValue();              // first step
  btnDown.lastRepeat = now;
}

// Repeat
if(btnDown.stableState == LOW &&
   now - btnDown.pressStart >= REPEAT_DELAY &&
   now - btnDown.lastRepeat >= REPEAT_RATE){

  btnDown.lastRepeat = now;
  lastInputTime = now;
  decrementMenuValue();
}


  // -------- MENU short press --------
  if(buttonEdge(btnMenu, BTN_MENUE, now)){
    lastInputTime = now;
    if(inMenu){
      menuIndex = (menuIndex + 1) % 14;
      menuNeedsRedraw = true;
    }
  }

  // -------- MENU long press --------
if(btnMenu.stableState == LOW &&
   !btnMenu.longHandled &&
   now - btnMenu.pressStart >= LONG_PRESS_TIME){

  btnMenu.longHandled = true;
  lastInputTime = now;

  if(!inMenu){
    inMenu = true;
    menuIndex = 0;
  } else {
    inMenu = false;
    EEPROM.put(ADDR_MINPWM,minPWM_percent);
    EEPROM.put(ADDR_MAXPWM,maxPWM_percent);
    EEPROM.put(ADDR_KP,Kp);
    EEPROM.put(ADDR_KI,Ki);
    EEPROM.put(ADDR_KD,Kd);
    EEPROM.put(ADDR_OFFSET,offset);
    EEPROM.put(ADDR_FANOFFSET,fanOffset);
    EEPROM.put(ADDR_FANOFFH,FanOffHumidity);
    EEPROM.put(ADDR_MINHUM,MinHumidity);
    EEPROM.put(ADDR_PREHEAT,PreheatTemp);
    EEPROM.put(ADDR_HIN_LOW, hinLow);
    EEPROM.put(ADDR_HIN_HIGH, hinHigh);
    EEPROM.put(ADDR_HOUT_LOW, houtLow);
    EEPROM.put(ADDR_HOUT_HIGH, houtHigh);
  }
  menuNeedsRedraw = true;
}
// Slope/Offset neu berechnen
if(hinHigh - hinLow != 0) hinSlope = 75.0 / (hinHigh - hinLow);
else hinSlope = 1.0;
hinOffset = -hinLow * hinSlope;

if(houtHigh - houtLow != 0) houtSlope = 75.0 / (houtHigh - houtLow);
else houtSlope = 1.0;
houtOffset = -houtLow * houtSlope;

}


// ------------------- Helper functions -------------------
void incrementMenuValue(){
  if(!inMenu) return;
  if(menuIndex==0 && minPWM_percent<100) minPWM_percent++;
  else if(menuIndex==1 && maxPWM_percent<100) maxPWM_percent++;
  else if(menuIndex==2) Kp+=0.1;
  else if(menuIndex==3) Ki+=0.05;
  else if(menuIndex==4) Kd+=0.1;
  else if(menuIndex==5) offset++;
  else if(menuIndex==6 && fanOffset<50) fanOffset++;
  else if(menuIndex==7 && FanOffHumidity<100) FanOffHumidity++;
  else if(menuIndex==8 && MinHumidity<100) MinHumidity++;
  else if(menuIndex==9 && PreheatTemp<50) PreheatTemp++;
  else if(menuIndex==10) hinLow = min(hinLow + 1, 100);
  else if(menuIndex==11) hinHigh = min(hinHigh + 1, 100);
  else if(menuIndex==12) houtLow = min(houtLow + 1, 100);
  else if(menuIndex==13) houtHigh = min(houtHigh + 1, 100);
  menuNeedsRedraw = true;
}

void decrementMenuValue(){
  if(!inMenu) return;
  if(menuIndex==0 && minPWM_percent>0) minPWM_percent--;
  else if(menuIndex==1 && maxPWM_percent>0) maxPWM_percent--;
  else if(menuIndex==2 && Kp>0.1) Kp-=0.1;
  else if(menuIndex==3 && Ki>0.0) Ki-=0.05;
  else if(menuIndex==4 && Kd>0.0) Kd-=0.1;
  else if(menuIndex==5) offset--;
  else if(menuIndex==6 && fanOffset>0) fanOffset--;
  else if(menuIndex==7 && FanOffHumidity>0) FanOffHumidity--;
  else if(menuIndex==8 && MinHumidity>0) MinHumidity--;
  else if(menuIndex==9 && PreheatTemp>-50) PreheatTemp--;
  else if(menuIndex==10) hinLow = max(hinLow - 1, 0);
  else if(menuIndex==11) hinHigh = max(hinHigh - 1, 0);
  else if(menuIndex==12) houtLow = max(houtLow - 1, 0);
  else if(menuIndex==13) houtHigh = max(houtHigh - 1, 0);

  menuNeedsRedraw = true;
}

