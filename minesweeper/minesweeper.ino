#include "Adafruit_LEDBackpack.h"
#include "Adafruit_GFX.h"
#include "Switch.h"
#include "FND.h"

#define FND_ADDR1  38
#define FND_ADDR2  32

#define JOY_X  68
#define JOY_Y  69
#define JOY_SEL  9

#define AMOUNT_MINES 10
#define FLAG_MASK 32
#define COVERED_MASK 16
#define NUM_MASK 15
#define MAX_MINES 30
#define XYTH 100
#define BLINK_RATE 100
#define CUR_INTERVAL_FAST 50
#define CUR_INTERVAL_SLOW 500
#define STATE_PREVIEW 0
#define STATE_RUNING 1
#define STATE_END 2

static uint8_t smileBmp[] = {0x3C, 0x42, 0x95, 0xA1, 0xA1, 0x95, 0x42, 0x3C};
static uint8_t frownBmp[] = {0x3C, 0x42, 0xA5, 0x91, 0x91, 0xA5, 0x42, 0x3C};
unsigned long gameElapsedTime = 0;
unsigned long prevMovingTime = 0;
unsigned long prevBlinkingTime = 0;
unsigned long delayInterval = CUR_INTERVAL_SLOW;
unsigned long curr_move = 0;
unsigned long curr_blink = 0;
boolean cur_moving = false;
boolean blinking = true;
boolean success = true;
boolean firstDraw = true;
int xCenterPos = 0;
int yCenterPos = 0;
int flagsNum = 0;
int uncoverNum = 0;
int curPos[4];
volatile int state = STATE_PREVIEW;
int mines = AMOUNT_MINES;
byte pool[8][8];


Adafruit_BicolorMatrix matrix = Adafruit_BicolorMatrix();
Switch pushSwitch;
FND fnd;

void setup(){
  Serial.begin(115200);
  fnd.begin(FND_ADDR1, FND_ADDR2);
  pushSwitch.begin();
  pinMode(JOY_X, INPUT);
  pinMode(JOY_Y, INPUT);
  pinMode(JOY_SEL, INPUT);
  matrix.begin(112);
  xCenterPos = analogRead(JOY_X);
  yCenterPos = analogRead(JOY_Y);
  attachInterrupt(4, selectInterrupt, RISING);
}

void selectInterrupt() {
  if(state != STATE_RUNING){
    state = (state+1) % 3;
    firstDraw = true;
  }
}

void clearPool(){
 for(int i=0;i<=7;i++){
  for(int j=0;j<=7;j++){
   pool[i][j] = 0; 
  }
 } 
}

void generatePool() {
  clearPool();
  randomSeed(analogRead(0));
  for(int i = 0; i<mines;){
    int x = random(0,7);
    int y = random(0,7);
    if(pool[x][y] != 9){
     pool[x][y] = 9;
     if(x-1 >= 0 && y-1 >=0 && pool[x-1][y-1] != 9){
       pool[x-1][y-1] = pool[x-1][y-1] + 1;
     }  
     if(x-1 >= 0 && pool[x-1][y] != 9){
       pool[x-1][y] = pool[x-1][y] + 1;
     }  
     if(x-1 >=0 && y+1 <=7 && pool[x-1][y+1] != 9){
       pool[x-1][y+1] = pool[x-1][y+1] + 1;
     }  
     if(y-1 >=0 && pool[x][y-1] != 9){
       pool[x][y-1] = pool[x][y-1] + 1;
     }   
     if(y+1 <=7 && pool[x][y+1] != 9){
       pool[x][y+1] = pool[x][y+1] + 1;
     }   
     if(x+1 <=7 && y-1 >= 0 && pool[x+1][y-1] != 9){
       pool[x+1][y-1] = pool[x+1][y-1] + 1;
     }  
     if(x+1 <=7 && pool[x+1][y] != 9){
      pool[x+1][y] = pool[x+1][y] + 1;
     }   
     if(x+1 <=7 && y+1 <=7 && pool[x+1][y+1] != 9){
       pool[x+1][y+1] = pool[x+1][y+1] + 1;
     }
     i++;
    }
  }
  printPool();
}

void printPool(){
 for(int i = 0; i<8; i++){
  for(int j = 0; j<8; j++) {
    Serial.print(pool[i][j]); 
    Serial.print(" ");
  }
  Serial.println();
 } 
 Serial.println("--------");
}

void draw(){
   if(!cur_moving) return;
   if(curPos[0] == curPos[2] && curPos[1] == curPos[3]) return;
   drawPoolValue(curPos[3],curPos[2]);

   matrix.drawPixel(curPos[0], curPos[1], 0);
   matrix.drawPixel(curPos[0], curPos[1], LED_RED);
   matrix.writeDisplay();
}

void reDraw(int x, int y){
  if(x == curPos[1] && y == curPos[0]) return;

  drawPoolValue(x, y);
  matrix.writeDisplay();
}

void drawPoolValue(int x, int y){
  matrix.drawPixel(y, x, 0);
  byte v = pool[x][y];
  if(COVERED_MASK != (v & COVERED_MASK)){
    if(FLAG_MASK == (v & FLAG_MASK)){
      matrix.drawPixel(y, x, LED_RED);
    }else{
      matrix.drawPixel(y, x, LED_YELLOW);
    }
  }else{
    int vv = v & NUM_MASK;
    if(vv != 0){
      matrix.drawPixel(y, x, LED_GREEN);
    }
  }
}

void loop(){
  unsigned long time = millis();
  if(state == STATE_PREVIEW){
    previewLoop();
  }else if(state == STATE_RUNING){
    runningLoop(time);
  }else if(state == STATE_END){
    endLoop(time);
  } 
}

boolean sleepWakable(int currState, unsigned long period){
  unsigned long start = millis();
  while(true){
    if(state != currState) return true;
    if((millis()-start) > period) return false;
    if(state == STATE_PREVIEW){
      
        switch(pushSwitch.getPushKey()){
          case 2: // ++
            mines = (mines+1) > MAX_MINES? mines: (mines+1);  
          break;
          case 1: // --
            mines = (mines-1) < 0? mines: (mines-1);
          break;
        }
      fnd.setAllNum(0,0,mines/10,mines%10);
    }
  }
}

void endLoop(unsigned long time){
  Serial.println("first Drawed");
  unsigned long gameTime = (time - gameElapsedTime) / 1000 / 60;
  fnd.setAllNum(gameTime/1000, gameTime/1000/100, gameTime/1000/100/10, gameTime/1000/100/10%10);

  matrix.setRotation(1);
  matrix.clear();
  if(success){
      matrix.drawBitmap(0, 0, smileBmp, 8, 8, LED_GREEN);
  }else{
      matrix.drawBitmap(0, 0, frownBmp, 8, 8, LED_YELLOW); 
  }
  matrix.writeDisplay();
  sleepWakable(STATE_END, 10000);
  matrix.setRotation(0);
  state = STATE_PREVIEW;
}

void previewLoop(){
  matrix.setTextWrap(false);
  matrix.setTextSize(1);
  matrix.setTextColor(LED_YELLOW);
  for(int x=7; x>=-70; x--) {
    matrix.clear();
    matrix.setCursor(x, 0);
    matrix.print("MINESWEEPER");
    matrix.writeDisplay();
    if(sleepWakable(STATE_PREVIEW, 200)) return;
  }
  
  matrix.setTextSize(1);
  matrix.setTextColor(LED_RED);
  for(int x=7; x>=-60; x--) {
    matrix.clear();
    matrix.setCursor(x, 0);
    matrix.print("By PengDU");
    matrix.writeDisplay();
    if(sleepWakable(STATE_PREVIEW, 200)) return;
  }
}


void runningLoop(unsigned long time){
  if(firstDraw){
   matrix.fillScreen(LED_YELLOW);
   generatePool();
   curPos[0] = 0;
   curPos[1] = 0;
   curPos[2] = 0;
   curPos[3] = 0;
   uncoverNum = 0;
   flagsNum = 0;
   gameElapsedTime = time;
   firstDraw = false;
  }
   int event = -1;
   if(cur_moving){
     if(time - prevMovingTime > delayInterval){
     prevMovingTime = time;
     event = event_handler();
     moveCur(event, 1);
     delayInterval = CUR_INTERVAL_FAST;
     }
   }else{
     event = event_handler();
     moveCur(event, 1);
     prevMovingTime = time;
     cur_moving = true;
     doBlink(time);
   }
   draw();

   if(event == 15){
     cur_moving = false;
     delayInterval = CUR_INTERVAL_SLOW;
   } 
   
   delay(50);
}

void doBlink(unsigned long t){
  if(t - prevBlinkingTime > BLINK_RATE){
    prevBlinkingTime = t;
    if(blinking){
      matrix.drawPixel(curPos[0], curPos[1], LED_OFF);
      blinking = false;
    }else{
      matrix.drawPixel(curPos[0], curPos[1], LED_ON);
      blinking = true;
    }
    matrix.writeDisplay();
  }
}

void moveCur(int dir, int offset){
  curPos[2] = curPos[0];
  curPos[3] = curPos[1];
  switch(dir){
    case 15: //C
    break;
    case 7: //R
      curPos[0] = curPos[0] + offset;
      break;
    case 5: //TR
      curPos[0] = curPos[0] + offset;
      curPos[1] = curPos[1] - offset;
      break;
    case 13: //T
      curPos[1] = curPos[1] - offset;
      break;
    case 9: //TL
      curPos[0] = curPos[0] - offset;
      curPos[1] = curPos[1] - offset;
    break;
    case 11: //L
      curPos[0] = curPos[0] - offset;
    break;
    case 10: //BL
      curPos[0] = curPos[0] - offset;
      curPos[1] = curPos[1] + offset;
      break;
    case 14: //B
      curPos[1] = curPos[1] + offset;
      break;
    case 6: //BR
      curPos[0] = curPos[0] + offset;
      curPos[1] = curPos[1] + offset;
      break;
  }
  
  if(curPos[0] > 7) curPos[0] =7;
  else if(curPos[0] < 0) curPos[0] = 0;
  if(curPos[1] > 7) curPos[1] = 7;
  else if(curPos[1] < 0) curPos[1] = 0;
  
  int value = pool[curPos[1]][curPos[0]];
  int num = value & NUM_MASK;
  if(value & COVERED_MASK){
    if(num >=0 && num <9){
      fnd.setAllNum(0,0,0,num);
    }
  }else{
    int mineLeft = mines - flagsNum;
    if(mineLeft < 0){
      fnd.setAllNum(0,0,(-mineLeft)/10,(-mineLeft)%10);
      if(mineLeft/10 != 0){      
         fnd.setFndVal(FND_ADDR1,REG_PORTA,0x71);
      }else{
        fnd.setFndVal(FND_ADDR1,REG_PORTB,0x71);
      }
    }else
      fnd.setAllNum(0,0,mineLeft/10,mineLeft%10);
    
   
  }

}

void dig(int x, int y){
  byte value = pool[x][y];
  if(COVERED_MASK == (value & COVERED_MASK)){
    return;
  }else{
    uncoverNum++;
    value = value & NUM_MASK;
    if(value == 9){
      state = STATE_END;
      success = false;
     Serial.println("Game Over"); 
    }else if(value >= 0 && value < 9){
      pool[x][y] |= COVERED_MASK;
      fnd.setAllNum(0,0,0,value);
      if(value == 0)
         digAround(x, y);
    }
  }
}

void digAround(int x, int y){
  for(int i=-1;i<=1;i++){
   for(int j=-1;j<=1;j++){
      if(i==0&&j==0) continue;
      if(x+i < 0 || x+i>7 || y+j<0 || y+j >7) continue;
      byte value = pool[x+i][y+j];
      if(COVERED_MASK == (value & COVERED_MASK)) continue;
      if((value & FLAG_MASK) == FLAG_MASK) continue;
      pool[x+i][y+j] |= COVERED_MASK;
      uncoverNum++;
      value = value & NUM_MASK;
      if(value == 0) digAround(x+i, y+j);
      reDraw(x+i,y+j);
   }
  }
}

void toggleFlag(){
  byte v = pool[curPos[1]][curPos[0]];
  if(COVERED_MASK == (v & COVERED_MASK)) return;
  if((v & FLAG_MASK) == FLAG_MASK){
    pool[curPos[1]][curPos[0]] &= ~FLAG_MASK;
    flagsNum--;
  }else{
    pool[curPos[1]][curPos[0]] |= FLAG_MASK;
    flagsNum++;
  }
  Serial.println(flagsNum);
}

void checkSuccess(){
  if(flagsNum == mines && uncoverNum == 64-mines){
    Serial.println("Checking...");
    success = true;
    for(int i=0; i<7;i++){
     for(int j=0; j<7; j++){
      byte v = pool[i][j];
      if(0 == (v & COVERED_MASK)){
        if((v & FLAG_MASK) == 0){
          success = false;
          break;
        }
      }
     }
     if(!success) break;
    }
    firstDraw = true;
    state = STATE_END;
  }
}

int event_handler(){
  switch(pushSwitch.getPushKey()){
    case 1: // dig
      dig(curPos[1], curPos[0]);
      checkSuccess();
      break;
    case 2: // flag
      toggleFlag();
      checkSuccess();
      break;
    case 16:
      printPool();
      break;
    case 5: //TL
      return 1|(1<<3);
    case 6: //T
      return 1|(1<<2)|(1<<3);
    case 7: //TR
      return 1|(1<<2);
    case 9: //L
      return (1<<3)|(1<<1)|1;
    case 10: //CC
      return 1|(1<<1)|(1<<2)|(1<<3);
    case 11: //R
      return (1<<2)|(1<<1)|1;
    case 13: //BL
      return (1<<1)|(1<<3);
    case 14: //B
      return (1<<1)|(1<<2)|(1<<3);
    case 15: //BR
      return (1<<1)|(1<<2);
  }
  return joyStick_event();
}

int joyStick_event(){
  int dir;
  if(analogRead(JOY_X) < xCenterPos - XYTH) {
    dir |= 1;// Up
  } else if(analogRead(JOY_X) > xCenterPos + XYTH) {
    dir |= 1<<1;// Down
  } else {
    dir |= (1<<1)|1 ;
  }

  if(analogRead(JOY_Y) < yCenterPos - XYTH) {
    dir |= 1<<2;// Right
  } else if(analogRead(JOY_Y) > yCenterPos + XYTH) {
    dir |= 1<<3;//Left
  } else {
    dir |= (1<<2)|(1<<3);
  }

  if(digitalRead(JOY_SEL) == LOW) {
    dir |= 1<<4;
  }
 return dir; 
}
