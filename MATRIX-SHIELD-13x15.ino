#include <SPI.h>
#include <Wire.h>
#include "MPU6050.h"
#include "font3x5.h"
#include "font5x7.h"
#include "font8x8.h"
#include "font8x16.h"
#include "vnfont8x16.h"

#define RowA3_Pin       8   // PH5
#define RowA2_Pin       6   // PH3
#define RowA1_Pin       5   // PE3
#define RowA0_Pin       4   // PG5

#define BLANK_Pin       2   // PE4
#define LATCH_Pin       3   // PE5
#define DATA_Pin        51   
#define CLOCK_Pin       52


#define N_GRAINS     65   // Number of grains of sand
#define WIDTH        15   // Display width in pixels
#define HEIGHT       13   // Display height in pixels
#define MAX_FPS      45   // Maximum redraw rate, frames/second
#define SCALE        128

// The 'sand' grains exist in an integer coordinate space that's 256X
// the scale of the pixel grid, allowing them to move and interact at
// less than whole-pixel increments.
#define MAX_X (WIDTH  * SCALE - 1) // Maximum X coordinate in grain space
#define MAX_Y (HEIGHT * SCALE - 1) // Maximum Y coordinate
struct Grain {
  int16_t  x,  y; // Position
  int16_t vx, vy; // Velocity
  uint16_t pos;
  uint16_t colour;
} grain[N_GRAINS];


MPU6050 mpu;
uint32_t      prevTime   = 0;      // Used for frames-per-second throttle
uint16_t      img[WIDTH * HEIGHT]; // Internal 'map' of pixels

float xOffset = -1350; 
float yOffset = -2590;

#define BAM_RESOLUTION  4

unsigned long samplingtime  = 0;

byte row, level;
int BAM_Bit, BAM_Counter=0;   // Bit Angle Modulation variables to keep track of things
uint8_t R;

byte matrixBuffer[BAM_RESOLUTION][26];

#define dist(a, b, c, d) sqrt(double((a - c) * (a - c) + (b - d) * (b - d)))

#define FONT3x5         4
#define FONT5x7         0
#define FONTVN8x16      1
#define FONT8x8         2
#define FONTEN8x16      3

wchar_t scrolltext_1[]=L"     LED MATRIX SHIELD 13x15 FOR ARDUINO MEGA/UNO...     ";
wchar_t scrolltext_2[]=L"     SAND FALLING GRAVITY EFFECTS - STARTING...     ";
void setup () 
{
  uint8_t i, j, bytes;
  row = 0;
  level = 0;
  
  noInterrupts();
  TCCR1A = B00000000;
  TCCR1B = B00001011;
  TIMSK1 = B00000010;
  OCR1A = 10;
  
  pinMode(DATA_Pin, OUTPUT);
  pinMode(CLOCK_Pin, OUTPUT);
  pinMode(LATCH_Pin, OUTPUT);
  //pinMode(BLANK_Pin, OUTPUT);
  pinMode(RowA0_Pin, OUTPUT);
  pinMode(RowA1_Pin, OUTPUT);
  pinMode(RowA2_Pin, OUTPUT);
  pinMode(RowA3_Pin, OUTPUT);

  SPI.begin();
  Wire.begin();
  interrupts();
  clearscreen();

}    

void loop()
{
 
  hScroll(0, 15, 0, scrolltext_1, FONTEN8x16, 40, 1, 1); //void hScroll(uint8_t y, Color For_color, Color Bk_color, wchar_t *mystring, uint8_t font, uint8_t delaytime, uint8_t times, uint8_t dir) 
  clearscreen();
  hScroll(0, 15, 2, scrolltext_2, FONTEN8x16, 40, 1, 1); //void hScroll(uint8_t y, Color For_color, Color Bk_color, wchar_t *mystring, uint8_t font, uint8_t delaytime, uint8_t times, uint8_t dir) 
  clearscreen();
  while (!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_4G, MPU6050_ADDRESS))
  {
    //Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    delay(500);
  }

  Vector accelVector = mpu.readRawAccel();

  float xOffset = (accelVector.XAxis * -1) * -1;
  float yOffset = (accelVector.YAxis * -1) * -1;
  
  while (true) {
  uint32_t t;
  while (((t = micros()) - prevTime) < (1000000L / MAX_FPS));
  prevTime = t;

  Vector accelVector = mpu.readRawAccel();

  float accelX = (accelVector.XAxis * -1) + xOffset;
  float accelY = (accelVector.YAxis * -1) + yOffset;
  float accelZ = accelVector.ZAxis;

  int16_t ax =  accelX / SCALE,       // Transform accelerometer axes
          ay =  accelY / SCALE,      // to grain coordinate space
          az = abs(accelZ) / 2048;  // Random motion factor
  az = (az >= 3) ? 1 : 4 - az;      // Clip & invert
  ax -= az;                         // Subtract motion factor from X, Y
  ay -= az;
  int16_t az2 = az * 2 + 1;         // Range of random motion to add back in

  // ...and apply 2D accel vector to grain velocities...
  int32_t v2; // Velocity squared
  float   v;  // Absolute velocity
  for (int i = 0; i < N_GRAINS; i++) {
    grain[i].vx += ax + random(az2); // A little randomness makes
    grain[i].vy += ay + random(az2); // tall stacks topple better!
    // Terminal velocity (in any direction) is 256 units -- equal to
    // 1 pixel -- which keeps moving grains from passing through each other
    // and other such mayhem.  Though it takes some extra math, velocity is
    // clipped as a 2D vector (not separately-limited X & Y) so that
    // diagonal movement isn't faster
    v2 = (int32_t)grain[i].vx * grain[i].vx + (int32_t)grain[i].vy * grain[i].vy;
    if (v2 > 65536) { // If v^2 > 65536, then v > 256
      v = sqrt((float)v2); // Velocity vector magnitude
      grain[i].vx = (int)(SCALE * (float)grain[i].vx / v); // Maintain heading
      grain[i].vy = (int)(SCALE * (float)grain[i].vy / v); // Limit magnitude
    }
  }

  // ...then update position of each grain, one at a time, checking for
  // collisions and having them react.  This really seems like it shouldn't
  // work, as only one grain is considered at a time while the rest are
  // regarded as stationary.  Yet this naive algorithm, taking many not-
  // technically-quite-correct steps, and repeated quickly enough,
  // visually integrates into something that somewhat resembles physics.
  // (I'd initially tried implementing this as a bunch of concurrent and
  // "realistic" elastic collisions among circular grains, but the
  // calculations and volument of code quickly got out of hand for both
  // the tiny 8-bit AVR microcontroller and my tiny dinosaur brain.)

  uint16_t       i, bytes, oldidx, newidx, delta;
  int16_t        newx, newy;

  for (i = 0; i < N_GRAINS; i++) {
    newx = grain[i].x + grain[i].vx; // New position in grain space
    newy = grain[i].y + grain[i].vy;
    if (newx > MAX_X) {              // If grain would go out of bounds
      newx         = MAX_X;          // keep it inside, and
      grain[i].vx /= -2;             // give a slight bounce off the wall
    } else if (newx < 0) {
      newx         = 0;
      grain[i].vx /= -2;
    }
    if (newy > MAX_Y) {
      newy         = MAX_Y;
      grain[i].vy /= -2;
    } else if (newy < 0) {
      newy         = 0;
      grain[i].vy /= -2;
    }

    oldidx = (grain[i].y / SCALE) * WIDTH + (grain[i].x / SCALE); // Prior pixel #
    newidx = (newy      / SCALE) * WIDTH + (newx      / SCALE); // New pixel #
    if ((oldidx != newidx) &&         // If grain is moving to a new pixel...
        img[newidx]) {                // but if that pixel is already occupied...
      delta = abs(newidx - oldidx);   // What direction when blocked?
      if (delta == 1) {               // 1 pixel left or right)
        newx         = grain[i].x;    // Cancel X motion
        grain[i].vx /= -2;            // and bounce X velocity (Y is OK)
        newidx       = oldidx;        // No pixel change
      } else if (delta == WIDTH) {    // 1 pixel up or down
        newy         = grain[i].y;    // Cancel Y motion
        grain[i].vy /= -2;            // and bounce Y velocity (X is OK)
        newidx       = oldidx;        // No pixel change
      } else { // Diagonal intersection is more tricky...
        // Try skidding along just one axis of motion if possible (start w/
        // faster axis).  Because we've already established that diagonal
        // (both-axis) motion is occurring, moving on either axis alone WILL
        // change the pixel index, no need to check that again.
        if ((abs(grain[i].vx) - abs(grain[i].vy)) >= 0) { // X axis is faster
          newidx = (grain[i].y / SCALE) * WIDTH + (newx / SCALE);
          if (!img[newidx]) {             // That pixel's free!  Take it!  But...
            newy         = grain[i].y;    // Cancel Y motion
            grain[i].vy /= -2;            // and bounce Y velocity
          } else { // X pixel is taken, so try Y...
            newidx = (newy / SCALE) * WIDTH + (grain[i].x / SCALE);
            if (!img[newidx]) {           // Pixel is free, take it, but first...
              newx         = grain[i].x;  // Cancel X motion
              grain[i].vx /= -2;          // and bounce X velocity
            } else { // Both spots are occupied
              newx         = grain[i].x;  // Cancel X & Y motion
              newy         = grain[i].y;
              grain[i].vx /= -2;          // Bounce X & Y velocity
              grain[i].vy /= -2;
              newidx       = oldidx;      // Not moving
            }
          }
        } else { // Y axis is faster, start there
          newidx = (newy / SCALE) * WIDTH + (grain[i].x / SCALE);
          if (!img[newidx]) { // Pixel's free!  Take it!  But...
            newx         = grain[i].x;    // Cancel X motion
            grain[i].vy /= -2;            // and bounce X velocity
          } else { // Y pixel is taken, so try X...
            newidx = (grain[i].y / SCALE) * WIDTH + (newx / SCALE);
            if (!img[newidx]) { // Pixel is free, take it, but first...
              newy         = grain[i].y; // Cancel Y motion
              grain[i].vy /= -2;         // and bounce Y velocity
            } else { // Both spots are occupied
              newx         = grain[i].x; // Cancel X & Y motion
              newy         = grain[i].y;
              grain[i].vx /= -2;         // Bounce X & Y velocity
              grain[i].vy /= -2;
              newidx       = oldidx;     // Not moving
            }
          }
        }
      }
    }
    grain[i].x  = newx; // Update grain position
    grain[i].y  = newy;
    img[oldidx] = 0;    // Clear old spot (might be same as new, that's OK)
    img[newidx] = 15;  // Set new spot
    grain[i].pos = newidx;
    //Serial.println(newidx);
  }
    for(i=0; i<WIDTH*HEIGHT; i++) {
    LED(i % WIDTH, i / WIDTH, img[i]);
    }
  } 
}

void LED(uint8_t X, uint8_t Y, uint8_t RR)
{
  
  //RR = constrain(RR, 0, (1 << BAM_RESOLUTION) - 1);
  
  uint8_t whichbyte = ((Y*2)+X/8);
  uint8_t whichbit = 7-(X % 8);
  
  for (byte BAM = 0; BAM < BAM_RESOLUTION; BAM++) 
  {
    bitWrite(matrixBuffer[BAM][whichbyte], whichbit, bitRead(RR, BAM));
  }
}


void fillTable(uint8_t BLUE)
{
    for (byte x=0; x<15; x++)
    {
      for (byte y=0; y<13; y++)
      {
        LED(x, y, BLUE);
      }
    }
}

void colorMorph(int timex)
{
  int keepColorTime = timex * 30;
  for(int BLUE = 0; BLUE <= 15; BLUE++)
  {
    fillTable(BLUE);
    delay(timex);
  }
  delay(keepColorTime);
  for(int BLUE = 15; BLUE >= 0; BLUE--)
  {
    fillTable(BLUE);
    delay(timex);
  }
  delay(keepColorTime);
}

void clearscreen()
{
  memset(matrixBuffer, 0, sizeof(matrixBuffer[0][0]) * BAM_RESOLUTION * 26);
}

void rowscan(byte row)
{
  
  if (row & 0x01) PORTG |= _BV(5);    //PORTA |= _BV(0)
    else          PORTG &= ~_BV(5);   //PORTA &= ~_BV(0)
  
  if (row & 0x02) PORTE |= _BV(3);
    else          PORTE &= ~_BV(3);

  if (row & 0x04) PORTH |= _BV(3);
    else          PORTH &= ~_BV(3);

  if (row & 0x08) PORTH |= _BV(5);
    else          PORTH &= ~_BV(5);

}  
void DIY_SPI(uint8_t DATA)
{
    uint8_t i;
    uint8_t val;
    for (i = 0; i<8; i++)  
    {
      digitalWrite(DATA_Pin, !!(DATA & (1 << (7 - i))));
      digitalWrite(CLOCK_Pin,HIGH);
      digitalWrite(CLOCK_Pin,LOW);                
    }
}


ISR(TIMER1_COMPA_vect)
{   
  PORTE |= ((1<<4));    // Set BLANK PIN high - 74HC595

  if(BAM_Counter==8)    // Bit weight 2^0 of BAM_Bit, lasting time = 8 ticks x interrupt interval time
  BAM_Bit++;
  else
  if(BAM_Counter==24)   // Bit weight 2^1 of BAM_Bit, lasting time = 24 ticks x interrupt interval time
  BAM_Bit++;
  else
  if(BAM_Counter==56)   // Bit weight 2^3 of BAM_Bit, lasting time = 56 ticks x interrupt interval time
  BAM_Bit++;
  BAM_Counter++;
  switch (BAM_Bit)
    {
    case 0:
      //  
        SPI.transfer(matrixBuffer[0][level + 0]);
        SPI.transfer(matrixBuffer[0][level + 1]);
      break;
    case 1:      
      //
        SPI.transfer(matrixBuffer[1][level + 0]);
        SPI.transfer(matrixBuffer[1][level + 1]);
      break;
    case 2:     
      //
        SPI.transfer(matrixBuffer[2][level + 0]);
        SPI.transfer(matrixBuffer[2][level + 1]);
      break;
    case 3:
      //
        SPI.transfer(matrixBuffer[3][level + 0]);
        SPI.transfer(matrixBuffer[3][level + 1]);      
    if(BAM_Counter==120)    //Bit weight 2^3 of BAM_Bit, lasting time = 120 ticks x interrupt interval time
    {
    BAM_Counter=0;
    BAM_Bit=0;
    }
    break;
  }
  
  rowscan(row);
  
  PORTE |= 1<<5;
  delayMicroseconds(2);
  PORTE &= ~(1<<5);
  delayMicroseconds(2); 
  PORTE &= ~(1<<4);       // Set BLANK PIN low - 74HC595
  //delayMicroseconds(2); 
  row++;
  level = row * 2; 
  if (row == 13) row=0;
  if (level == 26) level=0;
  
  DDRE |= _BV (4);      // pinMode (blank_pin, OUTPUT);
}


byte getPixelChar(uint8_t x, uint8_t y, wchar_t ch, uint8_t font)

{
  if (font==FONT3x5)
  {
    if (x > 2) return 0; // 2 = font Width -1
    return bitRead(pgm_read_byte(&font3x5[ch-32][4-y]), 2-x); // 2 = Font witdh -1  
  }
  
  else if (font==FONT5x7)
  {
    if (x > 4) return 0; // 4 = font Width -1
    return bitRead(pgm_read_byte(&font5x7[ch-32][6-y]), 4-x); // 4 = Font witdh -1  
  }
  
  else if (font==FONTVN8x16)
  {
    if (x > 7) return 0; // 7 = font Width -1
    
    if ((ch >=32) && (ch <= 127))
    return bitRead(pgm_read_byte(&font8x16[ch-32][15-y]), 7-x);
    
    if ((ch >=192) && (ch <= 273))
    return bitRead(pgm_read_byte(&font8x16[ch-96][15-y]), 7-x);
    
    if ((ch >=296) && (ch <= 297))
    return bitRead(pgm_read_byte(&font8x16[ch-118][15-y]), 7-x);
    
    if ((ch >=360) && (ch <= 361))
    return bitRead(pgm_read_byte(&font8x16[ch-180][15-y]), 7-x);

    if ((ch >=416) && (ch <= 417))
    return bitRead(pgm_read_byte(&font8x16[ch-234][15-y]), 7-x);
     
    if ((ch >=431) && (ch <= 432))
    return bitRead(pgm_read_byte(&font8x16[ch-247][15-y]), 7-x);
    
    if ((ch >=7840) && (ch <= 7929))
    return bitRead(pgm_read_byte(&vnfont8x16[ch-7840][15-y]), 7-x); // 7 = Font witdh -1  
  }
  else if (font==FONT8x8)
  {
    if (x > 7) return 0; // 7 = font Width -1
    return bitRead(pgm_read_byte(&font8x8[ch-32][7-y]), 7-x); // 7 = Font witdh -1  
  }
  else if (font==FONTEN8x16)
  {
    if (x > 7) return 0; // 7 = font Width -1
    return bitRead(pgm_read_byte(&font8x16[ch-32][15-y]), 7-x); // 7 = Font witdh -1  
  }
  
}
byte getPixelHString(uint16_t x, uint16_t y, wchar_t *p,uint8_t font)

{
  if (font==FONT3x5)
  {
    p=p+x/4;
    return getPixelChar(x%4, y, *p, FONT3x5);
  }
  
  else if (font==FONT5x7)
  {
    p=p+x/6;
    return getPixelChar(x%6, y, *p, FONT5x7);
  }
  
  else if (font==FONTVN8x16)
  {
    p=p+x/9;
    return getPixelChar(x%9, y, *p, FONTVN8x16);
  }
  else if (font==FONT8x8)
  {
    p=p+x/8;
    return getPixelChar(x%8, y, *p, FONT8x8);  
  }

  else if (font==FONTEN8x16)
  {
    p=p+x/9;
    return getPixelChar(x%9, y, *p, FONTEN8x16); 
  }
}

unsigned int lenString(wchar_t *p)
{
  unsigned int retVal=0;
  while(*p!='\0')
  { 
   retVal++;
   p++;
  }
  return retVal;
}


void printChar(uint8_t x, uint8_t y, uint8_t For_color, uint8_t Bk_color, char ch)
{
  uint8_t xx,yy;
  xx=0;
  yy=0;
    
  for (yy=0; yy < 5; yy++)
    {
    for (xx=0; xx < 3; xx++)
      {
      if (bitRead(pgm_read_byte(&font3x5[ch-32][4-yy]),2-xx))
      
        {
            LED(x+xx, y+yy, For_color);
        }
      else
        {
            LED(x+xx, y+yy, Bk_color);      
        }
      }
    }
}

void hScroll(uint8_t y, byte For_color, byte Bk_color, wchar_t *mystring, uint8_t font, uint8_t delaytime, uint8_t times, uint8_t dir)
{
  //int offset =0;
  // FONT 5x7
  int offset;
  byte color;
  if (font == FONT3x5)
  {
  while (times)
    {
    for ((dir) ? offset=0 : offset=((lenString(mystring)-4)*4-1) ; (dir) ? offset <((lenString(mystring)-4)*4-1) : offset >0; (dir) ? offset++ : offset--)
      {
      for (byte xx=0; xx<15; xx++)
        {
        for (byte yy=0; yy<5; yy++)
            {            
              if (getPixelHString(xx+offset, yy, mystring, FONT3x5))
              {
                color = For_color;                
              }
              else 
              {
                color = Bk_color;
              }
                LED(xx, (yy+y), color);
            }
        }
        delay(delaytime);  
      }
    times--;
    }
  }

////////////////////////////////////////////////////////////////
  
  else if (font == FONT5x7)
  {
  while (times)
    {
    for ((dir) ? offset=0 : offset=((lenString(mystring)-5)*6-1) ; (dir) ? offset <((lenString(mystring)-5)*6-1) : offset >0; (dir) ? offset++ : offset--)
      {
      for (byte xx=0; xx<15; xx++)
        {
        for (byte yy=0; yy<7; yy++)
            {            
              if (getPixelHString(xx+offset, yy, mystring, FONT5x7))
              {
                color = For_color;                
              }
              else 
              {
                color = Bk_color;
              }
                LED(xx, (yy+y), color);
            }
        }
        delay(delaytime);  
      }
    times--;
    }
  }
////////////////////////////////////////////      
  else if (font == FONTVN8x16)
  {
  while (times)
    {
    for ((dir) ? offset=0 : offset=((lenString(mystring)-6)*9-1) ; (dir) ? offset <((lenString(mystring)-6)*9-1) : offset >0; (dir) ? offset++ : offset--)
      {
      for (byte xx=0; xx<15; xx++)
        {
        for (byte yy=0; yy<13; yy++)
            {            
              if (getPixelHString(xx+offset, yy, mystring, FONTVN8x16))
              {
                color = For_color;                
              }
              else 
              {
                color = Bk_color;
              }
                LED(xx, (yy+y), color);
            }
        }
        delay(delaytime);  
      }
    times--;
    }
  }
//  
// FONT 8x8
  else if (font == FONT8x8)
    {
    while (times)
      {
      for ((dir) ? offset=0 : offset=((lenString(mystring)-6)*8-1); (dir) ? offset <((lenString(mystring)-6)*8-1): offset >0; (dir) ? offset++ : offset--)
        {
        for (byte xx=0; xx<15; xx++)
          {
            for (byte yy=0; yy<8; yy++)
              {
                if (getPixelHString(xx+offset, yy, mystring, FONT8x8)) 
                  {
                  color = For_color;
                  }
                else 
                {
                  color = Bk_color;
                }
                  LED(xx, (yy+y), color);
              }          
            }
      delay(delaytime);  
        }
      times--;
      }
    }
//
// FONT 8x16  
   else if (font == FONTEN8x16)
    {
    while (times)
    {
    for ((dir) ? offset=0 : offset=((lenString(mystring)-6)*9-1); (dir) ? offset <((lenString(mystring)-6)*9-1) : offset >0; (dir) ? offset++ : offset--)
      {
      for (byte xx=0; xx<15; xx++)
        {     
            for (byte yy=0; yy<13; yy++)
              {
                if (getPixelHString(xx+offset, yy, mystring, FONTEN8x16)) 
                  {
                  color = For_color;
                  }
                else 
                {
                  color = Bk_color;
                }
                  LED(xx, (yy+y), color);
              }   
          }
          delay(delaytime);  
        }
        times--;
      } 
    }
 }
