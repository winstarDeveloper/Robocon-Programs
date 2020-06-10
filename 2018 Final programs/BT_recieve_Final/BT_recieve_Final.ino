// show running rpm and pwm values - test it
// show loading motor state - added - test it
// display mode data on lcd
#include <SoftwareSerial.h>
SoftwareSerial mySerial(10, 9); // RX, TX

#include <LiquidCrystal.h>
LiquidCrystal lcd(12, 11, 5, 4, 3, 2);
unsigned int rpm, Angle, Pan, Velocity, stat, data[11];
boolean gotData = false;
int  start, c;
String rstr;
char command = -1;

void setup() {
  Serial.begin(9600);
  //Serial.begin(9600);//
  lcd.begin(20, 4);
  lcd.clear();
  lcd.setCursor(6, 0);
  lcd.print(" STARTED");
  pinMode(8, OUTPUT);
  analogWrite(8, 0);
//  Serial.println("STARTED");
}

void loop() {
  while (Serial.available() > 0) {
    //Serial.println("GOT DATA");
    if (Serial.read() == ':') {
      rstr = Serial.readStringUntil(';');
      //Serial.println(rstr);
    }
    start = -1;
    c = 0;
    for (int i = 0; i <= rstr.length(); i++) {
      if (rstr.charAt(i) == ',') {
        data[c] = rstr.substring(start + 1, i).toInt();
        //Serial.print(" ");
        //Serial.print(rstr.substring(start + 1, i));
        c = c + 1;
        start = i;
      }
    }
    //    Serial.print("c : ");
    //    Serial.println(c);
    //serialPrint();
    printLcd();
  }
}

//str = str + autoAdjust + ',' + command + ',' + autoAdjustCount + ',' + autoValues[autoAdjustCount][0] + ',' + autoValues[autoAdjustCount][1]
//+ ',' + autoValues[autoAdjustCount][2] +  ',' + throwingMotor + ',' + angleVal + ',' + panCount + ',' + ';';

//str = str + autoAdjust + ',' + command + ',' + autoAdjustCount + ',' + (int)desired_rpm + ',' + throwingMotor + ',' + angleVal
//+ ',' + panCount + ',' + loadingMotorState + ',' + ';';


void serialPrint() {
  if (c == 11 && data[0] == 1) {
    Serial.print("Auto Adjust : "); Serial.print(data[0]);
    Serial.print(" Command : "); Serial.print((char)data[1]);
    Serial.print(" Set SPOT : "); Serial.print(data[2]);
    Serial.print(" Set RPM : "); Serial.print(data[3]);
    Serial.print(" Set Angle : "); Serial.print(data[4]);
    Serial.print(" Set PAN : "); Serial.print(data[5]);
    Serial.print(" Throwing Motor : "); Serial.print(data[6]);
    Serial.print(" Angle Value : "); Serial.print(data[7]);
    Serial.print(" Pan Count : "); Serial.print(data[8]);
    Serial.print(" Loading Motor State : "); Serial.print(data[9]);
    Serial.print(" Line : "); Serial.println(data[10]);
  } else if (c == 9 && data[0] == 0) {
    Serial.print(" Auto Adjust : "); Serial.print(data[0]);
    Serial.print(" Command : "); Serial.print((char)data[1]);
    Serial.print(" Set SPOT : "); Serial.print(data[2]);
    Serial.print(" Set RPM : "); Serial.print(data[3]);
    Serial.print(" Angle : "); Serial.print(data[5]);
    Serial.print(" PAN : "); Serial.print(data[6]);
    Serial.print(" Throwing Motor : "); Serial.print(data[4]);
    Serial.print(" Loading Motor State : "); Serial.println(data[7]);
    Serial.print(" Line : "); Serial.println(data[8]);
  }
}

// Forward : F
// Reverse : B
// Left : L
// Right : R
// Stop : S
// Hard Stop : H
// LineFollow Forward : A
// LineFollow Backward : Z
// Position Left : O
// Position Right : P
// Automatic mode : Q
// Auto mode : D

void printLcd() {
  if (c == 11 && data[0] == 1) {
    lcd.clear();
    lcd.setCursor(0, 0);
    command = (char)data[1];
    if ((char)data[1] == 'F') {
      lcd.print("FORWARD");
    } else if ((char)data[1] == 'B') {
      lcd.print("REVERSE");
    } else if ((char)data[1] == 'L') {
      lcd.print("LEFT");
    } else if ((char)data[1] == 'R') {
      lcd.print("RIGHT");
    } else if ((char)data[1] == 'S') {
      lcd.print("STOP");
    } else if ((char)data[1] == 'H') {
      lcd.print("HARD_STOP");
    } else if ((char)data[1] == 'A') {
      lcd.print("LINE_FORWARD");
    } else if ((char)data[1] == 'Z') {
      lcd.print("LINE_REVERSE");
    } else if ((char)data[1] == 'O') {
      lcd.print("LEFT_SPOT");
    } else if ((char)data[1] == 'P') {
      lcd.print("RIGHT_SPOT");
    } else if ((char)data[1] == 'Q') {
      lcd.print("AUTO_CENTER");
    } else if ((char)data[1] == 'G') {
      lcd.print("A_L_SLOW");
    } else if ((char)data[1] == 'D') {
      lcd.print("AUTO_LOADING");
    } else if ((char)data[1] == 'V') {
      lcd.print("RIGHT_COUNT");
    } else if ((char)data[1] == 'N') {
      lcd.print("LEFT_COUNT");
    }

    lcd.setCursor(14, 1);
    if (data[6] == 0) {
      lcd.print("MOT_OFF");
    } else if (data[6] == 1) {
      lcd.print("MOT_ON");
    }

    lcd.setCursor(14, 2);
    if (data[9] == 0) {
      lcd.print("  OFF");
    } else if (data[9] == 1) {
      lcd.print("  PUSH");
    } else if (data[9] == 2) {
      lcd.print("GT_END");
    } else if (data[9] == 3) {
      lcd.print("  PULL");
    } else if (data[9] == 4) {
      lcd.print("S_DISC");
    } else if (data[9] == 5) {
      lcd.print(" DONE");
    }

    lcd.setCursor(13, 0);
    lcd.print("SPOT:");
    lcd.print(data[2]);

    lcd.setCursor(0, 1);
    lcd.print("RPM:");
    lcd.print(data[3]);

    lcd.setCursor(0, 2);
    lcd.print("ANGLE:");
    lcd.print(data[7]);
    lcd.print("=");
    lcd.print(data[4]);

    lcd.setCursor(0, 3);
    lcd.print("PAN:");
    lcd.print(data[8]);
    lcd.print("=");
    lcd.print(data[5]);

    lcd.setCursor(12, 3);
    if (data[10] == 0) {
      lcd.print("LINE_OFF");
    }else if(data[10] == 1){
      lcd.print("LINE_ON");
    }
  } else if (c == 9 && data[0] == 0) {
    lcd.clear();
    lcd.setCursor(0, 0);
    command = data[1];
    if ((char)data[1] == 'F') {
      lcd.print("FORWARD");
    } else if ((char)data[1] == 'B') {
      lcd.print("REVERSE");
    } else if ((char)data[1] == 'L') {
      lcd.print("LEFT");
    } else if ((char)data[1] == 'R') {
      lcd.print("RIGHT");
    } else if ((char)data[1] == 'S') {
      lcd.print("STOP");
    } else if ((char)data[1] == 'H') {
      lcd.print("HARD STOP");
    } else if ((char)data[1] == 'A') {
      lcd.print("LINE FORWARD");
    } else if ((char)data[1] == 'Z') {
      lcd.print("LINE REVERSE");
    } else if ((char)data[1] == 'O') {
      lcd.print("LEFT SPOT");
    } else if ((char)data[1] == 'P') {
      lcd.print("RIGHT SPOT");
    } else if ((char)data[1] == 'Q') {
      lcd.print("AUTO_CENTER");
    } else if ((char)data[1] == 'G') {
      lcd.print("A_L_SLOW");
    } else if ((char)data[1] == 'D') {
      lcd.print("AUTO_LOADING");
    } else if ((char)data[1] == 'V') {
      lcd.print("RIGHT_COUNT");
    } else if ((char)data[1] == 'N') {
      lcd.print("LEFT_COUNT");
    }

    lcd.setCursor(13, 0);
    lcd.print("SPOT:");
    lcd.print(data[2]);

    lcd.setCursor(0, 1);
    lcd.print("RPM:");
    lcd.print(data[3]);

    lcd.setCursor(14, 1);
    if (data[4] == 0) {
      lcd.print("MOT_OFF");
    } else if (data[4] == 1) {
      lcd.print("MOT_ON");
    }

    lcd.setCursor(14, 2);
    if (data[7] == 0) {
      lcd.print("  OFF");
    } else if (data[7] == 1) {
      lcd.print("  PUSH");
    } else if (data[7] == 2) {
      lcd.print("GT_END");
    } else if (data[7] == 3) {
      lcd.print("  PULL");
    } else if (data[7] == 4) {
      lcd.print("S_DISC");
    } else if (data[7] == 5) {
      lcd.print(" DONE");
    }

    lcd.setCursor(0, 2);
    lcd.print("ANGLE:");
    lcd.print(data[5]);

    lcd.setCursor(0, 3);
    lcd.print("PAN:");
    lcd.print(data[6]);
  
    lcd.setCursor(12, 3);
    if (data[8] == 0) {
      lcd.print("LINE_OFF");
    }else if(data[8] == 1){
      lcd.print("LINE_ON");
    }
  }
}
