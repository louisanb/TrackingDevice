
//define serCell to enable Celluler feature
//#define serCell
#define DEBUG
#define GPS1
String PackageID = "12345";

#include <SparkFunHTU21D.h>
HTU21D myHumidity;

#include <Wire.h>
// Serial Port initializations
#include <SoftwareSerial.h>
SoftwareSerial serCell(10,11);  // RX (10), TX (11)
//SoftwareSerial HT(21 20);
long baud_ArduIMU = 38400;
long baud_Cell = 9600;
long baud_Debug = 9600;
long baud_GPS = 57600;
long baud_SD = 9600;
long baud_SD2=9600;

// General initializations
String CELL_Reply = "8452404605";
int CellService = 0;    // =1 when Cell Service ok, =0 when Cell Service LOST
int LED = 13;
char charCR = 13;
char charLF = 10;
char charLT = '<';
char charCTRL_Z = 26;
char SD_WakeUp = '?';
long lLatitude, lLongitude;
String CEL_String, GPS_String, IMU_String;
String sAltitude, sLatitude, sLongitude, sTime, sDate;
// For MM:dd.sss -> decimal numeric conversions of Latitude and Longitude
char caTemp[20];
long lTemp;
String sTemp;

int count=0;//QA count and runtime can be easy changed to local variable
int runtime=0;
int cgd=0;//count goodData loop run time
String Mc="1";
String easyRead="@";
String simpleRead="@";

const int LGreen=42;//light green Tilt
const int Red =  44;//Temprature 
const int Green = 46;//Humidity
const int Yellow = 48;//acceleration 
const int BGreen = 41;//bright green Button
const int buttonPin = 38;     
int ledState;//for button light and cant be local variable 
String TEMP="@";
String HUMD="@";
int countT=0;
int countH=0;
unsigned long previousMillis = 0; 
const long interval = 1000;
int buttonState = 0;         
int buttonPress = 0;
boolean cur;
boolean pre;
int pres=0;
boolean flash;
boolean goodData;//cant be local variable
// Cellular State Machine
String CellMode = "thru";
String Reply_Phone = "\"8452404605\"";

void setup() {
  pinMode(LED, OUTPUT);    // Initialize the LED that shows us complete initialization
  digitalWrite(LED,HIGH);  // One on-off blink to show we are running
  delay(1000);
  digitalWrite(LED,LOW);
  
  pinMode(LGreen, OUTPUT);
  pinMode(Red, OUTPUT);
  pinMode(Yellow, OUTPUT);
  pinMode(Green, OUTPUT);
  pinMode(BGreen, OUTPUT);
  pinMode(buttonPin, INPUT);   
  // initialize both serial ports:
  delay(5000);      // TODO Remove for production -- Wait 5 seconds to allow debugging serial port connection
  #if defined DEBUG
    Serial.begin(baud_Debug);     // Debug port 9600
  #endif
  #if defined GPS1
  Serial1.begin(baud_GPS);      // GPS 57600
  #endif
  Serial2.begin(baud_SD);       // SD Card 9600
  Serial3.begin(baud_ArduIMU);  // ArduIMU 38400
  Serial2.begin(baud_Cell);     // Cellular SMS 9600
  Init_GPS();    // Set Refresh Rate on GPS, MAY need to update this to update baud rate also
  Init_SD();    // Start a new tracking file
  myHumidity.begin();
}
void loop() {
  // /////////////////////////////////////////SER0: Debug Port Processing///////////////////////
  #if defined DEBUG
    if (Serial.available()) {
      int inByte = Serial.read();
      if (inByte == '?') {
        Serial.println(sLongitude + "," + sLatitude + "," + sAltitude);
      } else { 
        if (inByte == '.' /*||goodData==false &&pres==2*/) {
          Serial2.println("        </coordinates>");
          Serial2.println("      </LineString>");
          Serial2.println("    </Placemark>");
          Serial2.println("  </Document>");
          Serial2.println("</kml>");
          Serial2.flush(); Serial2.end();
          Serial2.begin(baud_SD);
          Serial2.println("SD2");//easy to read version of the data after properly closed  QA
          String Quickread;
          Serial.print("Sd2");
         while(count-runtime>0){
            runtime+=1;
            Quickread=QuickRead(simpleRead);//trim data to have a single output instead of output all the data which is easy to read
            Serial2.println(Quickread);//send data to SD
            Serial.println(Quickread);
            simpleRead=SimpleRead(simpleRead);//triming data process(trim off displayed data) in order to have an easy read data output for function QuickRead 
          }
            String QuickreadT;//this is "r" not "R" as in the function. QuickReadT
            String QuickreadH;
            int runtimeT=0;
            int runtimeH=0;
          while(countH-runtimeH>0){
            runtimeH+=1;
            QuickreadH=QuickReadH(HUMD);
            Serial2.println(QuickreadH);//send data to SD
            Serial.println(QuickreadH);
            HUMD=SimpleReadH(HUMD);//triming data process(trim off displayed data)
            }//While
          while(countT-runtimeT>0){
            runtimeT+=1;
            QuickreadT=QuickReadT(TEMP);
            Serial2.println(QuickreadT);//send data to SD
            TEMP=SimpleReadT(TEMP);//triming data process(trim off displayed data)
            }//While*//*/
/*the bigbraket before the else below is for "if (inByte == '.')"*/  // QA        
        }else{
          if (inByte=='p'){//end communication with serial monitor
            Serial.end();
      }else {
          if (inByte == 'c') {
            CellChk();
          } else {
            if (inByte == 'i') {
              Init_Cell();
            } else {
              if (inByte == 's') {
                CellSig();
              } else {
            serCell.write(inByte);
              }
            }
          }
      }
        }//for else with if (inByte == 'c') QA
      }//for first else
    }//for first if
  #endif
  
#if defined GPS1
  // SER1: GPS Processing
  if (Serial1.available()) {
    int inByte = Serial1.read();
    if (inByte == 13) {
      ProcessGPS(GPS_String);
      GPS_String="";
    } 
    else {
      if (inByte != 10) {
        GPS_String = GPS_String + char(inByte);
      }
    }
  }
#endif
  // SER2: SD Card
  // SER3: ArduIMU Processing
 if (Serial3.available()) {
    int inByte = Serial3.read();
    if (inByte == 13 && goodData==false) {
      IMU_String="";//erasing no goodData QA IMU process need to be outside of button and goodData section to be able to work with goodData feature
    }else{
    if (inByte == 13 && goodData==true) {
      Process_IMU(IMU_String);
      IMU_String="";
      digitalWrite(LGreen, HIGH);
    } else {
      if (inByte != 10) {
        IMU_String = IMU_String + char(inByte);
      }
     }
    }
   }

  unsigned long currentMillis = millis();//QA
  //if (digitalRead.available()){}
  //if(currentMillis - previousMillis >= interval) {//  I1
  // previousMillis = currentMillis;
    cur = digitalRead(buttonPin);
  if(pre==LOW && cur==HIGH){
   // Serial.print(pres);
    pres++;
    cgd++;
   // digitalWrite(Yellow,HIGH);//goodData debug
     goodData=true;//no error occurs when goodData==true so be careful with the = signs here 
     flash=true;
    // Serial3.print("gD");//if send GD will interfere GPS output 
  }
  if(pre==LOW && cur==HIGH && pres==2){//pres=2 indicate its secound press which is to stop taking data
     goodData=false; 
     flash=false;
   pres=0;  
   Serial2.println("        </coordinates>");
          Serial2.println("      </LineString>");
          Serial2.println("    </Placemark>");
          Serial2.println("  </Document>");
          Serial2.println("</kml>");
          Serial2.flush(); Serial2.end();
          Serial2.begin(baud_SD);
          Serial2.println("SD2");//easy to read version of the data after properly closed  QA
          String Quickread;
          Serial.println("Sd2");
         while(count-runtime>0){
            runtime+=1;
            Quickread=QuickRead(simpleRead);//trim data to have a single output instead of output all the data which is easy to read
            Serial2.println(Quickread);//send data to SD
            Serial.println(Quickread);
            simpleRead=SimpleRead(simpleRead);//triming data process(trim off displayed data) in order to have an easy read data output for function QuickRead 
          }
            String QuickreadT;//this is "r" not "R" as in the function. QuickReadT
            String QuickreadH;
            int runtimeT=0;
            int runtimeH=0;
          while(countH-runtimeH>0){
            runtimeH+=1;
            QuickreadH=QuickReadH(HUMD);
            Serial2.println(QuickreadH);//send data to SD
            Serial.println(QuickreadH);
            HUMD=SimpleReadH(HUMD);//triming data process(trim off displayed data)
            }//While
          while(countT-runtimeT>0){
            runtimeT+=1;
            QuickreadT=QuickReadT(TEMP);
            Serial2.println(QuickreadT);//send data to SD
            TEMP=SimpleReadT(TEMP);//triming data process(trim off displayed data)
            }
  }
  if (flash==true){
  if(currentMillis - previousMillis >= interval) {
   previousMillis = currentMillis;//fixing press button to stop while flashing light was on and the flashing light stay on bug   
    if (ledState == LOW)
      ledState = HIGH;
    else
      ledState = LOW;
    digitalWrite(BGreen, ledState);
         }
       }
       else {digitalWrite(BGreen, LOW);}
       pre=cur;
  //}//for interval delay (I1)of fixing goodData/button/avalibale issue
      // delay(500);//for goodData debug
  // Serial.print(goodData);//*/  QA
 //////////////////////////////////////////////////goodData section///////////////////////////////////   
if (goodData==true){
 ////////////////////////////////////Humidity and Temperature processing /////////////////////////////////////////////
  //unsigned long currentMillis = millis();//QA
 // if(currentMillis - previousMillis >= interval) {//I2 this will in terfere Humidity feature when no button section, these two lines has to be enable so the humidity section wont in terfere with GPS
   //previousMillis = currentMillis;//alternate way to have available other wise humidity code interfere GPS data collection   
 //if (myHumidity.available()){ //error class HTU21D has no member names 'available'-solution: add available feature in the library source file
   float humd = myHumidity.readHumidity();
           float temp = myHumidity.readTemperature();
           String HUmd;
           String TEmp;
             // Serial.print("Time:");
             // Serial.print(millis());
            if (temp >28.7||temp<5){//40<temp<5 for code in real life
              digitalWrite(Red, HIGH);
              countT+=1;//once get GPS connection time should work TEMP+sDate + " at " + sTime + " " +Tempe
              TEmp=String(temp, 1);
              TEMP=TEMP+sDate + " at " + sTime + " " + " Temperature:"+TEmp+"C"+" @";
           // TEMP=TEMP+" Temperature:"+TEmp+"C"+" @";
           //   Serial.println(TEMP);
           }//else{
              if (humd >53){//>80 for code in real life 53 is just a demostration
                digitalWrite(Green, HIGH);
                countH+=1;
                HUmd=String(humd,1);
                //Serial.print(HUmd);
                HUMD=HUMD+sDate + " at " + sTime + " " +" Humidity:"+HUmd+"%"+" @";
                 Serial.println(HUMD);
            }//*/
           // } 
         //   delay(1000);
    //}
 // }//for the if with time interval(I2) check  QA
  
  // SerCel: Cellular Processing
  //Process_cell();//available impact testing
  #if defined serCell
  if (serCell.available()) {
    int inByte = serCell.read();
    if (inByte == 13) {
      Process_Cell(CEL_String);
      CEL_String="";
    } else {
      if (inByte != 10) {
        CEL_String = CEL_String + char(inByte);
      }
    }//*/
   }
   #endif
 }//while/if goodData
}//void loop

//
// INITIALIZATION SUBROUTINES
//

void Init_GPS() {
  Serial1.println("$PMTK220,5000*1B");      // 5 Second Intervals
  // Wait for $PMTK001,220,3*cc  back from GPS to indicate success
  Serial1.println("$PMTK314,0,1,1,2,0,0,0,0,0,0,0,0*36");
  // Wait for $PMTK001,314,3*cc  back from GPS to indicate success
}

void Init_SD() {
  //  // Wait for 12< for 3 seconds
  //  char s2input[10];
  //  int intCharsIn = 0;
  //  
  //  Serial2.setTimeout(3000);
  //  intCharsIn = Serial2.readBytesUntil(charCTRL_Z, s2input, 5);
  //  Serial.println("Recieved: " + String(intCharsIn));
  //
  //  // Attempt to go into command prompt, remove any existing logs, and reset SD Interface
  //  Serial2.print(SD_WakeUp);
  //  Serial2.print(SD_WakeUp);
  //  Serial2.print(SD_WakeUp);
  //  
  //  intCharsIn = Serial2.readBytesUntil('>', s2input, 10);
  //  Serial.println("CmdPrompt: " + String(intCharsIn));
  //  delay(1000);
  //  Serial2.print("rm Log*.txt\n");
  //  intCharsIn = Serial2.readBytesUntil('>', s2input, 10);
  //  Serial.println("Post rm: " + String(intCharsIn));
  //  delay(1000);
  //  Serial2.print("reset\n");
  //  intCharsIn = Serial2.readBytesUntil('<', s2input, 10);
  //  Serial.println("Post Reset: " + String(intCharsIn));
  //  Serial.println("Begin passthru from Serial -> Serial2 -> SD_Card");
  Serial2.println("<?xml version=\"1.0\" encoding=\"UTF-8\"?>");
  Serial2.println("<kml xmlns=\"http://www.opengis.net/kml/2.2\">");
  Serial2.println("  <Document>");
  Serial2.println("    <name>Package Tracking</name>");
  Serial2.println("    <description>Package ID: BlueBox_" + PackageID + "</description>");
  Serial2.println("    <Style id=\"yellowLineGreenPoly\">");
  Serial2.println("      <LineStyle>");
  Serial2.println("        <color>7f00ffff</color>");
  Serial2.println("        <width>4</width>");
  Serial2.println("      </LineStyle>");
  Serial2.println("      <PolyStyle>");
  Serial2.println("        <color>7f00ff00</color>");
  Serial2.println("      </PolyStyle>");
  Serial2.println("    </Style>");
  Serial2.println("    <Style id=\"redLineGreenPoly\">");
  Serial2.println("      <LineStyle>");
  Serial2.println("        <color>7f0000ff</color>");
  Serial2.println("        <width>4</width>");
  Serial2.println("      </LineStyle>");
  Serial2.println("      <PolyStyle>");
  Serial2.println("        <color>7f00ff00</color>");
  Serial2.println("      </PolyStyle>");
  Serial2.println("    </Style>");
  Serial2.println("    <Placemark>");
  Serial2.println("      <name>Package Tracking</name>");
  Serial2.println("      <description></description>");
  Serial2.println("      <styleUrl>#yellowLineGreenPoly</styleUrl>");
  Serial2.println("      <LineString>");
  Serial2.println("        <extrude>1</extrude>");
  Serial2.println("        <tessellate>1</tessellate>");
//  Serial2.println("        <altitudeMode>clampToGround</altitudeMode>");        // 2d tracking
  Serial2.println("        <altitudeMode>absolute</altitudeMode>");            // 3d tracking
  Serial2.println("        <coordinates>");
}


void Init_Cell() {
  // This may need to wait for cell service before sending this..
  serCell.println("AT+CMGF=1");          // Set SMS to text mode
  delay(500);
  serCell.println("AT+CNMI=3,3,0,0");    // Set cellular to pass all text messages direct to Arduino
  delay(500);                            // Required for 2-way messaging functionality
  serCell.println("AT+CMGD=1,4");        // Erase all SMS messages
  delay(2000);
  Serial.println("Init_Cell() called");
}


//
// PROCESSING SUBROUTINES
//

void ProcessGPS(String message) {
  String sLat, sLong;
  int MT;//modified time QA
   String Mt;
   int mT;
   
  if (message.startsWith("$GPRMC")) {
    //    Serial.println(CSV_Next(message));
    //    Serial.println();
    message = CSV_Remove(message);
    sTemp = CSV_Next(message);
 
    MT = sTemp.substring(0, 2).toInt();//converting string to int  
    mT=MT-4;//time zone correction
    Mt=String(mT);//converting corrected time from int to string
    //Serial.println("UTC Time/Before: '" + sTemp + "'");
    sTime = Mt + ":" + sTemp.substring(2,4) + ":" + sTemp.substring(4,6);
    //Serial.println("UTC Time: " + sTime);
    message = CSV_Remove(message);
    //    Serial.print("A/V: ");
    if (CSV_Next(message) == "A") {
      //      Serial.println("Active");
      message = CSV_Remove(message);
      // Latitude
      sLat = CSV_Next(message);
      //      Serial.println("Latitude: " + CSV_Next(message));
      // North/South
      message = CSV_Remove(message);
      String NS = CSV_Next(message);
      //      Serial.println("North/South(-): " + CSV_Next(message));
      message = CSV_Remove(message);
      // Longitude
      sLong = CSV_Next(message);
      //      Serial.println("Longitude: " + CSV_Next(message));
      message = CSV_Remove(message);
      // East/West
      String EW = CSV_Next(message);
      //      Serial.println("East/West(-): " + CSV_Next(message));
      // Speed over Ground in Knots
      message = CSV_Remove(message);
      //      Serial.println("Speed over ground in Knots: " + CSV_Next(message));
      // Course over Ground
      message = CSV_Remove(message);
      //      Serial.println("Course over ground: " + CSV_Next(message));
      message = CSV_Remove(message);
      // UTC Date
      sTemp = CSV_Next(message);
//      Serial.println("UTC Date/Before: '" + sTemp + "'");
      sDate = sTemp.substring(2,4) + "/" + sTemp.substring(0,2) + "/" + sTemp.substring(4);
//      Serial.println("UTC Date: " + sDate);
      message = CSV_Remove(message);
    //  easyRead=easyRead+"   "+sDate+sTime;

      // Format and store for use if a Tilt event occurs
      // Write GPS Data to OpenLog
      Serial2.print("        ");
      
      sLongitude="";
      if (EW == "W") { 
        sLongitude="-";
      }
      sLongitude = sLongitude + sLong.substring(0,3);
      sLongitude = sLongitude + ".";

      (sLong.substring(3,5) + sLong.substring(6)).toCharArray(caTemp,20);
      lTemp = atol(caTemp);
      lTemp = lTemp / 6;
      sTemp = String(lTemp);
      sLongitude += lTemp;
      Serial2.print(sLongitude);    

//      Serial2.print(sLong.substring(3,5)); 
//      sLongitude = sLongitude + sLong.substring(3,5);
//      Serial2.print(sLong.substring(6));
//      sLongitude = sLongitude + sLong.substring(6);

      Serial2.print(",");
      sLatitude="";
      if (NS == "S") { 
        sLatitude = "-";
      }
      sLatitude = sLatitude + sLat.substring(0,2);
      sLatitude = sLatitude + ".";
      (sLat.substring(2,4) + sLat.substring(5)).toCharArray(caTemp,20);
      lTemp = atol(caTemp);
      lTemp /= 6;
      sTemp = String(lTemp);
      sLatitude += lTemp;
      Serial2.print(sLatitude);
      
//      Serial2.print(sLat.substring(0,2)); 
//      sLatitude = sLatitude + sLat.substring(0,2);
//      Serial2.print("."); 
//      sLatitude = sLatitude + ".";
//      Serial2.print(sLat.substring(2,4)); 
//      sLatitude = sLatitude + sLat.substring(2,4);
//      Serial2.print(sLat.substring(5));
//      sLatitude = sLatitude + sLat.substring(5);
      Serial2.println("," + sAltitude);
    }
    #if defined DEBUG
      else {
        Serial.println("INVALID GPS"); 
      }
    #endif
    //    Serial.println("Remains of message: '" + message + "'");
  }
  if (message.startsWith("$GPVTG")) {
//    Serial.println(message);
  }
    if (message.startsWith("$GPGGA")) {
      // Process altitude information
      // $GPGGA,------.--,----.--,a,-----.--,-,-,--,-.-,#.#,M,-.-,-,-.-,----*hh
      // Serial.println(message);
      message = CSV_Remove(message);
      message = CSV_Remove(message);
      message = CSV_Remove(message);
      message = CSV_Remove(message);
      message = CSV_Remove(message);
      message = CSV_Remove(message);
      message = CSV_Remove(message);
      message = CSV_Remove(message);
      message = CSV_Remove(message);
      sAltitude = CSV_Next(message); // message = CSV_Remove(message);
      // sAltitude = sAltitude + CSV_Next(message);
      // Serial.println("Altitude = '" + sAltitude + "'");
    }
  #if defined DEBUG
    if (message.startsWith("$PMTK")) {
      Serial.println(message);
    }
  #endif
}

void Process_IMU(String message) {//IMU_string is the input out put is message
  String event;
  
  if (message.startsWith("TP") | message.startsWith("TR")) {
    //if (goodData==true){//goodData debug
    #if defined DEBUG
    Serial.print(" ");
    Serial.print(sDate);//GPS debug use
    Serial.print(" ");
    Serial.print(sTime);
    Serial.print(" ");
    Serial.println(message);//printint our processed IMU_string
    #endif
    count+=1;//QA
    Mc=String(count);//converting corrected time from int to string
    if (message.startsWith("TP")) { event = "Pitch"; } else { event = "Roll"; }
    /* if (message.startsWith("TP")) {//wont work with goodData
    event = "Pitch"; digitalWrite(LGreen, HIGH); } 
   else { event = "Roll"; digitalWrite(LGreen, HIGH); }
    if (message.startsWith("AN")) {digitalWrite(Yellow, HIGH);}//QA//*/  
    
    // Add a placemark with Pitch/Roll angle
    SetPlacemark(sDate + " at " + sTime + " " + event + " " + message.substring(2) + " degrees");

   //easyRead=easyRead+sDate + " at " + sTime + " " + event + " " + message.substring(2) + " degrees"+" @";//alternative for extra saved data outside of SD
   simpleRead=simpleRead+" "+sDate + " at " + sTime + " " + event + " " + message.substring(2) + " degrees"+" @";//QA

    SendSMS(sDate + " at " + sTime + " UTC SystemID=" + PackageID + "  " + event + " " + message.substring(2) + " degrees, GPS:" + sLatitude + "," + sLongitude + "," + sAltitude);
  // }
  }
}
void Process_Cell(String message) {
  Serial.println(message);
  if (CellMode == "att") {
    CellMode = "null";
    return;
  }
  message.toLowerCase();
  if (CellMode == "cmd") {
    CellMode = "null";
    if (message.startsWith("stat")) {
      // Return IMU data as well as GPS
    }
    if (message.startsWith("?")) {
      // Return just GPS
      #if defined DEBUG
        Serial.println("Reply to >" + Reply_Phone + "<- GPS: " + sLongitude + "," + sLatitude + "," + sAltitude);
      #endif
      SendSMS("GPS: " + sLongitude + "," + sLatitude + "," + sAltitude, Reply_Phone);
    }
  } else {
    // +cmt: "+18453376561","+12085978925","14/05/08,11:12:36+00",1
    // Cell-service=0 cmdmode="null": f+cmt: "+18456569539","+12085978925","14/05/08,11:45:52+00",1
    if (message.startsWith("+cmt: ")) {
      Serial.println(message);
//      CSV_Next(message); CSV_Remove(message);    // Parse off Cmd and index
//      CSV_Next(message); CSV_Remove(message);    // Parse off Msg Status
      Reply_Phone = CSV_Next(message);
      // Trim off command, quotes, etc
      Reply_Phone = Reply_Phone.substring(1,Reply_Phone.length());
      int leftquote = Reply_Phone.indexOf("+");
      Reply_Phone = (Reply_Phone.substring(leftquote+2,Reply_Phone.length()-1));
      if (Reply_Phone.indexOf("288") == -1) {
        // Store Reply Phone
        #if defined DEBUG
          Serial.println("Reply Phone Stored: >" + Reply_Phone + "<");
        #endif
        CellMode = "cmd";        // Set Cellular processing to command mode
      } else {
         Serial.println("Ignoring AT&T message");
         CellMode = "att";      // Set Cellular mode to skip next line of text
      }
    } else {
      #if defined DEBUG
//        Serial.println("Cell-service=" + String(CellService) + " cmdmode=\"" + CellMode + "\":" + message);
        Serial.println("cmdmode=\"" + CellMode + "\":" + message);
      #endif
      if (message.startsWith("+sind: 4")) {
        // Device is ready for AT commands
        CellService = 1;
        Init_Cell();
        CellChk();
      }
      CellMode = "null";
    }
  }
}


//
// Global Subroutines
//

void SetPlacemark(String message) {
    Serial2.println("        </coordinates>");
    Serial2.println("      </LineString>");
    Serial2.println("    </Placemark>");
    // Add Placemarker
    Serial2.println("  <Placemark>");
//    Serial2.println("    <name>" + sDate + " at " + sTime + " " + event + " event</name>");
    Serial2.println("    <description>" + message + "</description>");
    Serial2.println("    <Point>");
    Serial2.println("      <coordinates>" + sLongitude + "," + sLatitude + "," + sAltitude + "</coordinates>");
    Serial2.println("    </Point>");
    Serial2.println("  </Placemark>");
    // Restart GPS path
    Serial2.println("    <Placemark>");
    Serial2.println("      <name>Package Tracking</name>");
    Serial2.println("      <description></description>");
    if (CellService == 1) {
      Serial2.println("      <styleUrl>#yellowLineGreenPoly</styleUrl>");
    } else {
      Serial2.println("      <styleUrl>#redLineGreenPoly</styleUrl>");
    }
    Serial2.println("      <LineString>");
    Serial2.println("        <extrude>1</extrude>");
    Serial2.println("        <tessellate>1</tessellate>");
    Serial2.println("        <altitudeMode>clampToGround</altitudeMode>");
    Serial2.println("        <coordinates>");
    Serial2.println("        " + sLongitude + "," + sLatitude + "," + sAltitude);

   //easyRead=easyRead+" Tilt number "+ Mc + " GPS: " + sLongitude + "," + sLatitude + "," + sAltitude;//extra data save  QA
   simpleRead=simpleRead+" Tilt number "+ Mc + " GPS: " + sLongitude + "," + sLatitude + "," + sAltitude;
}

String CSV_Next(String message) {
  int commaPosition = message.indexOf(',');
                                            //Serial.println("CSV_Next");
                                            //Serial.println(message);
  if(commaPosition != -1)
  {
    return (message.substring(0,commaPosition));
    //          message = message.substring(commaPosition+1, message.length());
  }
  else
  {  // here after the last comma is found
    if(message.length() > 0)
      return(message);  // if there is text after the last comma, return it
  }
}

String CSV_Remove(String message) {
  int commaPosition = message.indexOf(',');
  return message.substring(commaPosition+1, message.length());
}

String SimpleRead(String simpleRead){  //triming process(trim off displayed data) for easy read diaply QA
  int atPosition = simpleRead.indexOf('@');
  int atPosition2=simpleRead.substring(atPosition+1, simpleRead.length()).indexOf('@')+1;
  return simpleRead.substring(atPosition2, simpleRead.length());
}

String QuickRead(String simplreRead){  //making single data output which is easy to read
  int qPosition = simpleRead.indexOf('@');
  int qPosition2=simpleRead.substring(qPosition+1, simpleRead.length()).indexOf('@')+1;
  return simpleRead.substring(qPosition+1, qPosition2-1);
}

String SimpleReadH(String simpleRead_HUMD){  //triming process(trim off displayed data) for easy read diaply 
  int atPosition = HUMD.indexOf('@');
  int atPosition2=HUMD.substring(atPosition+1, HUMD.length()).indexOf('@')+1;
  return HUMD.substring(atPosition2, HUMD.length());
}//*/

String QuickReadH(String HUMD){  //making single data output which is easy to read
  int qPosition = HUMD.indexOf('@');
  int qPosition2=HUMD.substring(qPosition+1, HUMD.length()).indexOf('@')+1;
  return HUMD.substring(qPosition+1, qPosition2-1);
}  

String SimpleReadT(String TEMP){  //triming process(trim off displayed data) for easy read diaply 
  int atPosition = TEMP.indexOf('@');
  int atPosition2=TEMP.substring(atPosition+1, TEMP.length()).indexOf('@')+1;
  return TEMP.substring(atPosition2, TEMP.length());
}//*/

String QuickReadT(String TEMP){  //making single data output which is easy to read
  int qPosition = TEMP.indexOf('@');
  int qPosition2=TEMP.substring(qPosition+1, TEMP.length()).indexOf('@')+1;
  return TEMP.substring(qPosition+1, qPosition2-1);
}//QA
  
void CellChk() {
  // Query cell for signal strength, etc
  Serial.println("Querying cell");
//  serCell.println("AT+CMGF=1");
//  delay(500);
  serCell.println("AT+CCLK?");
  // Wait for '+CCLK: "14/01/23,03:12:34' response line
  Serial.println("AT+CCLK response: " + CellWaitForLine());
  WaitForOK();
  serCell.println("AT+CSQ");
  Serial.println("AT+CSQ response: " + CellWaitForLine());
  // Wait for 'OK'
  WaitForOK();
//  serCell.println("AT+CNMI=3,3,0,0");    // Set cellular to pass all text messages direct to Arduino
//  // Wait for 'OK'
//  delay(500);
//  serCell.println("AT+CMGD=1,4");        // Erase all SMS messages
}

void CellSig() {
  serCell.println("AT+CSQ");
}

void SendSMS(String message) {
  SendSMS(message, "8452404605");
}

void SendSMS(String message, String PhoneNum) {
  String strResponse;
  serCell.println("AT+CMGF=1");
  delay(1000);
//  strResponse = WaitForOK();
//  if (strResponse.startsWith("OK") == false) {
//    Serial.println("No OK received to text mode switch");
//    return;
//  }
//  serCell.println("AT+CMGS=\"8455055932\"");
//  delay(1000);
//  serCell.print(message); serCell.println(char(26));
//  delay(4000);
  serCell.println("AT+CMGS=\"" + PhoneNum + "\"");
  delay(100);
  int success = CellWaitForChar('>');
  if (success == false) { return; }
  serCell.print(message); serCell.println(char(26));
// +CMGS:36
// OK
  #if defined DEBUG
  Serial.println("AT+CMGS=\"" + PhoneNum + "\"");
//  Serial.print("AT+CMGS=");
//  Serial.print('"');
//  Serial.print(PhoneNum);
//  Serial.println('"');       // -was-  \"8453376561\"");
    Serial.println("AT+CMGS=\"" + PhoneNum + "\"");      // -was-  \"8453376561\"");
    Serial.println("SendSMS to " + PhoneNum + " -> \"" + message + "\"");
  #endif
//  CellChk();
}


String WaitForOK() {
  String cellInput;
  int timeout = 5;
  do {
    cellInput = CellWaitForLine();
    if (cellInput == "OK") {
      return "OK";
    }
    if (cellInput.startsWith("+CME ERROR")) {
      return "";
    }
    timeout = timeout - 1;
  } while (timeout > 0);
  Serial.println("Timeout waiting for OK");
  return "";
}

int CellWaitForChar(char inWaitForThis) {
  int timeout=2000;
  do {
    if (serCell.available()) {
      int inByte = serCell.read();
      if (inByte == inWaitForThis) {
        return true;
      }
    }
    timeout = timeout - 1;
    delay(1);
  } while (timeout > 0);
  return false;
}

String CellWaitForLine() {
  String inLine;
  int timeout=1000;
  do {
    if (serCell.available()) {
      int inByte = serCell.read();
      if (inByte == 13) {
        Serial.println(inLine + "= CellWaitForLine()");
        return inLine;
      } else {
        if (inByte != 10) {
          inLine = inLine + char(inByte);
        }
      }
    }
    timeout = timeout - 1;
    delay(1);
  } while (timeout > 0);
  return "";
}

