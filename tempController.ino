#include <Adafruit_Sensor.h>
#include <DHT_U.h>
#include <DHT.h>

#include <LiquidCrystal_I2C.h>
#include <SoftwareSerial.h>

#define DHTPIN            12         // DHT sensor to pin 12. 
#define DHTTYPE            DHT11     // DHT 11.

#define BUZZER		6
#define BUZZER_NOTE	2093

#define PWM_PIN		9

#define BLUETOOTH_TX 2
#define BLUETOOTH_RX 3

#define SEP '|'	//구분자

DHT_Unified dht(DHTPIN, DHTTYPE);
uint32_t delayMS;
SoftwareSerial bluetoothSerial(BLUETOOTH_TX, BLUETOOTH_RX);

float pre_delay_buzzer_time = 1.0;	//버저가 울리기 전 기다리는 시간(초 단위)
float buzzer_time = 0.25;	//버저 출력 시간
float set_temp = 30.0;		//설정한 온도
float cur_temp = 0.0;		//현재 온도
bool simulated = false;		//현재 온도 시뮬레이션
bool on = true;				//프로그램 on 상태

LiquidCrystal_I2C lcd(0x27, 16, 2);  // Set the LCD I2C address. Use 16-kan2 line LCD. 
									// Put the scanned address instead of 0x27. 

void LCD_Init()
{
	lcd.init();
	lcd.backlight();                //  Turn on the backlight. (lcd.noBacklight() turns off the backlight.) 
}

void LCD_Write(uint8_t column, uint8_t line, const char* prn)
{
	lcd.setCursor(column, line);                 // first line first column
	lcd.print(prn);
}

void Pre_Delay_Buzzer()
{
	unsigned long delayTime = (unsigned long)(pre_delay_buzzer_time * 1000);
	delay(delayTime);

}

void Buzzer()	//시간만큼 부저가 울린다.
{
	int sec = (int)(1000 * buzzer_time);

	tone(BUZZER, BUZZER_NOTE, sec);       //Connect the Piezo buzzer to No. 6 
	int pauseBetweenNotes = sec * 1.30;          //phonetic delimiting 
	delay(pauseBetweenNotes);                           //delay
	noTone(BUZZER);
}

int calcAnalogValueCurrentPerMax(float current, float max)		// 1.0 / 5.0
{
	int retValue = (int)(255 * (current / max));
	return retValue;
}

float GetTemp()
{
	delay(delayMS);
	sensors_event_t event;
	dht.temperature().getEvent(&event);  // Get temperature event and print its value. 

	if (isnan(event.temperature)) {
		Serial.println("Error reading temperature!");
		return 0.0;
	}
	else
	{
		/*
		Serial.print("Temperature: ");
		Serial.print(event.temperature);
		Serial.println(" *C");
		*/

		return event.temperature;
	}
}

String GetBluetoothMsg()
{
	String retString = "";	
	while (bluetoothSerial.available())
	{
		char myChar = (char)bluetoothSerial.read();
		retString += myChar;
		delay(5);
	}
	return retString;
}

//블루투스 처리
void Proc_Bluetooth()
{
	String bluetooth_msg = GetBluetoothMsg();
	if (!bluetooth_msg.equals(""))
	{
		Serial.print("Recv From Bluetooth : ");
		Serial.println(bluetooth_msg);

		String cmd = Split(bluetooth_msg, 0);

		Serial.println(cmd);
		if (cmd == "GET")
		{	
			String writeData = "";
			writeData += set_temp;
			writeData += SEP;
			writeData += cur_temp;
			writeData += SEP;
			writeData += pre_delay_buzzer_time;
			writeData += SEP;
			writeData += buzzer_time;

			Serial.print("Send To Bluetooth : ");
			Serial.println(writeData);

			bluetoothSerial.write(writeData.c_str());
		}
		else if (cmd == "SET")
		{			
			simulated = Split(bluetooth_msg, 1).toInt() == 1;		
			set_temp = Split(bluetooth_msg, 2).toFloat();

			if(simulated)
				cur_temp = Split(bluetooth_msg, 3).toFloat();

			pre_delay_buzzer_time = Split(bluetooth_msg, 4).toFloat();
			buzzer_time = Split(bluetooth_msg, 5).toFloat();

			lcd.clear();
			LCD_Write(0, 0, "Set Property");
			delay(2000);
			lcd.clear();
		}
	}
}

//온도에 따라 액션
void Action_Temp(float diff_temp)
{
	if (on)
	{
		//온도에 따른 액션
		if (diff_temp <= 0)
		{
			Pre_Delay_Buzzer();
			Buzzer();

			on = false;	//종료
			analogWrite(PWM_PIN, 0);
			delay(100);
		}
		else if (diff_temp <= 5.0)
		{
			int pmwPower = calcAnalogValueCurrentPerMax(diff_temp, 5.0);
			analogWrite(PWM_PIN, pmwPower);
			delay(100);
		}
		else
		{
			analogWrite(PWM_PIN, 255);
		}
	}
}



//String Split
String Split(String data, int index)
{
	String copy = data;	
	for (int i = 0; i <= index; i++)
	{
		int nIndex = copy.indexOf(SEP);
		if (-1 != nIndex)
		{
			String cur = copy.substring(0, nIndex);
			if (i == index)
				return cur;

			copy = copy.substring(nIndex + 1);
		}
		else
		{
			if (i == index)
				return copy;
			else
				return "";
		}
	}
}

// The setup() function runs once each time the micro-controller starts
void setup()
{
	Serial.begin(9600);
	bluetoothSerial.begin(9600);

	dht.begin();

	LCD_Init();

	pinMode(PWM_PIN, OUTPUT);

	// 센서 정보
	sensor_t sensor;
	dht.temperature().getSensor(&sensor);
	delayMS = sensor.min_delay / 1000;

	/*
	Serial.print(sensor.max_value); Serial.print(" *C");
	Serial.println(sensor.min_delay);
	*/
}

// Add the main program code into the continuous loop() function
void loop()
{
	if (!simulated)
		cur_temp = GetTemp();

	float diff_temp = set_temp - cur_temp;

	if (!on && diff_temp > 5.0)
		on = true;

	Proc_Bluetooth();
	Action_Temp(diff_temp);

	LCD_Write(0, 0, "SetTemp : ");
	lcd.print(set_temp, 1);

	LCD_Write(0, 1, "CurTemp : ");
	lcd.print(cur_temp, 1);
}
