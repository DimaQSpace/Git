#include <Wire.h>
#include <EEPROM.h>
#define MCP4725 0x60

uint8_t cmd[3];

float inclination_angle = 97.6 * (PI/180); // Угол наклонения орбиты спутника в градусах
float g = 9.8; // Ускорение свободного падения м/c^2
float R_e = 6731; // Радиус земли в км
float H_o = 540; // Высота орбиты спутника в км
float R_o = R_e + H_o; // Радиус орбиты спутника в км
float omega_s = sqrt(g/(R_o*1000)); // Угловая скорость вращения спутника в ГЦИ системе координат с ускорением под действием силы тяжести g и радиусом орбиты R_o
float omega_e = 111.4 * 0.00001; // Угловая скорость вращения земли в рад/c
float omega = omega_s + omega_e * cos(inclination_angle); // Угловая скорость спутника на НОО в ГЦН системе координат
float phi = -10 * (PI/180); // Начальный угол пролета спутника относительно вертикальной оси через центр земли
float gamma = 0.00016; // Угол расхождения спутника мкрад
float U_power = 4.9; // Напряжение питания ардуино в Вольтах
//float Input_Power = 15.1; // Входная оптическая мощность в мВт
float Reference_ADC_Value = 207;
const int flight_time = 300;
float Attenuation_zenith = 0.00003; // Коэффициент потерь в зените
float k_loss = 1/Attenuation_zenith; // Коэффициент для установления рабочего режима аттенюатора

int VOA_Flyby_function[flight_time];
int VOA_function[4096];
int mathematical_expectation_VOA_Value[4096];


float Attenuation(int t) // Расчет необходимого ослабления от времени при пролете спутника
  {
    float alpha = omega * t + phi; // Угол пролета спутника от времени
    //Serial.print("alpha_rad: " + String(alpha,4)+ "\t" + "alpha_deg: " + String(alpha*(180/PI),4) + "\t");
    float theta_EL = acos(sqrt((1-pow(cos(alpha), 2))/(1 + pow((R_e/R_o), 2) - 2 * (R_e/R_o) * cos(alpha)))); // Угол между горизонтом и прямой спутник земля
    //Serial.print("theta_EL_rad: " + String(theta_EL,4) + "\t" + "theta_EL_deg: " + String(theta_EL*(180/PI),4) + "\t");
    float distance = sqrt(pow(R_e * sin(theta_EL),2) + H_o * H_o + 2 * R_e * H_o) - R_e * sin(theta_EL); // Расстояние между наземной станцией и спутником
    //Serial.print("distance: " + String(distance,4) + "\t");
    float Att = ((0.73 * 36 * pow(10,-8))/pow((gamma * distance),2)) * pow(10,(-0.4 * 0.23 * (1/(sin(theta_EL))) * (1 - 0.0012 * pow(1/tan(theta_EL),2)))); //Отношение входной к выходной мощностей
    //Serial.println("Attenuation: " + String(Att,10) + "\t");
    return(Att);
  }

void set_output_level(size_t level, bool save_to_eeprom) // Выставление значения DAC
  {

  // Определяем команду в зависимости от необходимости сохранения значения в EEPROM
    if (save_to_eeprom == true) 
      {
        cmd[0] = 0x60;  // Команда для установки значения на ЦАП с сохранением в EEPROM
      } else 
          {
            cmd[0] = 0x40;  // Команда для установки значения на ЦАП без сохранения в EEPROM
          }

    cmd[1] = level >> 4;
    cmd[2] = level << 4;

    Wire.beginTransmission(MCP4725);

    Wire.write(cmd, 3);

    Wire.endTransmission();

    delayMicroseconds(100);
  }

int get_Ref_ADC_Values(void) // Получение значения со входа в систему
  {
    set_output_level(0, false);
    delay(100);
    unsigned int Ref_Value_Sum = 0;
    Attenuation_zenith = 0;
    for(int i = 0; i < 100; i++) // Определение максимального выходного значения
      {
        Ref_Value_Sum = Ref_Value_Sum + analogRead(0); 
        Serial.println("Ref_Value_Sum: " + String(Ref_Value_Sum) + "\t");
        delay(100);
      }
    for(int i = 0; i < flight_time; i++) // Находим максимальное значение отношения мощностей
      {
        if(Attenuation(i) > Attenuation_zenith)
          {
            Attenuation_zenith = Attenuation(i);
          }
      }
    return(Ref_Value_Sum/100);
  }

void Flyby_Calibration(int Power_In_ADC_Value) // Калибровка аттенюатора
  {
    int j = 0;
    int value = 0;
    float Att = 0;
    k_loss = 1/Attenuation_zenith;
    for (int i = 0; i < flight_time; i++)
      {
        set_output_level(0, false);
        delay(100);
        Att = Attenuation(i) * k_loss * Power_In_ADC_Value;
        //Serial.println("Attenuation: " + String(Attenuation(i),10) + "Att: " + String(Att));
        while (analogRead(0) > Att && j < 4096)
          {
            j++;
            set_output_level(j, false);
            //delay(1);
            //Serial.println("Att * Power_In_ADC_Value: " + String(Att * Power_In_ADC_Value) + "    analogRead: " + String(analogRead(0)) + "   set_output_level: " + String(j));
          }     
        //VOA_Flyby_function[i] = j;
        EEPROM.get(i*2, value);
        if(value != j)
          {
            EEPROM.put(i*2, j);
          }
        Serial.println("i: " + String(i) + " " + "VOA_function: " + String(value) + " " + "j: " + String(j));
        j = 0;         
      }
  }

void Satellite_Flyby (int Number_of_flights) // Пролет спутника
  {
    int value = 0;
    for (int i = 0; i < Number_of_flights; i++)
      {
        for (int t = 0; t < flight_time; t++)
          {
            EEPROM.get(t*2, value);
            set_output_level(value, false);
            delay(1000);
          }
      }
  }

void setup() 
  {
    pinMode(A2, OUTPUT);
    digitalWrite(A2, LOW);
    pinMode(A3, OUTPUT);
    digitalWrite(A3, HIGH);

    Wire.begin();

    Serial.begin(9600);
    Serial.setTimeout(50);

    set_output_level(0, false);

    delay(100);
}

void loop() 
  {
    if (Serial.available())
      {
        String command = Serial.readString();
        Serial.println("Received command: " + command);

        if (command[0] == '1') // Определение входной мощности для калибровки
          {
            Serial.println("Received command: " + String(command[0]) + " - Get referense ADC value");
            Reference_ADC_Value = get_Ref_ADC_Values();
            Serial.println("Reference_ADC_Value: " + String(Reference_ADC_Value));
            Serial.println("Attenuation_zenith: " + String(Attenuation_zenith,10));
            Serial.println("k_loss: " + String(k_loss));
          }
        if (command[0] == '2') // Калибровка системы
          {
            Serial.println("Received command: " + String(command[0]) + " - Calibration started");
            Flyby_Calibration(Reference_ADC_Value);
            Serial.println("Calibration done");
            
          }
        if (command[0] == '3') // Запуск пролета спутника
          {
            Serial.println("Received command: " + String(command[0]) + " - Flight simulation started");
            Satellite_Flyby (1);
            Serial.println("Val0: " + String(command[0]));
          }
        if (command[0] == '4') // Установка значения ЦАП в ручную, формат "4 X", "4 XX", "4 XXX","4 XXXX"
          {
            int DAC_Value = 0;
            
            Serial.println("Received command: " + String(command[0]) + " - Set the DAC Value");
            
            for (int i = 2; i< command.length(); i++)
              {
                DAC_Value = DAC_Value + (command[i] - '0') * round(pow(10, command.length() - i - 1));
              }
            
            Serial.println("DAC Value: " + String(DAC_Value));
            set_output_level(DAC_Value, false);
            Serial.println("set: " + String((float)(analogRead(1) * 5.0) / 1024) + " V");
          }
      }  
  }
