#include <Arduino.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h> 
#include <math.h>
// configurare si definitii
// adrese i2c
#define LCD_ADDR    0x27
#define AHT20_ADDR  0x38
#define BMP280_ADDR 0x77

// constante pentru controlul lcdului prin i2c
#define LCD_CMD     0
#define LCD_DAT     1
#define EN          0x04 // bit de enable
#define BK          0x08 // bit pentru lumina de fundal

// configurare eeprom pentru salvarea setarilor
#define MAGIC_NUMBER 0x44 // verificare scriere eeprom
struct DeviceSettings {
    float t_min, t_opt, t_max;
    float h_low, h_high;
    uint8_t magic;
    float altitude_offset; // corectie altitudine
};
DeviceSettings settings; 
// variabile de stare
bool in_menu = false;           // indicator setari
uint8_t menu_item = 0;          // elementul selectat
volatile uint8_t display_page = 0; // pagina curenta
volatile bool manual_mute = false; // stare mute
volatile uint8_t temp_unit = 0;    // 0 c 1 f 2 k
volatile uint8_t pressure_unit = 0;// 0 mmhg 1 hpa
volatile uint8_t altitude_unit = 0;// 0 m 1 ft
// filtrare date presiune
#define PRESSURE_SAMPLES 10
float p_buffer[PRESSURE_SAMPLES] = {0};
uint8_t p_index = 0;
bool p_filled = false;
static float P0 = 0; // presiunea de referinta pentru calculul altitudinii
// calibrare senzori
constexpr float AHT20_T_OFFSET = -0.8f;
constexpr float BMP280_P_OFFSET = 600.0f;
// gestionare memorie eeprom
// salveaza structura de setari in memoria permanenta
void save_to_eeprom() {
    settings.magic = MAGIC_NUMBER;
    eeprom_update_block((const void*)&settings, (void*)0, sizeof(DeviceSettings));
}
// incarca setarile sau aplica valori implicite daca memoria e goala
void load_from_eeprom() {
    eeprom_read_block((void*)&settings, (const void*)0, sizeof(DeviceSettings));
    if (!isfinite(settings.altitude_offset) || settings.magic != MAGIC_NUMBER) {
        settings.t_min = 5.0f;
        settings.t_opt = 20.0f;
        settings.t_max = 30.0f;
        settings.h_low = 30.0f;
        settings.h_high = 60.0f;
        settings.altitude_offset = 73.0f;
        settings.magic = MAGIC_NUMBER;
        save_to_eeprom();
    }
}

// calcul punct de roua formula magnus tetens
float calculate_dew_point(float t, float h) {
    float a = 17.27f, b = 237.7f;
    float alpha = ((a * t) / (b + t)) + log(h / 100.0f);
    return (b * alpha) / (a - alpha);
}
// filtru medie mobila pentru presiune
float smooth_pressure(float new_val) {
    p_buffer[p_index] = new_val;
    p_index = (p_index + 1) % PRESSURE_SAMPLES;
    if (p_index == 0) p_filled = true;
    uint8_t count = p_filled ? PRESSURE_SAMPLES : p_index;
    float sum = 0;
    for (uint8_t i = 0; i < count; i++) sum += p_buffer[i];
    return sum / count;
}

void i2c_init() { 
    TWSR = 0;
    TWBR = 72;
    TWCR = (1 << TWEN); 
}
bool i2c_start() {
    TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
    uint16_t t = 10000; while (!(TWCR & (1 << TWINT)) && --t);
    return t > 0;
}
void i2c_stop() { 
    TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN); 
}
void i2c_write(uint8_t d) {
    TWDR = d; 
    TWCR = (1 << TWINT) | (1 << TWEN);
    uint16_t t = 10000; while (!(TWCR & (1 << TWINT)) && --t);
}
uint8_t i2c_read(bool ack) {
    TWCR = (1 << TWINT) | (1 << TWEN) | (ack ? (1 << TWEA) : 0);
    uint16_t t = 10000; while (!(TWCR & (1 << TWINT)) && --t);
    return TWDR;
}

void lcd_send(uint8_t val, uint8_t mode) {
    auto p = [&](uint8_t n) {
        i2c_start(); i2c_write(LCD_ADDR << 1);
        i2c_write(n | mode | BK | EN);
        i2c_write(n | mode | BK);
        i2c_stop();
    };
    p(val & 0xF0);
    p((val << 4) & 0xF0);
}
void lcd_init() {
    _delay_ms(50); lcd_send(0x30, LCD_CMD); _delay_ms(5);
    lcd_send(0x30, LCD_CMD); lcd_send(0x32, LCD_CMD);
    lcd_send(0x28, LCD_CMD);
    lcd_send(0x0C, LCD_CMD);
    lcd_send(0x01, LCD_CMD);
    _delay_ms(2);
}
void lcd_print(const char* s) { 
    while (*s) lcd_send(*s++, LCD_DAT); 
}
// intreruperi butoane isr
// pin d3 plus mute intrerupere externa
ISR(INT1_vect) {
    static unsigned long last_t = 0;
    if (millis() - last_t > 250) {
        if (!in_menu) manual_mute = !manual_mute;
        else {
            switch(menu_item) {
                case 0: settings.t_min += 0.5f; 
                        break;
                case 1: settings.t_opt += 0.5f; 
                        break;
                case 2: settings.t_max += 0.5f; 
                        break;
                case 3: settings.h_low += 1.0f; 
                        break;
                case 4: settings.h_high += 1.0f; 
                        break;
                case 5: settings.altitude_offset += 1.0f; 
                        break;
            }
        }
    }
    last_t = millis();
}
// pin d4 minus unitati intrerupere la schimbare pin
ISR(PCINT2_vect) {
    static unsigned long last_t = 0;
    if (!(PIND & (1 << PIND4)) && (millis() - last_t > 250)) {
        if (!in_menu) {
            if (display_page == 0) temp_unit = (temp_unit + 1) % 3;
            else if (display_page == 1) pressure_unit = (pressure_unit + 1) % 2;
            else altitude_unit = (altitude_unit + 1) % 2;
        } else {
            switch(menu_item) {
                case 0: settings.t_min -= 0.5f; 
                        break;
                case 1: settings.t_opt -= 0.5f; 
                        break;
                case 2: settings.t_max -= 0.5f; 
                        break;
                case 3: settings.h_low -= 1.0f; 
                        break;
                case 4: settings.h_high -= 1.0f; 
                        break;
                case 5: settings.altitude_offset -= 1.0f; 
                        break;
            }
        }
    }
    last_t = millis();
}
// control periferice buzzer leduri
void dynamic_buzzer(float t, float lux) {
    bool night_mute = (lux < 10.0f); // dezactivare automata pe timp de noapte
    
    // buzzerul tace daca temperatura este in limite sau daca este activat modul silentios
    if (manual_mute || night_mute || (t < settings.t_max && t > settings.t_min)) {
        TCCR0A &= ~(1 << COM0B1); 
        PORTD &= ~(1 << PD5); 
        return;
    }
    
    TCCR0A |= (1 << COM0B1); // activare pwm pe buzzer pd5
    float severity = (t >= settings.t_max) ? (t - settings.t_max) : (settings.t_min - t);
    OCR0B = (uint8_t)((severity / 10.0f) * 255.0f); // intensitatea sunetului depinde de gravitate
}
void update_leds(float t, float h) {
    
    uint8_t r = 0, g = 0, b = 0;
    
    // logica tranzitie culori rece albastru optim verde fierbinte rosu
    if (t <= settings.t_min) 
        b = 255;
    else if (t < settings.t_opt) { 
        b = map(t*10, settings.t_min*10, settings.t_opt*10, 255, 0); 
        g = map(t*10, settings.t_min*10, settings.t_opt*10, 0, 255); 
    }
    else if (t < settings.t_max) { 
        g = map(t*10, settings.t_opt*10, settings.t_max*10, 255, 0); 
        r = map(t*10, settings.t_opt*10, settings.t_max*10, 0, 255); 
    }
    else { r = 255; }
    // aplicare pwm pentru led rgb pb2r pb1g pd6b
    if (r == 0) { 
        TCCR1A &= ~(1 << COM1B1); 
        PORTB &= ~(1 << PB2); 
    } else { 
        TCCR1A |= (1 << COM1B1); 
        OCR1B = r; }
    if (g == 0) { 
        TCCR1A &= ~(1 << COM1A1); 
        PORTB &= ~(1 << PB1); 
    } else { 
        TCCR1A |= (1 << COM1A1); 
        OCR1A = g; }
    if (b == 0) { 
        TCCR0A &= ~(1 << COM0A1); 
        PORTD &= ~(1 << PD6);
    } else { 
        TCCR0A |= (1 << COM0A1);
         OCR0A = b; 
    }
    // leduri discrete pentru umiditate pb3 low pb4 ok pb5 high
    PORTB &= ~((1 << PB5) | (1 << PB4) | (1 << PB3));
    if (h < settings.h_low) 
        PORTB |= (1 << PB3); 
    else 
        if (h > settings.h_high) 
            PORTB |= (1 << PB5); 
        else 
            PORTB |= (1 << PB4);
}
// bmp280 calcul si compensare
uint16_t dig_T1, dig_P1; 
int16_t dig_T2, dig_T3, dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9; 
int32_t t_fine;

uint16_t bmp_read16(uint8_t reg) {
    i2c_start(); 
    i2c_write(BMP280_ADDR << 1); 
    i2c_write(reg);
    i2c_start(); 
    i2c_write((BMP280_ADDR << 1) | 1);
    uint8_t lo = i2c_read(true);
    uint8_t hi = i2c_read(false);
    i2c_stop(); 
    return (uint16_t)(hi << 8) | lo;
}

void bmp_read_calibration() {
    dig_T1 = bmp_read16(0x88); dig_T2 = (int16_t)bmp_read16(0x8A); 
    dig_T3 = (int16_t)bmp_read16(0x8C);
    dig_P1 = bmp_read16(0x8E); dig_P2 = (int16_t)bmp_read16(0x90); 
    dig_P3 = (int16_t)bmp_read16(0x92);
    dig_P4 = (int16_t)bmp_read16(0x94); 
    dig_P5 = (int16_t)bmp_read16(0x96); 
    dig_P6 = (int16_t)bmp_read16(0x98);
    dig_P7 = (int16_t)bmp_read16(0x9A); 
    dig_P8 = (int16_t)bmp_read16(0x9C); 
    dig_P9 = (int16_t)bmp_read16(0x9E);
}

float bmp_compensate_temp(int32_t adc_T) {
    int32_t v1 = ((((adc_T >> 3) - ((int32_t)dig_T1 << 1))) * ((int32_t)dig_T2)) >> 11;
    int32_t v2 = (((((adc_T >> 4) - ((int32_t)dig_T1)) * ((adc_T >> 4) - ((int32_t)dig_T1))) >> 12) * ((int32_t)dig_T3)) >> 14;
    t_fine = v1 + v2;
    return (float)((t_fine * 5 + 128) >> 8) / 100.0f;
}

float bmp_compensate_pressure(int32_t adc_P) {
    int64_t v1 = ((int64_t)t_fine) - 128000;
    int64_t v2 = v1 * v1 * (int64_t)dig_P6;
    v2 += ((v1 * (int64_t)dig_P5) << 17);
    v2 += ((int64_t)dig_P4 << 35);
    v1 = ((v1 * v1 * (int64_t)dig_P3) >> 8) + ((v1 * (int64_t)dig_P2) << 12);
    v1 = (((int64_t)1 << 47) + v1) * ((int64_t)dig_P1) >> 33;
    if (v1 == 0) 
        return 0;
    int64_t p = 1048576 - adc_P;
    p = (((p << 31) - v2) * 3125) / v1;
    v1 = ((int64_t)dig_P9 * (p >> 13) * (p >> 13)) >> 25;
    v2 = ((int64_t)dig_P8 * p) >> 19;
    return (float)(((p + v1 + v2) >> 8) + ((int64_t)dig_P7 << 4)) / 256.0f;
}

// configurare sistem setup
void setup() {
    load_from_eeprom(); 
    
    // configurare pini de iesire
    DDRB |= 0x3F; // pb0pb5 devin iesiri
    DDRD |= (1 << DDD5) | (1 << DDD6); // pd5 buzzer pd6 led albastru
    
    // configurare rezistente pullup pentru butoane
    PORTD |= (1 << PORTD2) | (1 << PORTD3) | (1 << PORTD4);
    
    // configurare timer 1 fast pwm 8bit pentru led rosu verde
    TCCR1A = (1 << WGM10) | (1 << COM1A1) | (1 << COM1B1);
    TCCR1B = (1 << WGM12) | (1 << CS11) | (1 << CS10);
    
    // configurare timer 0 fast pwm pentru buzzer si led albastru
    TCCR0A |= (1 << COM0A1) | (1 << WGM00) | (1 << WGM01); 
    TCCR0B |= (1 << CS01) | (1 << CS00);
    // activare intreruperi externe
    EICRA |= (1 << ISC11); 
    EIMSK |= (1 << INT1);
    PCICR  |= (1 << PCIE2); 
    PCMSK2 |= (1 << PCINT20);
    
    // configurare adc senzor lumina pin a3
    ADMUX  = (1 << REFS0) | (1 << MUX1) | (1 << MUX0);
    ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
    
    sei(); // activare globala intreruperi
    i2c_init(); 
    lcd_init();
    
    // initializare senzori
    if (i2c_start()) { 
        i2c_write(BMP280_ADDR << 1);
        i2c_write(0xF4);
        i2c_write(0x57);
        i2c_stop(); }
    delay(100); 
    bmp_read_calibration();
    if (i2c_start()) { 
        i2c_write(AHT20_ADDR << 1); 
        i2c_write(0xBE); 
        i2c_write(0x08); 
        i2c_write(0x00); 
        i2c_stop(); 
    }
    
    // mesaj de bun venit pe ecran
    lcd_print("  Statie Meteo  "); 
    lcd_send(0xC0, LCD_CMD); 
    lcd_print("Nicolaescu Alex");
    delay(1500); 
    lcd_send(0x01, LCD_CMD);
}
// ciclul principal loop
void loop() {
    static unsigned long d2_press_time = 0;
    static bool d2_was_pressed = false, long_press_triggered = false;
    // logica buton d2 meniu apasare lunga navigare apasare scurta
    bool d2_pressed = !(PIND & (1 << PIND2));
    if (d2_pressed) {
        if (!d2_was_pressed) { 
            d2_was_pressed = true; d2_press_time = millis();
            long_press_triggered = false; 
        }
        if (!long_press_triggered && (millis() - d2_press_time >= 2000)) {
            in_menu = !in_menu; 
            long_press_triggered = true;
            if (!in_menu) 
                save_to_eeprom(); // salveaza setarile la iesirea din meniu
            lcd_send(0x01, LCD_CMD); 
            if (in_menu) 
                menu_item = 0;
            delay(250);
        }
    } else {
        if (d2_was_pressed) {
            if (!long_press_triggered) {
                if (!in_menu) 
                    display_page = (display_page + 1) % 3;
                else 
                    menu_item = (menu_item + 1) % 6;
                delay(120);
            }
            d2_was_pressed = false;
        }
    }
    // afisare mod meniu
    if (in_menu) {
        char valBuf[10];
        lcd_send(0x80, LCD_CMD); 
        lcd_print("SETARI PRAGURI:");
        lcd_send(0xC0, LCD_CMD);
        switch(menu_item) {
            case 0: lcd_print("T min: "); 
                dtostrf(settings.t_min, 4, 1, valBuf); 
                break;
            case 1: lcd_print("T opt: "); 
                    dtostrf(settings.t_opt, 4, 1, valBuf); 
                    break;
            case 2: lcd_print("T max: "); 
                    dtostrf(settings.t_max, 4, 1, valBuf); 
                    break;
            case 3: lcd_print("H low: "); 
                    dtostrf(settings.h_low, 4, 1, valBuf); 
                    break;
            case 4: lcd_print("H high:"); 
                    dtostrf(settings.h_high, 4, 1, valBuf); 
                    break;
            case 5: lcd_print("Alt offset: "); 
                    dtostrf(settings.altitude_offset, 4, 1, valBuf); 
                    break;
        }
        lcd_print(valBuf); 
        lcd_print("   ");
        delay(100);
    } 
    // afisare mod monitorizare pagini
    else {
        // citire aht20 umiditate si temperatura
        i2c_start(); 
        i2c_write(AHT20_ADDR << 1); 
        i2c_write(0xAC); 
        i2c_write(0x33); 
        i2c_write(0x00); 
        i2c_stop();
        delay(80);
        i2c_start(); 
        i2c_write((AHT20_ADDR << 1) | 1);
        i2c_read(true); 
        uint8_t h1=i2c_read(true), h2=i2c_read(true), h3=i2c_read(true), t1=i2c_read(true), t2=i2c_read(false); 
        i2c_stop();
        
        float h = (((uint32_t)h1 << 12) | ((uint32_t)h2 << 4) | (h3 >> 4)) * 100.0f / 1048576.0f;
        float t = ((((uint32_t)h3 & 0x0F) << 16) | ((uint32_t)t1 << 8) | t2) * 200.0f / 1048576.0f - 50.0f + AHT20_T_OFFSET;
        // citire bmp280 presiune
        i2c_start(); 
        i2c_write(BMP280_ADDR << 1); 
        i2c_write(0xF7);
        i2c_start(); 
        i2c_write((BMP280_ADDR << 1) | 1);
        uint8_t p_msb=i2c_read(true), p_lsb=i2c_read(true), p_xlsb=i2c_read(true), t_msb=i2c_read(true), t_lsb=i2c_read(true), t_xlsb=i2c_read(false); i2c_stop();
        
        bmp_compensate_temp(((int32_t)t_msb << 12) | ((int32_t)t_lsb << 4) | (t_xlsb >> 4));
        float p_pa = bmp_compensate_pressure(((int32_t)p_msb << 12) | ((int32_t)p_lsb << 4) | (p_xlsb >> 4)) + BMP280_P_OFFSET;
        
        if (!isfinite(p_pa) || p_pa <= 0) 
            return;
        if (!isfinite(P0) || P0 <= 0) 
            P0 = p_pa; // prima citire devine referinta pentru calcul
        // calcul altitudine
        float ratio = p_pa / P0;
        if (!isfinite(ratio) || ratio <= 0.0f) 
            ratio = 0.0001f;
        float alt_raw = 44330.0f * (1.0f - powf(ratio, 0.1903f));
        float alt = alt_raw + settings.altitude_offset;
        float p_mm = smooth_pressure(p_pa / 133.322f);
        // citire lux senzor analogic de lumina
        ADCSRA |= (1 << ADSC); 
        while (ADCSRA & (1 << ADSC));
        float lux = (float)ADC * 0.9765f;
        update_leds(t, h);
        dynamic_buzzer(t, lux);
        char buf[16];
        lcd_send(0x01, LCD_CMD); delay(2);
        
        // pagina 0 temperatura umiditate si punct de roua
        if (display_page == 0) {
            float dt = (temp_unit == 1) ? t * 1.8 + 32 : (temp_unit == 2 ? t + 273.15 : t);
            lcd_print("T:"); 
            dtostrf(dt, 5, 1, buf); 
            lcd_print(buf); 
            lcd_print(temp_unit == 1 ? "F" : (temp_unit == 2 ? "K" : "C"));
            if (manual_mute) 
                lcd_print(" [M]"); 
            else if (lux < 10) 
                lcd_print(" [N]");
            
            lcd_send(0xC0, LCD_CMD);
            lcd_print("H:"); 
            dtostrf(h, 2, 0, buf);
            lcd_print(buf); 
            lcd_print("% ROUA:");
            float dp_c = calculate_dew_point(t, h);
            float ddp = (temp_unit == 1) ? dp_c * 1.8 + 32 : (temp_unit == 2 ? dp_c + 273.15 : dp_c);
            dtostrf(ddp, 4, 1, buf);
            lcd_print(buf);
        } 
        // pagina 1 presiune si prognoza simpla
        else if (display_page == 1) {
            float dp = (pressure_unit == 1) ? p_mm * 1.33322f : p_mm;
            lcd_print("P:"); 
            dtostrf(dp, 5, 1, buf); 
            lcd_print(buf); 
            lcd_print(pressure_unit == 1 ? "hPa" : "mmHg");
            
            lcd_send(0xC0, LCD_CMD);
            if (p_mm > 775) 
                lcd_print("Cer senin"); 
            else if (p_mm > 760) 
                lcd_print("Frumos"); 
            else if (p_mm > 745) 
                lcd_print("Variabil"); 
            else if (p_mm > 730) 
                lcd_print("Innorat"); 
            else    
                lcd_print("Ploaie/Furtuna");
        } 
        // pagina 2 altitudine si luminozitate
        else {
            float da = (altitude_unit == 1) ? alt * 3.28084f : alt;
            lcd_print("Alt:"); 
            dtostrf(da, 4, 1, buf); 
            lcd_print(buf); 
            lcd_print(altitude_unit == 1 ? "ft" : "m");
            
            lcd_send(0xC0, LCD_CMD);
            lcd_print("Lum: ");
            dtostrf(lux, 4, 1, buf);
            lcd_print(buf); 
            lcd_print(" lx");
        }
        delay(200);
    }
}