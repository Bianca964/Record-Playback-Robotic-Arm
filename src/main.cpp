#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <util/delay.h>

// Definitii Pini Servomotoare
#define SERVO1_PIN PB3 // D11 (Baza)
#define SERVO2_PIN PB2 // D10 (Umar)
#define SERVO3_PIN PB1 // D9  (Cot)
#define SERVO4_PIN PD5 // D5  (Cleste)

#define BTN_REC_PIN PD2  // Joystick Stâng (Click)
#define LED_PIN PD3      // LED-ul rosu de pe shield este conectat la pinul D3

#define MODE_MANUAL 0
#define MODE_PLAY 1

volatile uint16_t servo_ticks[4] = {3000, 3000, 3000, 3000}; // stocheaza latimea impulsului PWM pentru fiecare motor (3000 ticks = 1.5ms = mijloc)
volatile uint8_t pwm_slot = 0; // contorizeaza care motor este controlat in secunda curenta
volatile uint8_t tick_20ms = 0;  // devine 1 la fiecare 20 de ms ca sa anunte ca a trecut un ciclu complet

#define MAX_FRAMES 50
uint8_t EEMEM ee_frames[MAX_FRAMES][4];  // matricea salvata direct in EEPROM (nevolatila)
uint8_t EEMEM ee_frame_count;            // salveaza in EEPROM numarul total de cadre inregistrate

// {Baza, Umarul, Cotul, Clestele}
uint8_t current_pos[4] = {127, 50, 127, 127}; 
uint8_t target_pos[4] = {127, 50, 127, 127};
uint8_t saved_frame_count = 0;      // contor in memoria RAM pentru nr de cadre salvate curent



// ==============================================================================
// MODUL HARDWARE I2C / TWI (BARE-METAL) - CU PROTECTIE ANTI-BLOCARE
// ==============================================================================
uint8_t oled_conectat = 1; // 1 = Ecranul merge; 0 = Ecranul e mort/lipsa

void i2c_init(void) {
    TWSR = 0x00; // Prescaler = 1
    TWBR = 0x48; // Setează frecvența SCL la ~100kHz
    TWCR = (1 << TWEN); // Activează modulul hardware TWI
}

void i2c_start(void) {
    if (!oled_conectat) return; // daca ecranul nu raspunde, ignoram
    TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
    uint16_t timeout = 5000; // Timp limita de asteptare
    while (!(TWCR & (1 << TWINT)) && timeout > 0) {
        timeout--; // Scade pana la 0 dacă ecranul e scos
    }

    if (timeout == 0) oled_conectat = 0; // DEZACTIVARE INSTANTANEE!
}

void i2c_stop(void) {
    if (!oled_conectat) return;
    TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN);
    _delay_us(10);
}

void i2c_write(uint8_t data) {
    if (!oled_conectat) return;
    TWDR = data;
    TWCR = (1 << TWINT) | (1 << TWEN);
    uint16_t timeout = 5000; // Timp limită de așteptare
    while (!(TWCR & (1 << TWINT)) && timeout > 0) {
        timeout--;
    }

    if (timeout == 0) oled_conectat = 0; // DEZACTIVARE INSTANTANEE!
}

// ==============================================================================
// DRIVER OLED SSD1306 MINIMALIST (BARE-METAL)
// ==============================================================================
#define OLED_ADDR 0x78 // Adresa standard I2C pentru ecranele OLED de 0.96"

void oled_command(uint8_t cmd) {
    if (!oled_conectat) return;
    i2c_start();
    i2c_write(OLED_ADDR);
    i2c_write(0x00); // 0x00 indică flux de comenzi
    i2c_write(cmd);
    i2c_stop();
}

void oled_init(void) {
    if (!oled_conectat) return;
    _delay_ms(100); // Așteaptă stabilizarea tensiunii pe display
    oled_command(0xAE); // Oprește ecranul
    oled_command(0x20); oled_command(0x02); // Setare mod adresare pagină
    oled_command(0xB0); // Setează pagina de start la 0
    oled_command(0x00); oled_command(0x10); // Setează coloana de start la 0
    oled_command(0x8D); oled_command(0x14); // Activează pompa de încărcare (Charge Pump)
    oled_command(0xAF); // Pornește ecranul (pixelii prind viață)
}

void oled_clear(void) {
    if (!oled_conectat) return;
    for (uint8_t p = 0; p < 8; p++) {
        oled_command(0xB0 + p); // Treci la pagina următoare
        oled_command(0x00); oled_command(0x10); // Resetează coloana
        i2c_start();
        i2c_write(OLED_ADDR);
        i2c_write(0x40); // 0x40 indică flux de date (pixeli)
        for (uint16_t c = 0; c < 128; c++) {
            i2c_write(0x00); // Pune toți pixelii pe 0 (Stinge ecranul)
        }
        i2c_stop();
    }
}

// Matrice bitmap minimală 5x7 pentru caracterele esențiale proiectului
const uint8_t font_bits[][5] = {
    {0x00, 0x00, 0x00, 0x00, 0x00}, // [0] Spațiu
    {0x7E, 0x11, 0x11, 0x11, 0x7E}, // [1] A
    {0x7F, 0x49, 0x49, 0x49, 0x36}, // [2] B
    {0x3E, 0x41, 0x41, 0x41, 0x22}, // [3] C
    {0x7F, 0x41, 0x41, 0x22, 0x1C}, // [4] D
    {0x7F, 0x49, 0x49, 0x49, 0x41}, // [5] E
    {0x7F, 0x40, 0x40, 0x40, 0x40}, // [6] L
    {0x7F, 0x02, 0x0C, 0x02, 0x7F}, // [7] M
    {0x7F, 0x04, 0x08, 0x10, 0x7F}, // [8] N
    {0x3E, 0x41, 0x41, 0x41, 0x3E}, // [9] O
    {0x7F, 0x09, 0x09, 0x09, 0x06}, // [10] P
    {0x7F, 0x09, 0x19, 0x29, 0x46}, // [11] R
    {0x01, 0x01, 0x7F, 0x01, 0x01}, // [12] T
    {0x3F, 0x40, 0x40, 0x40, 0x3F}, // [13] U
    {0x03, 0x0C, 0x70, 0x0C, 0x03}, // [14] Y
    {0x61, 0x51, 0x49, 0x45, 0x43}, // [15] Z
    {0x00, 0x00, 0x36, 0x36, 0x00}, // [16] :
    // SĂGEȚI
    {0x10, 0x38, 0x7C, 0x10, 0x10}, // [17] Săgeată SUS (^)
    {0x10, 0x10, 0x7C, 0x38, 0x10}, // [18] Săgeată JOS (v)
    {0x10, 0x30, 0x7F, 0x30, 0x10}, // [19] Săgeată STÂNGA (<-)
    {0x10, 0x0C, 0x7E, 0x0C, 0x10}  // [20] Săgeată DREAPTA (->)
};

void oled_char(uint8_t font_idx) {
    if (!oled_conectat) return;
    i2c_start();
    i2c_write(OLED_ADDR);
    i2c_write(0x40);
    for (uint8_t i = 0; i < 5; i++) {
        i2c_write(font_bits[font_idx][i]);
    }
    i2c_write(0x00); // Spațiu de 1 pixel între litere
    i2c_stop();
}

// Funcție care setează coordonatele cursorului pe ecran
void oled_set_cursor(uint8_t page, uint8_t column) {
    if (!oled_conectat) return;
    oled_command(0xB0 + page);
    oled_command(0x00 + (column & 0x0F));
    oled_command(0x10 + ((column >> 4) & 0x0F));
}

// Desenează interfața de bază fixă, salvând timp de procesare
void draw_ui_static(void) {
    if (!oled_conectat) return;
    oled_clear();
    // Scrie "MOD:" pe prima linie
    oled_set_cursor(0, 0);
    oled_char(7); oled_char(9); oled_char(4); oled_char(16); // M, O, D :
    
    // Scrie denumirile motoarelor pe linii diferite
    oled_set_cursor(2, 0); oled_char(2); oled_char(1); oled_char(15); oled_char(1); oled_char(16); // BAZA:
    oled_set_cursor(4, 0); oled_char(13); oled_char(7); oled_char(1); oled_char(11); oled_char(16); // UMAR:
    oled_set_cursor(6, 0); oled_char(3); oled_char(9); oled_char(12); oled_char(16); // COT:
}

// Actualizează elementele în mișcare (Status + Săgeți)
void update_ui_dynamic(uint8_t mode, uint16_t j_baza, uint16_t j_umar, uint16_t j_cot) {
    if (!oled_conectat) return;
    // 1. Actualizare text Mod Functionare
    oled_set_cursor(0, 36);
    if (mode == MODE_MANUAL) {
        oled_char(7); oled_char(1); oled_char(8); oled_char(13); oled_char(1); oled_char(6); // MANUAL
    } else {
        oled_char(10); oled_char(6); oled_char(1); oled_char(14); oled_char(0); oled_char(0); // PLAY
    }

    // 2. Dinamică Săgeată Bază (Axa X)
    oled_set_cursor(2, 50);
    if (j_baza > 600)      oled_char(18); // ->
    else if (j_baza < 400) oled_char(17); // <-
    else                   oled_char(0);  // Șterge săgeata dacă e pe centru

    // 3. Dinamică Săgeată Umăr (Axa Y)
    oled_set_cursor(4, 50);
    if (j_umar > 600)      oled_char(19); // v
    else if (j_umar < 400) oled_char(20); // ^
    else                   oled_char(0);

    // 4. Dinamică Săgeată Cot (Axa Y de la joystick-ul al doilea)
    oled_set_cursor(6, 50);
    if (j_cot > 600)      oled_char(19); // v
    else if (j_cot < 400) oled_char(20); // ^
    else                   oled_char(0);
}



// ==============================================================================
// TIMERE (Generare semnal PWM 50Hz)
// ==============================================================================
void timer1_init() {
    TCCR1A = 0; 
    TCCR1B = (1 << WGM12) | (1 << CS11); // mod CTC (numara pana la o tinta fixa) si Prescaler 8 (viteza ceas)
    OCR1A = 5000;                        // tinta la 5000 (5000 de numarari inseamna fix 2.5ms)
    TIMSK1 = (1 << OCIE1A) | (1 << OCIE1B); // activez intreruperile Compare Match A si B pentru Timer1
}

ISR(TIMER1_COMPA_vect) {
    pwm_slot++;
    if (pwm_slot >= 8) { 
        pwm_slot = 0; 
        tick_20ms = 1; 
    }
    
    // la inceputul fiecarui slot din primele 4, pornim curentul (punem 1 logic) pe pinul motorului respectiv
    if (pwm_slot == 0) {  // Porneste Servo 1
        PORTB |= (1 << SERVO1_PIN); 
        OCR1B = servo_ticks[0]; 
    } else if (pwm_slot == 1) { // Porneste Servo 2
        PORTB |= (1 << SERVO2_PIN); 
        OCR1B = servo_ticks[1]; 
    } else if (pwm_slot == 2) { // Porneste Servo 3
        PORTB |= (1 << SERVO3_PIN); 
        OCR1B = servo_ticks[2]; 
    } else if (pwm_slot == 3) { // Porneste Servo 4
        PORTD |= (1 << SERVO4_PIN); 
        OCR1B = servo_ticks[3]; 
    }
}

ISR(TIMER1_COMPB_vect) {
    // cand Timerul ajunge la valoarea din OCR1B opresc curentul (pun 0 logic) pe pinul motorului din slotul curent
    if (pwm_slot == 0)      {
        PORTB &= ~(1 << SERVO1_PIN); 
    } else if (pwm_slot == 1) { 
        PORTB &= ~(1 << SERVO2_PIN); 
    } else if (pwm_slot == 2) { 
        PORTB &= ~(1 << SERVO3_PIN); 
    } else if (pwm_slot == 3) { 
        PORTD &= ~(1 << SERVO4_PIN); 
    }
}

// ADC (Citire Joystick)
void adc_init() {
    // setez tensiunea de referinta la 5V (tensiunea placii)
    ADMUX = (1 << REFS0);
    // activez modulul ADC si ii setez o viteza stabila (Prescaler 128)
    ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); 
}

uint16_t adc_read(uint8_t channel) {
    ADMUX = (ADMUX & 0xF0) | (channel & 0x0F); // Selectăm pinul analogic din care vrem să citim (A0, A1, A2 sau A3)
    ADCSRA |= (1 << ADSC);                     // Pornim conversia hardware (citirea tensiunii)
    while (ADCSRA & (1 << ADSC));              // Așteptăm pe loc până când cipul termină de citit
    return ADC;                                // Returnăm valoarea citită (0-1023)        
}

void update_servo_ticks() {
    for (uint8_t i = 0; i < 4; i++) {
        // Transformă valoarea simplă de poziție (0-255) în ticks pentru Timer (2000 înseamnă 1ms, 4000 înseamnă 2ms)
        servo_ticks[i] = 2000 + (((uint32_t)current_pos[i] * 2000) / 255);
    }
}

int main(void) {
    // 1. Motoarele ca OUTPUT & LED
    DDRB |= (1 << SERVO1_PIN) | (1 << SERVO2_PIN) | (1 << SERVO3_PIN);
    DDRD |= (1 << SERVO4_PIN);
    
    // 2. LED-ul Rosu de pe D3 ca OUTPUT
    DDRD |= (1 << LED_PIN);
    PORTD &= ~(1 << LED_PIN); // stins complet la început

    // 3. Butonul REC (Joystick Stâng) ca INPUT PULLUP
    DDRD &= ~(1 << BTN_REC_PIN);
    PORTD |= (1 << BTN_REC_PIN); // Activăm rezistența internă de Pull-up (ține pinul în 5V până e apăsat)

    // 4. Activăm Pull-Up pe TOȚI pinii posibili pentru butonul PLAY
    // D4, D6, D7 (pe portul D) și D8 (pe portul B)
    DDRD &= ~((1 << PD4) | (1 << PD6) | (1 << PD7));
    PORTD |= ((1 << PD4) | (1 << PD6) | (1 << PD7));
    DDRB &= ~(1 << PB0);
    PORTB |= (1 << PB0);
    // Activăm Pull-Up intern pentru pinii I2C (A4 și A5, care corespund portului PC4 și PC5)
    // Asta forțează ecranul să "vorbească" dacă îi lipseau rezistențele fizice
    DDRC &= ~((1 << PC4) | (1 << PC5));
    PORTC |= ((1 << PC4) | (1 << PC5));

    // Initializare hardware ecran si cititoare
    i2c_init();     // Porneste magistrala I2C
    oled_init();    // Trimite comenzile de setup catre OLED
    draw_ui_static(); // Deseneaza textul fix pe ecran (eficientizare viteza)

    timer1_init();  // Pornim Timerul și sistemul PWM
    adc_init();     // Pornim cititorul de joystick-uri
    
    saved_frame_count = eeprom_read_byte(&ee_frame_count);
    if (saved_frame_count == 0xFF) { // EEPROM neinitializata (valoare default 0xFF), deci o setam la 0
        saved_frame_count = 0;
    }

    sei(); 
    update_servo_ticks();

    uint8_t mode = MODE_MANUAL;
    uint8_t play_index = 0;
    
    uint8_t last_rec_state = 1;
    uint8_t last_play_state = 1;

    // Variabile temporare pentru stocare pozitii joystick
    uint16_t j_baza = 512, j_umar = 512, j_cot = 512, j_cleste = 512;

    while (1) {
        if (tick_20ms) {
            tick_20ms = 0;

            // Citim butonul REC normal
            uint8_t current_rec_state = (PIND & (1 << BTN_REC_PIN)) ? 1 : 0;
            
            // Citim butonul PLAY (Dacă ORICARE din pini e tras la masă, înseamnă că l-am apăsat)
            uint8_t current_play_state = 1; 
            if (!(PIND & (1 << PD4)) || !(PIND & (1 << PD6)) || !(PIND & (1 << PD7)) || !(PINB & (1 << PB0))) {
                current_play_state = 0; // S-a apasat joystick-ul drept!
            }

            // ================= LOGICA PLAY/STOP =================
            if (current_play_state == 0 && last_play_state == 1) {  // daca s-a apasat butonul PLAY (joystick drept)
                if (mode == MODE_MANUAL && saved_frame_count > 0) {
                    mode = MODE_PLAY;
                    play_index = 0;
                    eeprom_read_block((void*)target_pos, (const void*)ee_frames[play_index], 4);

                    // APRIND LED-UL PERMANENT ÎN MODUL PLAY
                    PORTD |= (1 << LED_PIN);
                } else {
                    mode = MODE_MANUAL;

                    // STING LED-UL CÂND REVENIM ÎN MODUL MANUAL
                    PORTD &= ~(1 << LED_PIN);
                }
            }

            // ================= MODUL MANUAL =================
            if (mode == MODE_MANUAL) {
                j_baza = adc_read(0);
                j_umar = adc_read(1);
                j_cot  = adc_read(2);
                j_cleste = adc_read(3);

                uint8_t speed = 2; //  3 daca vreau sa se miste mai repede

                // if (j_baza > 600 && current_pos[0] < 250) current_pos[0] += speed;
                // if (j_baza < 400 && current_pos[0] > 5)   current_pos[0] -= speed;
                if (j_baza > 600 && current_pos[0] > 5)   current_pos[0] -= speed;
                if (j_baza < 400 && current_pos[0] < 250) current_pos[0] += speed;

                // if (j_umar > 600 && current_pos[1] < 250) current_pos[1] += speed;
                // if (j_umar < 400 && current_pos[1] > 5)   current_pos[1] -= speed;
                if (j_umar > 600 && current_pos[1] > 5)     current_pos[1] -= speed;
                if (j_umar < 400 && current_pos[1] < 250)   current_pos[1] += speed;

                // if (j_cot > 600 && current_pos[2] < 250)  current_pos[2] += speed;
                // if (j_cot < 400 && current_pos[2] > 5)    current_pos[2] -= speed;
                if (j_cot > 600 && current_pos[2] > 5)    current_pos[2] -= speed;
                if (j_cot < 400 && current_pos[2] < 250)  current_pos[2] += speed;

                if (j_cleste > 600 && current_pos[3] < 250) current_pos[3] += speed;
                if (j_cleste < 400 && current_pos[3] > 5)   current_pos[3] -= speed;

                update_servo_ticks();

                // SALVARE CADRU (RECORD)
                if (current_rec_state == 0 && last_rec_state == 1) { // daca s-a apasat butonul REC (joystick stâng)
                    if (saved_frame_count < MAX_FRAMES) {
                        PORTD |= (1 << LED_PIN); // APRINDEM LED-ul ROȘU PUTERNIC SCURT
                        
                        eeprom_write_block((const void*)current_pos, (void*)ee_frames[saved_frame_count], 4);
                        saved_frame_count++;
                        eeprom_write_byte(&ee_frame_count, saved_frame_count);
                        
                        _delay_ms(200);           // tin ledul aprins pentru 200ms ca sa se vada clar ca s-a salvat un cadru
                        PORTD &= ~(1 << LED_PIN); // STINGEM LED-ul INAPOI
                    }
                }
            }

            // ================= MODUL PLAYBACK =================
            if (mode == MODE_PLAY) {
                uint8_t reached_target = 1;  // presupunem ca am ajuns la tinta, daca oricare din motoare nu e acolo, o sa setam la 0

                // In modul PLAY, resetam indicatorii ca fiind centrali
                j_baza = 512; j_umar = 512; j_cot = 512;

                // Determinam directiile dinamice pe baza diferentei dintre pozitia curenta si cea tinta
                // Setam valori "false" in variabilele j_x pentru a activa sagetile pe ecran
                if (current_pos[0] < target_pos[0]) j_baza = 300; // Se misca dreapta (genereaza sageata -> pe font inversat)
                else if (current_pos[0] > target_pos[0]) j_baza = 700; // Se misca stanga (genereaza sageata <-)
                
                if (current_pos[1] < target_pos[1]) j_umar = 300; // Se misca sus
                else if (current_pos[1] > target_pos[1]) j_umar = 700; // Se misca jos
                
                if (current_pos[2] < target_pos[2]) j_cot = 300; // Se misca sus
                else if (current_pos[2] > target_pos[2]) j_cot = 700; // Se misca jos

                // Interpolare software: miscare lenta, cu cate un pas, spre valorile din target_pos
                for (uint8_t i = 0; i < 4; i++) {
                    if (current_pos[i] < target_pos[i]) {
                        current_pos[i]++;
                        reached_target = 0;
                    } else if (current_pos[i] > target_pos[i]) {
                        current_pos[i]--;
                        reached_target = 0;
                    }
                }

                update_servo_ticks(); 

                // Dacă am ajuns la poziția țintă, încă așteptăm puțin acolo pentru a se vedea mișcarea, apoi trecem la următorul cadru
                if (reached_target) {
                    play_index++;
                    if (play_index >= saved_frame_count) {
                        play_index = 0; // daca am ajuns la ultimul cadru, ne întoarcem la primul pentru a repeta secventa
                    }
                    eeprom_read_block((void*)target_pos, (const void*)ee_frames[play_index], 4);
                    _delay_ms(300); 
                }
            }

            // Update ecranul OLED o data la 20ms cu datele noi
            update_ui_dynamic(mode, j_baza, j_umar, j_cot);

            last_rec_state = current_rec_state;
            last_play_state = current_play_state;
        }
    }
    return 0;
}
