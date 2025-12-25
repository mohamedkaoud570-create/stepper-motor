#define F_CPU 8000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <stdlib.h>
#include <math.h>

typedef unsigned char u8;
typedef signed char s8;
typedef signed long s32;

// --- Macros ---
#define SET_BIT(VAR,BITNO) (VAR) |=  (1 << (BITNO))
#define CLR_BIT(VAR,BITNO) (VAR) &= ~(1 << (BITNO))
#define GET_BIT(VAR,BITNO) (((VAR) >> (BITNO)) & 0x01)

// --- CONFIG ---
#define LCD_DATA_PORT    PORTA
#define LCD_DATA_DIR     DDRA
#define LCD_CTRL_PORT    PORTB
#define LCD_CTRL_DIR     DDRB
#define LCD_RS 1
#define LCD_RW 2
#define LCD_EN 3

#define KPD_PORT         PORTD
#define KPD_PIN          PIND
#define KPD_DIR          DDRD
#define NOT_PRESSED      0xFF

#define STEPPER_PORT     PORTC
#define STEPPER_DIR      DDRC
#define STEPS_PER_REV    64

/**************** 1. LCD & KEYPAD FUNCTIONS ****************/

void LCD_SendCommand(u8 cmd) {
	CLR_BIT(LCD_CTRL_PORT, LCD_RS); CLR_BIT(LCD_CTRL_PORT, LCD_RW);
	LCD_DATA_PORT = cmd; SET_BIT(LCD_CTRL_PORT, LCD_EN);
	_delay_ms(1); CLR_BIT(LCD_CTRL_PORT, LCD_EN); _delay_ms(2);
}
void LCD_SendData(u8 data) {
	SET_BIT(LCD_CTRL_PORT, LCD_RS); CLR_BIT(LCD_CTRL_PORT, LCD_RW);
	LCD_DATA_PORT = data; SET_BIT(LCD_CTRL_PORT, LCD_EN);
	_delay_ms(1); CLR_BIT(LCD_CTRL_PORT, LCD_EN); _delay_ms(2);
}
void LCD_Init(void) {
	LCD_DATA_DIR = 0xFF; LCD_CTRL_DIR |= (1<<LCD_RS)|(1<<LCD_RW)|(1<<LCD_EN);
	_delay_ms(20); LCD_SendCommand(0x38); LCD_SendCommand(0x0C); LCD_SendCommand(0x01);
}
void LCD_PrintString(char *str) { while(*str) LCD_SendData(*str++); }
void LCD_Clear(void) { LCD_SendCommand(0x01); _delay_ms(2); }
void LCD_GoTo(u8 row, u8 col) { LCD_SendCommand((row==0)? (0x80+col):(0xC0+col)); }
void LCD_WriteNum(s32 num) { char buffer[16]; itoa(num, buffer, 10); LCD_PrintString(buffer); }

u8 KPD_GetPressed(void) {
	u8 col, row; KPD_DIR = 0x0F; KPD_PORT = 0xFF;
	for(col=0; col<4; col++) {
		CLR_BIT(KPD_PORT, col);
		for(row=0; row<4; row++) {
			if(GET_BIT(KPD_PIN, row+4) == 0) {
				u8 map[4][4] = {{'7','8','9','/'},{'4','5','6','*'},{'1','2','3','-'},{'C','0','=','+'}};
				u8 key = map[row][col];
				while(GET_BIT(KPD_PIN, row+4) == 0); _delay_ms(20); return key;
			}
		}
		SET_BIT(KPD_PORT, col);
	}
	return NOT_PRESSED;
}

/**************** 2. PRECISE STEPPER LOGIC  ****************/
const u8 full_step_sequence[4] = { 0b1001, 0b0011, 0b0110, 0b1100};

void Stepper_Go_To_Angle(s32 target_angle) {
	float steps_needed = (fabs(target_angle) * (float)STEPS_PER_REV) / 360.0;
	s32 total_steps = (s32)(steps_needed + 0.5);
	static s8 current_idx = 0;
	u8 direction = (target_angle > 0) ? 1 : 0;

	for (s32 i = 0; i < total_steps; i++) {
		if (direction == 1) current_idx++; else current_idx--;
		if (current_idx > 3) current_idx = 0;
		if (current_idx < 0) current_idx = 3;
		STEPPER_PORT = (STEPPER_PORT & 0xF0) | full_step_sequence[current_idx];
		_delay_ms(20);
	}
	STEPPER_PORT &= 0xF0;
}

/**************** 3. USER INTERFACE LOGIC ****************/
s32 Get_Number_Input(char* prompt) {
	s32 val = 0; u8 key; s8 sign = 1;
	LCD_Clear(); LCD_PrintString(prompt); LCD_GoTo(1,0);
	while(1) {
		key = KPD_GetPressed();
		if(key != NOT_PRESSED) {
			if(key == '=') break;
			if(key == '-') { sign = -1; LCD_SendData('-'); }
			else if(key >= '0' && key <= '9') { LCD_SendData(key); val = val * 10 + (key - '0'); }
			else if(key == 'C') { val = 0; sign = 1; LCD_Clear(); LCD_PrintString(prompt); LCD_GoTo(1,0); }
		}
	}
	return val * sign;
}

/**************** 4. MAIN PROGRAM ****************/
int main(void) {
	LCD_Init();
	STEPPER_DIR |= 0x0F;

	// --- Welcome Screen ---
	LCD_Clear();
	LCD_GoTo(0, 4);
	LCD_PrintString("Welcome");
	_delay_ms(200);

	while(1) {
		LCD_Clear();
		LCD_PrintString("1-Angle  2-rotate");

		u8 choice = NOT_PRESSED;
		while(choice == NOT_PRESSED) { choice = KPD_GetPressed(); }

		if (choice == '1') {
			s32 angle = Get_Number_Input("Enter Angle:");
			LCD_Clear(); LCD_PrintString("Moving...");
			Stepper_Go_To_Angle(angle);
		}
		else if (choice == '2') {
			s32 revs = Get_Number_Input("Enter rot num:");
			s32 angle_from_revs = revs * 360;
			LCD_Clear(); LCD_PrintString("Moving...");
			Stepper_Go_To_Angle(angle_from_revs);
		}

		LCD_Clear();
		LCD_PrintString("Done!");
		_delay_ms(200);
	}
}
