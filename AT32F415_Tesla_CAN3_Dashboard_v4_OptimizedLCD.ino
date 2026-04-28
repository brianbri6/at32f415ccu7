/*
  AT32F415CCU7 Tesla Model S CAN3 Read-Only Dashboard - Optimized LCD
  Target core: brianb:at32f415 v0.3.2 or newer

  What this sketch does:
    - Draws labels once, then updates only changed value rectangles.
    - Optimized LCD text renderer streams whole character/value windows instead of opening a window for every font pixel.
    - Initializes the 320x480 8-bit parallel LCD.
    - Turns on the backlight on PA15.
    - Initializes CAN1 in LISTEN-ONLY mode.
    - Uses PA11 pin 32 as CAN RXD.
    - PA12 pin 33 is CAN TXD-safe, held input-pullup/recessive.
    - Does NOT transmit.
    - Does NOT ACK.
    - Does NOT send heartbeat frames.
    - Shows last received CAN frame on the LCD.

  Known project pin map:
    LCD data D0..D7 : PA0..PA7
    LCD WR/SCL      : PC13
    LCD RS/DC       : PC15
    LCD RESET       : PB4
    LCD backlight   : PA15

    CAN RXD         : PA11, pin 32
    CAN TXD         : PA12, pin 33, held input-pullup/recessive

  Notes:
    - v6 switches the AT32F415 from reset-default 8 MHz to 48 MHz HICK.
      CAN timing below assumes APB1/PCLK1 is 48 MHz.
    - For vehicle HS-CAN/OBD, start with 500 kbps.
    - If CAN init fails, the screen shows STAGE/MSTS/MCTRL/BTMG/APB1 diagnostics.
    - If it initializes but you see no frames, try CAN_BITRATE_KBPS 250 or 125.
*/

#include <Arduino.h>
#include <stdint.h>
#include <stdio.h>

/* ================= USER SETTINGS ================= */

#define CAN_BITRATE_KBPS 500UL   /* Try 500 first. Other useful values: 250, 125. */
#define CAN_PCLK1_HZ     48000000UL

/* Set to 1 to accept only the Tesla CAN IDs decoded by this dashboard.
   Set to 0 to accept all CAN frames for debugging. */
#define TESLA_CAN_FILTERING 1

/* Display orientation.
   0x48 is common portrait/top-left.
   Try 0x28, 0x88, or 0xE8 if text is mirrored/rotated. */
#define LCD_MADCTL       0xC8

/* Backlight polarity.
   Set to 0 if your PA15 backlight driver is active-low. */
#define LCD_BACKLIGHT_ACTIVE_HIGH 1

/* ================= LOW LEVEL REGISTERS ================= */

#define REG32(addr) (*(volatile uint32_t *)(uintptr_t)(addr))
#define BIT(n)      (1UL << (n))

#define PERIPH_BASE       0x40000000UL
#define APB1_BASE         (PERIPH_BASE + 0x00000UL)
#define APB2_BASE         (PERIPH_BASE + 0x10000UL)
#define AHB_BASE          (PERIPH_BASE + 0x20000UL)

#define CRM_BASE          (AHB_BASE  + 0x1000UL)
#define FLASH_REG_BASE    (AHB_BASE  + 0x2000UL)
#define IOMUX_BASE        (APB2_BASE + 0x0000UL)
#define GPIOA_BASE        (APB2_BASE + 0x0800UL)
#define GPIOB_BASE        (APB2_BASE + 0x0C00UL)
#define GPIOC_BASE        (APB2_BASE + 0x1000UL)
#define CAN1_BASE         (APB1_BASE + 0x6400UL)

#define CRM_CTRL          REG32(CRM_BASE + 0x00UL)
#define CRM_CFG           REG32(CRM_BASE + 0x04UL)
#define CRM_AHBEN         REG32(CRM_BASE + 0x14UL)
#define CRM_APB2EN        REG32(CRM_BASE + 0x18UL)
#define CRM_APB1EN        REG32(CRM_BASE + 0x1CUL)
#define CRM_APB1RST       REG32(CRM_BASE + 0x10UL)
#define CRM_MISC1         REG32(CRM_BASE + 0x30UL)
#define CRM_MISC2         REG32(CRM_BASE + 0x54UL)
#define FLASH_PSR         REG32(FLASH_REG_BASE + 0x00UL)

#define IOMUX_REMAP       REG32(IOMUX_BASE + 0x04UL)
#define IOMUX_REMAP6      REG32(IOMUX_BASE + 0x2CUL)
#define IOMUX_REMAP7      REG32(IOMUX_BASE + 0x30UL)

#define GPIO_CFGLR(base)  REG32((base) + 0x00UL)
#define GPIO_CFGHR(base)  REG32((base) + 0x04UL)
#define GPIO_IDT(base)    REG32((base) + 0x08UL)
#define GPIO_ODT(base)    REG32((base) + 0x0CUL)
#define GPIO_SCR(base)    REG32((base) + 0x10UL)
#define GPIO_CLR(base)    REG32((base) + 0x14UL)

/* CAN registers */
#define CAN_MCTRL         REG32(CAN1_BASE + 0x000UL)
#define CAN_MSTS          REG32(CAN1_BASE + 0x004UL)
#define CAN_TSTS          REG32(CAN1_BASE + 0x008UL)
#define CAN_RF0           REG32(CAN1_BASE + 0x00CUL)
#define CAN_INTEN         REG32(CAN1_BASE + 0x014UL)
#define CAN_ESTS          REG32(CAN1_BASE + 0x018UL)
#define CAN_BTMG          REG32(CAN1_BASE + 0x01CUL)

#define CAN_FIFO0_RFI     REG32(CAN1_BASE + 0x1B0UL)
#define CAN_FIFO0_RFC     REG32(CAN1_BASE + 0x1B4UL)
#define CAN_FIFO0_RDTL    REG32(CAN1_BASE + 0x1B8UL)
#define CAN_FIFO0_RDTH    REG32(CAN1_BASE + 0x1BCUL)

#define CAN_FCTRL         REG32(CAN1_BASE + 0x200UL)
#define CAN_FMCFG         REG32(CAN1_BASE + 0x204UL)
#define CAN_FBWCFG        REG32(CAN1_BASE + 0x20CUL)
#define CAN_FRF           REG32(CAN1_BASE + 0x214UL)
#define CAN_FACFG         REG32(CAN1_BASE + 0x21CUL)
#define CAN_FFB1(n)       REG32(CAN1_BASE + 0x240UL + ((uint32_t)(n) * 8UL))
#define CAN_FFB2(n)       REG32(CAN1_BASE + 0x244UL + ((uint32_t)(n) * 8UL))

/* CAN bit definitions */
#define CAN_MCTRL_FZEN    BIT(0)
#define CAN_MCTRL_DZEN    BIT(1)
#define CAN_MCTRL_MDRSEL  BIT(3)
#define CAN_MCTRL_AEDEN   BIT(5)
#define CAN_MCTRL_AEBOEN  BIT(6)
#define CAN_MSTS_FZC      BIT(0)
#define CAN_BTMG_LBEN     BIT(30)
#define CAN_BTMG_LOEN     BIT(31)
#define CAN_RF0_RELEASE   BIT(5)

/* ================= LCD PIN MAP ================= */

#define LCD_BL_PIN        PA15
#define LCD_RESET_PIN     PB4
#define LCD_WR_PIN        PC13
#define LCD_RS_PIN        PC15

/* Direct GPIO bits for speed */
#define LCD_WR_BIT        13
#define LCD_RS_BIT        15
#define LCD_RESET_BIT     4
#define LCD_BL_BIT        15

/* RGB565 colors */
#define C_BLACK           0x0000
#define C_WHITE           0xFFFF
#define C_RED             0xF800
#define C_GREEN           0x07E0
#define C_BLUE            0x001F
#define C_YELLOW          0xFFE0
#define C_CYAN            0x07FF
#define C_DARK            0x2104

/* ================= SMALL FONT ================= */

static void glyph5x7(char c, uint8_t out[5]) {
  uint8_t a=0,b=0,d=0,e=0,f=0;
  switch (c) {
    case '0': a=0x3E; b=0x51; d=0x49; e=0x45; f=0x3E; break;
    case '1': a=0x00; b=0x42; d=0x7F; e=0x40; f=0x00; break;
    case '2': a=0x42; b=0x61; d=0x51; e=0x49; f=0x46; break;
    case '3': a=0x21; b=0x41; d=0x45; e=0x4B; f=0x31; break;
    case '4': a=0x18; b=0x14; d=0x12; e=0x7F; f=0x10; break;
    case '5': a=0x27; b=0x45; d=0x45; e=0x45; f=0x39; break;
    case '6': a=0x3C; b=0x4A; d=0x49; e=0x49; f=0x30; break;
    case '7': a=0x01; b=0x71; d=0x09; e=0x05; f=0x03; break;
    case '8': a=0x36; b=0x49; d=0x49; e=0x49; f=0x36; break;
    case '9': a=0x06; b=0x49; d=0x49; e=0x29; f=0x1E; break;

    case 'A': a=0x7E; b=0x11; d=0x11; e=0x11; f=0x7E; break;
    case 'B': a=0x7F; b=0x49; d=0x49; e=0x49; f=0x36; break;
    case 'C': a=0x3E; b=0x41; d=0x41; e=0x41; f=0x22; break;
    case 'D': a=0x7F; b=0x41; d=0x41; e=0x22; f=0x1C; break;
    case 'E': a=0x7F; b=0x49; d=0x49; e=0x49; f=0x41; break;
    case 'F': a=0x7F; b=0x09; d=0x09; e=0x09; f=0x01; break;
    case 'G': a=0x3E; b=0x41; d=0x49; e=0x49; f=0x7A; break;
    case 'H': a=0x7F; b=0x08; d=0x08; e=0x08; f=0x7F; break;
    case 'I': a=0x00; b=0x41; d=0x7F; e=0x41; f=0x00; break;
    case 'J': a=0x20; b=0x40; d=0x41; e=0x3F; f=0x01; break;
    case 'K': a=0x7F; b=0x08; d=0x14; e=0x22; f=0x41; break;
    case 'L': a=0x7F; b=0x40; d=0x40; e=0x40; f=0x40; break;
    case 'M': a=0x7F; b=0x02; d=0x0C; e=0x02; f=0x7F; break;
    case 'N': a=0x7F; b=0x04; d=0x08; e=0x10; f=0x7F; break;
    case 'O': a=0x3E; b=0x41; d=0x41; e=0x41; f=0x3E; break;
    case 'P': a=0x7F; b=0x09; d=0x09; e=0x09; f=0x06; break;
    case 'Q': a=0x3E; b=0x41; d=0x51; e=0x21; f=0x5E; break;
    case 'R': a=0x7F; b=0x09; d=0x19; e=0x29; f=0x46; break;
    case 'S': a=0x46; b=0x49; d=0x49; e=0x49; f=0x31; break;
    case 'T': a=0x01; b=0x01; d=0x7F; e=0x01; f=0x01; break;
    case 'U': a=0x3F; b=0x40; d=0x40; e=0x40; f=0x3F; break;
    case 'V': a=0x1F; b=0x20; d=0x40; e=0x20; f=0x1F; break;
    case 'W': a=0x3F; b=0x40; d=0x38; e=0x40; f=0x3F; break;
    case 'X': a=0x63; b=0x14; d=0x08; e=0x14; f=0x63; break;
    case 'Y': a=0x07; b=0x08; d=0x70; e=0x08; f=0x07; break;
    case 'Z': a=0x61; b=0x51; d=0x49; e=0x45; f=0x43; break;

    case ':': a=0x00; b=0x36; d=0x36; e=0x00; f=0x00; break;
    case '-': a=0x08; b=0x08; d=0x08; e=0x08; f=0x08; break;
    case '.': a=0x00; b=0x60; d=0x60; e=0x00; f=0x00; break;
    case '%': a=0x63; b=0x13; d=0x08; e=0x64; f=0x63; break;
    case '/': a=0x20; b=0x10; d=0x08; e=0x04; f=0x02; break;
    case ' ': default: a=0; b=0; d=0; e=0; f=0; break;
  }
  out[0]=a; out[1]=b; out[2]=d; out[3]=e; out[4]=f;
}

/* ================= FAST CLOCK SETUP ================= */

static void clock_48mhz_begin(void) {
  /*
    AT32F415 reset default is HICK 48 MHz divided by 6 = 8 MHz.
    The Arduino core build uses F_CPU=48 MHz, so switch HICK to no-divider
    early so delay(), millis timing, LCD writes, and CAN timing match.

    Relevant AT32F415 CRM fields:
      CRM_MISC1 bit25 = hickdiv, 1 = HICK 48 MHz no divide
      CRM_MISC2 bit9  = hick_to_sclk, 1 = HICK-to-SCLK can use 48 MHz
      CRM_CFG bits 0..1 select SCLK source, 0 = HICK
      CRM_CFG APB/AHB dividers set to /1 so APB1 = APB2 = AHB = 48 MHz

    Flash wait state:
      33-64 MHz requires FLASH_WAIT_CYCLE_1. 0x10 keeps prefetch enabled.
  */

  CRM_CTRL |= BIT(0);                         /* HICK enable */
  while ((CRM_CTRL & BIT(1)) == 0) { }        /* wait HICK stable */

  FLASH_PSR = 0x10UL | 0x01UL;                /* prefetch + 1 wait cycle */

  CRM_CFG &= ~((0x3UL << 0) |                 /* SCLK = HICK */
               (0xFUL << 4) |                 /* AHB /1 */
               (0x7UL << 8) |                 /* APB1 /1 */
               (0x7UL << 11));                /* APB2 /1 */

  CRM_MISC1 |= BIT(25);                       /* HICK no divide */
  CRM_MISC2 |= BIT(9);                        /* HICK 48 MHz allowed as SCLK */
}


/* ================= GPIO HELPERS ================= */

static void gpio_config_nibble(uint32_t base, uint8_t bit, uint8_t cfg) {
  volatile uint32_t *reg = (bit < 8) ? &GPIO_CFGLR(base) : &GPIO_CFGHR(base);
  uint8_t shift = (bit & 7) * 4;
  uint32_t v = *reg;
  v &= ~(0xFUL << shift);
  v |= ((uint32_t)(cfg & 0x0F) << shift);
  *reg = v;
}

static void release_jtag_keep_swd(void) {
  /* JTAG off, SWD still enabled. Releases PA15/PB3/PB4. */
  IOMUX_REMAP  = (IOMUX_REMAP  & ~(7UL << 24)) | (2UL << 24);
  IOMUX_REMAP7 = (IOMUX_REMAP7 & ~(7UL << 16)) | (2UL << 16);
}

static void gpio_basic_clock_on(void) {
  CRM_APB2EN |= BIT(0) | BIT(2) | BIT(3) | BIT(4);  /* IOMUX, GPIOA/B/C */
  (void)CRM_APB2EN;
  release_jtag_keep_swd();
}

static void gpio_output_pp_50mhz(uint32_t base, uint8_t bit) {
  gpio_config_nibble(base, bit, 0x3);  /* output push-pull, 50 MHz */
}

static void gpio_mux_pp_50mhz(uint32_t base, uint8_t bit) {
  gpio_config_nibble(base, bit, 0xB);  /* alternate-function push-pull, 50 MHz */
}

static void gpio_input_pullup(uint32_t base, uint8_t bit) {
  gpio_config_nibble(base, bit, 0x8);  /* input pull up/down */
  GPIO_SCR(base) = BIT(bit);           /* pull-up */
}

/* ================= LCD DRIVER ================= */

static inline void lcd_wr_low(void)  { GPIO_CLR(GPIOC_BASE) = BIT(LCD_WR_BIT); }
static inline void lcd_wr_high(void) { GPIO_SCR(GPIOC_BASE) = BIT(LCD_WR_BIT); }
static inline void lcd_rs_low(void)  { GPIO_CLR(GPIOC_BASE) = BIT(LCD_RS_BIT); }
static inline void lcd_rs_high(void) { GPIO_SCR(GPIOC_BASE) = BIT(LCD_RS_BIT); }

static inline void lcd_write8(uint8_t v) {
  GPIO_ODT(GPIOA_BASE) = (GPIO_ODT(GPIOA_BASE) & ~0x00FFUL) | v;
  lcd_wr_low();
  __asm__ volatile ("nop\nnop\nnop");
  lcd_wr_high();
}

static void lcd_cmd(uint8_t c) {
  lcd_rs_low();
  lcd_write8(c);
}

static void lcd_data(uint8_t d) {
  lcd_rs_high();
  lcd_write8(d);
}

static void lcd_data16(uint16_t v) {
  lcd_data((uint8_t)(v >> 8));
  lcd_data((uint8_t)(v & 0xFF));
}

static void lcd_set_window(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1) {
  lcd_cmd(0x2A);
  lcd_data16(x0);
  lcd_data16(x1);

  lcd_cmd(0x2B);
  lcd_data16(y0);
  lcd_data16(y1);

  lcd_cmd(0x2C);
  lcd_rs_high();
}

static inline void lcd_write_color(uint16_t color) {
  lcd_write8((uint8_t)(color >> 8));
  lcd_write8((uint8_t)(color & 0xFF));
}

static void lcd_fill_screen(uint16_t color) {
  lcd_set_window(0, 0, 319, 479);
  uint32_t count = 320UL * 480UL;
  while (count--) {
    lcd_write_color(color);
  }
}

static void lcd_fill_rect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color) {
  if (x >= 320 || y >= 480) return;
  if ((uint32_t)x + w > 320) w = 320 - x;
  if ((uint32_t)y + h > 480) h = 480 - y;

  lcd_set_window(x, y, x + w - 1, y + h - 1);
  uint32_t count = (uint32_t)w * (uint32_t)h;
  while (count--) {
    lcd_write_color(color);
  }
}

/*
  Fast text renderer:
    Old method opened a new LCD window for every scaled font pixel.
    This method opens one LCD window per character and streams pixels.
*/
static void lcd_draw_char(uint16_t x, uint16_t y, char c, uint16_t fg, uint16_t bg, uint8_t scale) {
  uint8_t g[5];
  uint8_t w = (uint8_t)(6U * scale);
  uint8_t h = (uint8_t)(8U * scale);

  if (x >= 320 || y >= 480) return;
  if ((uint32_t)x + w > 320) w = (uint8_t)(320 - x);
  if ((uint32_t)y + h > 480) h = (uint8_t)(480 - y);

  glyph5x7(c, g);
  lcd_set_window(x, y, x + w - 1, y + h - 1);

  for (uint8_t py = 0; py < h; py++) {
    uint8_t row = py / scale;

    for (uint8_t px = 0; px < w; px++) {
      uint8_t col = px / scale;
      uint16_t color = bg;

      if (col < 5 && (g[col] & (1U << row))) {
        color = fg;
      }

      lcd_write_color(color);
    }
  }
}

static void lcd_draw_text(uint16_t x, uint16_t y, const char *s, uint16_t fg, uint16_t bg, uint8_t scale) {
  uint8_t step = (uint8_t)(6U * scale);

  while (*s) {
    if (x >= 320) return;
    lcd_draw_char(x, y, *s, fg, bg, scale);
    x += step;
    s++;
  }
}

/*
  Text box renderer:
    Opens one LCD window for the entire value field.
    It draws text plus background in one pass, replacing:
      lcd_fill_rect(...)
      lcd_draw_text(...)
    This is faster and avoids flicker/tearing on changing values.
*/
static void lcd_draw_text_box(uint16_t x, uint16_t y, uint16_t w, uint16_t h,
                              const char *s, uint16_t fg, uint16_t bg, uint8_t scale) {
  if (x >= 320 || y >= 480) return;
  if ((uint32_t)x + w > 320) w = 320 - x;
  if ((uint32_t)y + h > 480) h = 480 - y;

  uint8_t char_w = (uint8_t)(6U * scale);
  uint8_t char_h = (uint8_t)(8U * scale);

  lcd_set_window(x, y, x + w - 1, y + h - 1);

  for (uint16_t py = 0; py < h; py++) {
    if (py >= char_h) {
      for (uint16_t px = 0; px < w; px++) {
        lcd_write_color(bg);
      }
      continue;
    }

    uint8_t font_row = (uint8_t)(py / scale);
    uint16_t written = 0;
    uint8_t char_index = 0;

    while (written < w) {
      char c = s[char_index];

      if (c == 0) {
        while (written < w) {
          lcd_write_color(bg);
          written++;
        }
        break;
      }

      uint8_t g[5];
      glyph5x7(c, g);

      for (uint8_t px = 0; px < char_w && written < w; px++) {
        uint8_t font_col = (uint8_t)(px / scale);
        uint16_t color = bg;

        if (font_col < 5 && (g[font_col] & (1U << font_row))) {
          color = fg;
        }

        lcd_write_color(color);
        written++;
      }

      char_index++;
    }
  }
}

static void lcd_begin(void) {
  gpio_basic_clock_on();

  /* Data PA0..PA7 */
  for (uint8_t i = 0; i < 8; i++) {
    gpio_output_pp_50mhz(GPIOA_BASE, i);
  }

  /* Control pins */
  gpio_output_pp_50mhz(GPIOC_BASE, LCD_WR_BIT);
  gpio_output_pp_50mhz(GPIOC_BASE, LCD_RS_BIT);
  gpio_output_pp_50mhz(GPIOB_BASE, LCD_RESET_BIT);
  gpio_output_pp_50mhz(GPIOA_BASE, LCD_BL_BIT);

  lcd_wr_high();
  lcd_rs_high();

#if LCD_BACKLIGHT_ACTIVE_HIGH
  GPIO_SCR(GPIOA_BASE) = BIT(LCD_BL_BIT);
#else
  GPIO_CLR(GPIOA_BASE) = BIT(LCD_BL_BIT);
#endif

  /* Hardware reset */
  GPIO_CLR(GPIOB_BASE) = BIT(LCD_RESET_BIT);
  delay(30);
  GPIO_SCR(GPIOB_BASE) = BIT(LCD_RESET_BIT);
  delay(150);

  /* ILI9486/ILI9488/ST7796-compatible init */
  lcd_cmd(0x01); delay(150);             /* software reset */
  lcd_cmd(0x11); delay(150);             /* sleep out */

  lcd_cmd(0x3A); lcd_data(0x55);         /* RGB565, 16 bits/pixel */
  lcd_cmd(0x36); lcd_data(LCD_MADCTL);   /* orientation */

  /* Common power/gamma setup. These are safe defaults for many ILI9486-type panels. */
  lcd_cmd(0xC0); lcd_data(0x0D); lcd_data(0x0D);
  lcd_cmd(0xC1); lcd_data(0x43); lcd_data(0x00);
  lcd_cmd(0xC2); lcd_data(0x00);
  lcd_cmd(0xC5); lcd_data(0x00); lcd_data(0x48);

  lcd_cmd(0xB6); lcd_data(0x00); lcd_data(0x22); lcd_data(0x3B);

  lcd_cmd(0xE0);
  lcd_data(0x0F); lcd_data(0x24); lcd_data(0x1C); lcd_data(0x0A);
  lcd_data(0x0F); lcd_data(0x08); lcd_data(0x43); lcd_data(0x88);
  lcd_data(0x32); lcd_data(0x0F); lcd_data(0x10); lcd_data(0x06);
  lcd_data(0x0F); lcd_data(0x07); lcd_data(0x00);

  lcd_cmd(0xE1);
  lcd_data(0x0F); lcd_data(0x38); lcd_data(0x30); lcd_data(0x09);
  lcd_data(0x0F); lcd_data(0x0F); lcd_data(0x4E); lcd_data(0x77);
  lcd_data(0x3C); lcd_data(0x07); lcd_data(0x10); lcd_data(0x05);
  lcd_data(0x23); lcd_data(0x1B); lcd_data(0x00);

  lcd_cmd(0x29); delay(30);              /* display on */
}

/* ================= CAN READ-ONLY DRIVER ================= */

struct CanFrame {
  uint32_t id;
  uint8_t extended;
  uint8_t remote;
  uint8_t dlc;
  uint8_t data[8];
  uint8_t filter;
};

static uint32_t can_rx_count = 0;
static uint32_t can_overrun_count = 0;
static uint8_t can_ok = 0;
static uint8_t can_fail_stage = 0;
static uint32_t can_dbg_mctrl = 0;
static uint32_t can_dbg_msts  = 0;
static uint32_t can_dbg_btmg  = 0;
static uint32_t can_dbg_apb1  = 0;
static uint8_t can_raw_rx_level = 0;
static uint8_t have_frame = 0;
static CanFrame last_frame;

/* ================= TESLA DECODED VALUES ================= */

struct TeslaData {
  uint32_t rx_total;

  uint8_t have_0102;
  uint16_t pack_voltage_cV;      /* 0.01 V */
  int16_t pack_current_dA;       /* 0.1 A, no-offset decode */

  uint8_t have_0256;
  uint8_t speed_display;         /* byte6, MPH or KPH per car setting */
  uint16_t speed_bar_dmph;       /* 0.1 MPH from 12-bit analog-speed value */
  uint8_t speed_unit_kph;

  uint8_t have_0232;
  uint16_t max_regen_cKW;        /* 0.01 kW */
  uint16_t max_discharge_cKW;    /* 0.01 kW */

  uint8_t have_0302;
  uint16_t soc_min_dP;           /* 0.1 % */
  uint16_t soc_ui_dP;            /* 0.1 % */

  uint8_t have_0382;
  uint16_t nominal_full_dkWh;    /* 0.1 kWh */
  uint16_t nominal_remain_dkWh;  /* 0.1 kWh */
  uint16_t expected_remain_dkWh; /* 0.1 kWh */
  uint16_t ideal_remain_dkWh;    /* 0.1 kWh */
  uint16_t energy_buffer_dkWh;   /* 0.1 kWh */

  uint8_t have_0106;
  int16_t motor_rpm;
  uint16_t pedal_dP;             /* 0.1 % */

  uint8_t have_0116;
  int16_t torque_est_Nm;
  int16_t rear_speed_cmph;       /* 0.01 MPH */
  uint8_t gear;
};

static TeslaData tesla;
static uint8_t tesla_dirty = 1;


static uint32_t can_std_filter_word(uint16_t std_id) {
  /*
    32-bit filter word layout for a standard 11-bit ID:
      STDID bits go to bits 31:21
      IDE = 0 for standard ID
      RTR = 0 for data frame
  */
  return ((uint32_t)(std_id & 0x7FFU) << 21);
}

static void can_accept_all_filter_fifo0(void) {
  /* Filter 0, 32-bit mask mode, ID=0, MASK=0 accepts everything. */
  CAN_FCTRL |= BIT(0);       /* filter config mode */

  CAN_FACFG = 0x00000000UL;  /* disable all filters while editing */
  CAN_FBWCFG |= BIT(0);      /* filter 0 = 32-bit */
  CAN_FMCFG &= ~BIT(0);      /* filter 0 = ID-mask mode */
  CAN_FRF &= ~BIT(0);        /* filter 0 -> FIFO0 */
  CAN_FFB1(0) = 0x00000000;  /* ID */
  CAN_FFB2(0) = 0x00000000;  /* MASK */
  CAN_FACFG |= BIT(0);       /* enable filter 0 */

  CAN_FCTRL &= ~BIT(0);      /* leave filter config mode */
}

static void can_accept_tesla_dashboard_filters_fifo0(void) {
#if TESLA_CAN_FILTERING
  /*
    Use 32-bit identifier-list filters.
    Each filter bank accepts two exact standard IDs.
    FIFO0 receives only the frames decoded by the dashboard:

      0x0102 BMS voltage/current
      0x0256 speed/status
      0x0232 battery power limits
      0x0302 battery SOC
      0x0382 battery energy status
      0x0106 rear drive unit info
      0x0116 rear drive unit torque/status

    The 8th slot repeats 0x0116 so no extra ID is opened.
  */
  CAN_FCTRL |= BIT(0);       /* filter config mode */

  /* Disable all filters while editing. */
  CAN_FACFG = 0x00000000UL;

  /*
    Banks 0..3:
      32-bit scale = FBWCFG bit = 1
      identifier-list mode = FMCFG bit = 1
      FIFO0 assignment = FRF bit = 0
  */
  CAN_FBWCFG |=  (BIT(0) | BIT(1) | BIT(2) | BIT(3));
  CAN_FMCFG  |=  (BIT(0) | BIT(1) | BIT(2) | BIT(3));
  CAN_FRF    &= ~(BIT(0) | BIT(1) | BIT(2) | BIT(3));

  CAN_FFB1(0) = can_std_filter_word(0x0102);
  CAN_FFB2(0) = can_std_filter_word(0x0256);

  CAN_FFB1(1) = can_std_filter_word(0x0232);
  CAN_FFB2(1) = can_std_filter_word(0x0302);

  CAN_FFB1(2) = can_std_filter_word(0x0382);
  CAN_FFB2(2) = can_std_filter_word(0x0106);

  CAN_FFB1(3) = can_std_filter_word(0x0116);
  CAN_FFB2(3) = can_std_filter_word(0x0116);

  /* Enable only banks 0..3. */
  CAN_FACFG = (BIT(0) | BIT(1) | BIT(2) | BIT(3));

  CAN_FCTRL &= ~BIT(0);      /* leave filter config mode */
#else
  can_accept_tesla_dashboard_filters_fifo0();
#endif
}

static uint8_t can_choose_timing(uint32_t kbps, uint16_t *brdiv, uint8_t *bts1, uint8_t *bts2, uint8_t *rsaw) {
  /*
    Timing assumes PCLK1 = 48 MHz after clock_48mhz_begin().
    16 time quanta total: 1 sync + 12 BS1 + 3 BS2.
    CAN bitrate = PCLK1 / (BRDIV * 16).

    Artery register stores enum values:
      bts1 enum 11 means 12TQ
      bts2 enum 2 means 3TQ
      rsaw enum 2 means 3TQ
  */
  *bts1 = 11;
  *bts2 = 2;
  *rsaw = 2;

  if (kbps == 500) { *brdiv = 6;  return 1; }  /* 48 MHz / 6 / 16 = 500 kbps */
  if (kbps == 250) { *brdiv = 12; return 1; }  /* 48 MHz / 12 / 16 = 250 kbps */
  if (kbps == 125) { *brdiv = 24; return 1; }  /* 48 MHz / 24 / 16 = 125 kbps */

  /* fallback to 500 kbps */
  *brdiv = 6;
  return 0;
}

static uint8_t can_begin_read_only(uint32_t kbps) {
  uint16_t brdiv;
  uint8_t bts1, bts2, rsaw;

  can_fail_stage = 0;
  can_dbg_mctrl = 0;
  can_dbg_msts = 0;
  can_dbg_btmg = 0;
  can_dbg_apb1 = 0;
  can_raw_rx_level = 0;

  gpio_basic_clock_on();

  /*
    CAN1 default pins, no remap:
      CAN RXD = PA11, physical pin 32
      CAN TXD = PA12, physical pin 33

    PA12/TXD is deliberately NOT configured as alternate-function output in this
    read-only sketch. It is held as input-pullup/recessive so the external CAN
    transceiver TXD input stays high. The CAN controller is also put in hardware
    LISTEN-ONLY mode and no transmit mailbox is ever loaded.
  */
  IOMUX_REMAP6 = (IOMUX_REMAP6 & ~0x0FUL);   /* CAN1 default PA11/PA12 pins */

  gpio_input_pullup(GPIOA_BASE, 11);    /* PA11 CAN RXD */
  gpio_input_pullup(GPIOA_BASE, 12);    /* PA12 CAN TXD safe/recessive */

  /* Show whether PA11/RXD is high/recessive before CAN takes over. */
  can_raw_rx_level = (GPIO_IDT(GPIOA_BASE) & BIT(11)) ? 1 : 0;

  if (!can_choose_timing(kbps, &brdiv, &bts1, &bts2, &rsaw)) {
    can_fail_stage = 1;                 /* unsupported bitrate */
    return 0;
  }

  /* Enable CAN1 clock */
  CRM_APB1EN |= BIT(25);
  (void)CRM_APB1EN;
  can_dbg_apb1 = CRM_APB1EN;
  if ((CRM_APB1EN & BIT(25)) == 0) {
    can_fail_stage = 2;                 /* CAN clock bit did not stick */
    return 0;
  }

  /* Reset CAN1 peripheral */
  CRM_APB1RST |= BIT(25);
  for (volatile uint32_t i = 0; i < 2000; i++) { __asm__ volatile ("nop"); }
  CRM_APB1RST &= ~BIT(25);
  for (volatile uint32_t i = 0; i < 2000; i++) { __asm__ volatile ("nop"); }

  /*
    Enter freeze/config mode using the exact Artery driver sequence:
      DZEN = 0
      FZEN = 1
      wait MSTS.FZC = 1
  */
  CAN_MCTRL &= ~CAN_MCTRL_DZEN;
  CAN_MCTRL |= CAN_MCTRL_FZEN;

  uint32_t timeout = 0x00FFFFFFUL;
  while (((CAN_MSTS & CAN_MSTS_FZC) == 0) && timeout--) { }

  can_dbg_mctrl = CAN_MCTRL;
  can_dbg_msts  = CAN_MSTS;

  if ((CAN_MSTS & CAN_MSTS_FZC) == 0) {
    can_fail_stage = 3;                 /* failed to enter freeze */
    return 0;
  }

  /*
    Use default Artery base settings:
      TTCEN=0, AEBOEN=0, AEDEN=0, PRSFEN=0, MDRSEL=0, MMSSR=0.
    Earlier v3 enabled a few extras; remove them for clean bring-up.
  */
  CAN_MCTRL &= ~(BIT(2) | BIT(3) | BIT(4) | BIT(5) | BIT(6) | BIT(7));

  /*
    BTMG:
      brdiv: bits 0..11 stores divider-1
      bts1 : bits 16..19
      bts2 : bits 20..22
      rsaw : bits 24..25
      LOEN : bit 31 listen-only enable
      LBEN : bit 30 loopback disabled
  */
  CAN_BTMG =
    ((uint32_t)(brdiv - 1) & 0x0FFFUL) |
    ((uint32_t)bts1 << 16) |
    ((uint32_t)bts2 << 20) |
    ((uint32_t)rsaw << 24) |
    CAN_BTMG_LOEN;

  can_dbg_btmg = CAN_BTMG;

  /*
    Set accept-all filter while still in freeze/config.
  */
  can_accept_tesla_dashboard_filters_fifo0();

  /* No interrupts: polling only. */
  CAN_INTEN = 0;

  /*
    Leaving freeze requires the CAN RX input to see an idle/recessive level.
    If PA11/RXD is low here, the controller may stay in freeze forever.
  */
  can_raw_rx_level = (GPIO_IDT(GPIOA_BASE) & BIT(11)) ? 1 : 0;

  CAN_MCTRL &= ~CAN_MCTRL_FZEN;

  timeout = 0x01FFFFFFUL;
  while ((CAN_MSTS & CAN_MSTS_FZC) && timeout--) {
    /* keep sampling raw RX while waiting */
    can_raw_rx_level = (GPIO_IDT(GPIOA_BASE) & BIT(11)) ? 1 : 0;
  }

  can_dbg_mctrl = CAN_MCTRL;
  can_dbg_msts  = CAN_MSTS;

  if (CAN_MSTS & CAN_MSTS_FZC) {
    if (can_raw_rx_level == 0) {
      can_fail_stage = 5;               /* RXD/PA11 stuck low/dominant */
    } else {
      can_fail_stage = 4;               /* failed to leave freeze while RX high */
    }
    return 0;
  }

  can_fail_stage = 0;
  return 1;
}

static uint8_t can_available(void) {
  return (uint8_t)(CAN_RF0 & 0x03UL);
}

static uint8_t can_read_latest(void) {
  if (!can_available()) {
    return 0;
  }

  uint32_t rfi  = CAN_FIFO0_RFI;
  uint32_t rfc  = CAN_FIFO0_RFC;
  uint32_t rdtl = CAN_FIFO0_RDTL;
  uint32_t rdth = CAN_FIFO0_RDTH;

  last_frame.extended = (rfi & BIT(2)) ? 1 : 0;
  last_frame.remote   = (rfi & BIT(1)) ? 1 : 0;

  if (last_frame.extended) {
    last_frame.id = (rfi >> 3) & 0x1FFFFFFFUL;
  } else {
    last_frame.id = (rfi >> 21) & 0x7FFUL;
  }

  last_frame.dlc = (uint8_t)(rfc & 0x0FUL);
  if (last_frame.dlc > 8) last_frame.dlc = 8;
  last_frame.filter = (uint8_t)((rfc >> 8) & 0xFF);

  last_frame.data[0] = (uint8_t)(rdtl >> 0);
  last_frame.data[1] = (uint8_t)(rdtl >> 8);
  last_frame.data[2] = (uint8_t)(rdtl >> 16);
  last_frame.data[3] = (uint8_t)(rdtl >> 24);
  last_frame.data[4] = (uint8_t)(rdth >> 0);
  last_frame.data[5] = (uint8_t)(rdth >> 8);
  last_frame.data[6] = (uint8_t)(rdth >> 16);
  last_frame.data[7] = (uint8_t)(rdth >> 24);

  if (CAN_RF0 & BIT(4)) {
    can_overrun_count++;
  }

  CAN_RF0 = CAN_RF0_RELEASE;  /* release FIFO0 */

  have_frame = 1;
  can_rx_count++;
  return 1;
}

/* ================= TESLA DECODE HELPERS ================= */

static int16_t sign_extend_12(uint16_t v) {
  v &= 0x0FFF;
  if (v & 0x0800) return (int16_t)(v | 0xF000);
  return (int16_t)v;
}

static int16_t sign_extend_15(uint16_t v) {
  v &= 0x7FFF;
  if (v & 0x4000) return (int16_t)(v | 0x8000);
  return (int16_t)v;
}

static const char *gear_name(uint8_t g) {
  switch (g) {
    case 1: return "P";
    case 2: return "R";
    case 3: return "N";
    case 4: return "D";
    default: return "-";
  }
}

static void tesla_decode_frame(void) {
  uint8_t *b = last_frame.data;
  uint8_t dlc = last_frame.dlc;
  uint32_t id = last_frame.id;

  tesla.rx_total++;
  tesla_dirty = 1;

  if (id == 0x0102 && dlc >= 6) {
    /* BMS Current and Voltage:
       voltage = bytes 1/0 unsigned, 0.01 V.
       current = bytes 3/2 signed 15-bit, 0.1 A, no-offset variant. */
    tesla.pack_voltage_cV = (uint16_t)b[0] | ((uint16_t)b[1] << 8);
    tesla.pack_current_dA = sign_extend_15((uint16_t)b[2] | ((uint16_t)b[3] << 8));
    tesla.have_0102 = 1;
  }
  else if (id == 0x0256 && dlc >= 8) {
    /* Speed and cruise/status:
       byte6 numeric speed, byte3 bit7 selects KPH.
       bytes 2 and low nibble byte3 are 12-bit analog speed in 0.1 MPH. */
    uint16_t raw = (uint16_t)b[2] | (((uint16_t)b[3] & 0x0F) << 8);
    tesla.speed_bar_dmph = raw;
    tesla.speed_display = b[6];
    tesla.speed_unit_kph = (b[3] & 0x80) ? 1 : 0;
    tesla.have_0256 = 1;
  }
  else if (id == 0x0232 && dlc >= 4) {
    /* Battery power limits:
       maxRegenPower = bytes 1/0 / 100 kW.
       maxDischargePower = bytes 3/2 / 100 kW. */
    tesla.max_regen_cKW = (uint16_t)b[0] | ((uint16_t)b[1] << 8);
    tesla.max_discharge_cKW = (uint16_t)b[2] | ((uint16_t)b[3] << 8);
    tesla.have_0232 = 1;
  }
  else if (id == 0x0302 && dlc >= 3) {
    /* Battery SoC:
       socMin = (byte0 + ((byte1 & 0x03) << 8)) / 10.
       socUI = ((byte1 >> 2) + ((byte2 & 0x0F) << 6)) / 10. */
    tesla.soc_min_dP = (uint16_t)b[0] + (((uint16_t)b[1] & 0x03) << 8);
    tesla.soc_ui_dP = ((uint16_t)b[1] >> 2) + (((uint16_t)b[2] & 0x0F) << 6);
    tesla.have_0302 = 1;
  }
  else if (id == 0x0382 && dlc >= 8) {
    /* Battery Energy Status, all values 0.1 kWh. */
    tesla.nominal_full_dkWh =
      ((uint16_t)b[0] + (((uint16_t)b[1] & 0x03) << 8));
    tesla.nominal_remain_dkWh =
      (((uint16_t)b[1] >> 2) + (((uint16_t)b[2] & 0x0F) << 6));
    tesla.expected_remain_dkWh =
      (((uint16_t)b[2] >> 4) + (((uint16_t)b[3] & 0x3F) << 4));
    tesla.ideal_remain_dkWh =
      (((uint16_t)b[3] >> 6) + ((uint16_t)b[4] << 2));
    tesla.energy_buffer_dkWh =
      (((uint16_t)b[6] >> 2) + (((uint16_t)b[7] & 0x03) << 6));
    tesla.have_0382 = 1;
  }
  else if (id == 0x0106 && dlc >= 7) {
    /* Rear drive unit info:
       motorRPM = byte4 + byte5<<8, signed 16-bit.
       pedalPos = byte6 * 0.4%. Store as 0.1% = byte6 * 4. */
    tesla.motor_rpm = (int16_t)((uint16_t)b[4] | ((uint16_t)b[5] << 8));
    tesla.pedal_dP = (uint16_t)b[6] * 4;
    tesla.have_0106 = 1;
  }
  else if (id == 0x0116 && dlc >= 4) {
    /* Rear drive unit torque/status:
       torqueEstimate = signed 12-bit / 2 Nm.
       vehicleSpeed = (raw - 500) / 20 MPH. Store as 0.01 MPH.
       gear = bits 4..6 of byte1. */
    uint16_t tq_raw = (uint16_t)b[0] | (((uint16_t)b[1] & 0x0F) << 8);
    int16_t tq_signed = sign_extend_12(tq_raw);
    tesla.torque_est_Nm = (int16_t)(tq_signed / 2);

    uint16_t sp_raw = (uint16_t)b[2] | (((uint16_t)b[3] & 0x0F) << 8);
    tesla.rear_speed_cmph = (int16_t)(((int32_t)sp_raw - 500L) * 5L); /* /20 mph = *5 hundredths */
    tesla.gear = (b[1] & 0x70) >> 4;
    tesla.have_0116 = 1;
  }
}

static void fmt_fixed1(char *out, int32_t value10, const char *unit) {
  char sign = 0;
  if (value10 < 0) {
    sign = '-';
    value10 = -value10;
  }
  if (sign) {
    snprintf(out, 24, "-%ld.%ld%s", (long)(value10 / 10), (long)(value10 % 10), unit);
  } else {
    snprintf(out, 24, "%ld.%ld%s", (long)(value10 / 10), (long)(value10 % 10), unit);
  }
}

static void fmt_fixed2u(char *out, uint32_t value100, const char *unit) {
  snprintf(out, 24, "%lu.%02lu%s", (unsigned long)(value100 / 100), (unsigned long)(value100 % 100), unit);
}

/* ================= PARTIAL SCREEN UPDATE HELPERS ================= */

#define VALUE_X       120
#define VALUE_W       196
#define VALUE_H       20
#define BIG_VALUE_H   30

enum ValueSlot {
  SLOT_SPEED = 0,
  SLOT_RX,
  SLOT_SOC_UI,
  SLOT_SOC_MIN,
  SLOT_PACK_V,
  SLOT_PACK_A,
  SLOT_DISCH_MAX,
  SLOT_REGEN_MAX,
  SLOT_NOM_REM,
  SLOT_NOM_FULL,
  SLOT_BUFFER,
  SLOT_MOTOR_RPM,
  SLOT_PEDAL,
  SLOT_GEAR,
  SLOT_TORQUE,
  SLOT_LAST_ID,
  SLOT_COUNT
};

static char screen_cache[SLOT_COUNT][28];

static void clear_screen_cache(void) {
  for (uint8_t i = 0; i < SLOT_COUNT; i++) {
    screen_cache[i][0] = 0;
  }
}

static uint8_t cache_text_same(uint8_t slot, const char *txt) {
  const char *oldtxt = screen_cache[slot];
  uint8_t i = 0;

  while (i < (sizeof(screen_cache[slot]) - 1)) {
    char a = oldtxt[i];
    char b = txt[i];

    if (a != b) {
      return 0;
    }

    if (a == 0 || b == 0) {
      return 1;
    }

    i++;
  }

  return 1;
}

static void cache_text_copy(uint8_t slot, const char *txt) {
  uint8_t i = 0;

  while (i < (sizeof(screen_cache[slot]) - 1) && txt[i] != 0) {
    screen_cache[slot][i] = txt[i];
    i++;
  }

  screen_cache[slot][i] = 0;
}

static uint8_t text_changed(uint8_t slot, const char *txt) {
  if (cache_text_same(slot, txt)) {
    return 0;
  }

  cache_text_copy(slot, txt);
  return 1;
}

static void draw_value_if_changed(uint8_t slot, uint16_t x, uint16_t y, uint16_t w, uint16_t h,
                                  const char *txt, uint16_t color, uint8_t scale) {
  if (!text_changed(slot, txt)) {
    return;
  }

  lcd_draw_text_box(x, y, w, h, txt, color, C_BLACK, scale);
}

static void draw_value_blank_once(uint8_t slot, uint16_t x, uint16_t y, uint16_t w, uint16_t h,
                                  uint16_t color, uint8_t scale) {
  draw_value_if_changed(slot, x, y, w, h, "---", color, scale);
}

static void draw_static_labels(void) {
  lcd_fill_screen(C_BLACK);

  lcd_fill_rect(0, 0, 320, 36, C_BLUE);
  lcd_draw_text(8, 8, "TESLA CAN3 DASH", C_WHITE, C_BLUE, 2);

#if TESLA_CAN_FILTERING
  lcd_draw_text(152, 42, "FILTER ON", C_GREEN, C_BLACK, 1);
#else
  lcd_draw_text(152, 42, "FILTER OFF", C_YELLOW, C_BLACK, 1);
#endif

  lcd_fill_rect(0, 58, 320, 2, C_DARK);

  lcd_draw_text(8, 64,  "RX", C_CYAN, C_BLACK, 1);
  lcd_draw_text(8, 82,  "SPEED", C_CYAN, C_BLACK, 1);

  lcd_fill_rect(0, 118, 320, 2, C_DARK);

  lcd_draw_text(8, 128, "SOC UI", C_CYAN, C_BLACK, 1);
  lcd_draw_text(8, 154, "SOC MIN", C_CYAN, C_BLACK, 1);
  lcd_draw_text(8, 184, "PACK V", C_CYAN, C_BLACK, 1);
  lcd_draw_text(8, 210, "PACK A", C_CYAN, C_BLACK, 1);
  lcd_draw_text(8, 240, "DISCH MAX", C_CYAN, C_BLACK, 1);
  lcd_draw_text(8, 266, "REGEN MAX", C_CYAN, C_BLACK, 1);
  lcd_draw_text(8, 296, "NOM REM", C_CYAN, C_BLACK, 1);
  lcd_draw_text(8, 322, "NOM FULL", C_CYAN, C_BLACK, 1);
  lcd_draw_text(8, 348, "BUFFER", C_CYAN, C_BLACK, 1);
  lcd_draw_text(8, 378, "MOTOR RPM", C_CYAN, C_BLACK, 1);
  lcd_draw_text(8, 404, "PEDAL", C_CYAN, C_BLACK, 1);
  lcd_draw_text(8, 434, "GEAR", C_CYAN, C_BLACK, 1);
  lcd_draw_text(8, 456, "TORQUE", C_CYAN, C_BLACK, 1);

  clear_screen_cache();
}

static void draw_can_error_partial(void) {
  char buf[40];
  char hx[12];

  static uint8_t error_screen_drawn = 0;
  if (!error_screen_drawn) {
    lcd_fill_rect(0, 64, 320, 416, C_BLACK);
    lcd_draw_text(8, 72, "CAN INIT ERR", C_RED, C_BLACK, 2);
    error_screen_drawn = 1;
    clear_screen_cache();
  }

  snprintf(buf, sizeof(buf), "STAGE %u", can_fail_stage);
  draw_value_if_changed(SLOT_RX, 8, 104, 300, 20, buf, C_RED, 2);

  hex_u32(hx, can_dbg_msts, 8);
  snprintf(buf, sizeof(buf), "MSTS:%s", hx);
  draw_value_if_changed(SLOT_SOC_UI, 8, 138, 300, 12, buf, C_YELLOW, 1);

  hex_u32(hx, can_dbg_mctrl, 8);
  snprintf(buf, sizeof(buf), "MCTRL:%s", hx);
  draw_value_if_changed(SLOT_SOC_MIN, 8, 158, 300, 12, buf, C_YELLOW, 1);

  hex_u32(hx, can_dbg_btmg, 8);
  snprintf(buf, sizeof(buf), "BTMG:%s", hx);
  draw_value_if_changed(SLOT_PACK_V, 8, 178, 300, 12, buf, C_YELLOW, 1);

  hex_u32(hx, can_dbg_apb1, 8);
  snprintf(buf, sizeof(buf), "APB1:%s", hx);
  draw_value_if_changed(SLOT_PACK_A, 8, 198, 300, 12, buf, C_YELLOW, 1);

  snprintf(buf, sizeof(buf), "PA11 RX LEVEL:%u", can_raw_rx_level);
  draw_value_if_changed(SLOT_DISCH_MAX, 8, 218, 300, 12, buf, C_YELLOW, 1);
}


/* ================= SCREEN FORMAT HELPERS ================= */

static void hex_u32(char *out, uint32_t value, uint8_t digits) {
  static const char hex[] = "0123456789ABCDEF";
  for (int8_t i = digits - 1; i >= 0; i--) {
    out[i] = hex[value & 0x0F];
    value >>= 4;
  }
  out[digits] = 0;
}

static void draw_static_screen(void) {
  draw_static_labels();
}

/* Partial update: this function does NOT clear the full screen. */
static void draw_tesla_dashboard(void) {
  char buf[40];
  char val[24];

  if (!can_ok) {
    draw_can_error_partial();
    return;
  }

  snprintf(buf, sizeof(buf), "%lu", (unsigned long)tesla.rx_total);
  draw_value_if_changed(SLOT_RX, VALUE_X, 64, VALUE_W, VALUE_H, buf, C_WHITE, 1);

  if (tesla.have_0256) {
    snprintf(buf, sizeof(buf), "%u %s", tesla.speed_display, tesla.speed_unit_kph ? "KPH" : "MPH");
    draw_value_if_changed(SLOT_SPEED, 96, 76, 220, BIG_VALUE_H, buf, C_GREEN, 3);
  } else {
    draw_value_blank_once(SLOT_SPEED, 96, 76, 220, BIG_VALUE_H, C_YELLOW, 3);
  }

  if (tesla.have_0302) {
    fmt_fixed1(val, tesla.soc_ui_dP, "%");
    draw_value_if_changed(SLOT_SOC_UI, VALUE_X, 128, VALUE_W, VALUE_H, val, C_GREEN, 2);

    fmt_fixed1(val, tesla.soc_min_dP, "%");
    draw_value_if_changed(SLOT_SOC_MIN, VALUE_X, 154, VALUE_W, VALUE_H, val, C_WHITE, 2);
  } else {
    draw_value_blank_once(SLOT_SOC_UI, VALUE_X, 128, VALUE_W, VALUE_H, C_YELLOW, 2);
    draw_value_blank_once(SLOT_SOC_MIN, VALUE_X, 154, VALUE_W, VALUE_H, C_YELLOW, 2);
  }

  if (tesla.have_0102) {
    fmt_fixed2u(val, tesla.pack_voltage_cV, "V");
    draw_value_if_changed(SLOT_PACK_V, VALUE_X, 184, VALUE_W, VALUE_H, val, C_WHITE, 2);

    fmt_fixed1(val, tesla.pack_current_dA, "A");
    draw_value_if_changed(SLOT_PACK_A, VALUE_X, 210, VALUE_W, VALUE_H, val, C_WHITE, 2);
  } else {
    draw_value_blank_once(SLOT_PACK_V, VALUE_X, 184, VALUE_W, VALUE_H, C_YELLOW, 2);
    draw_value_blank_once(SLOT_PACK_A, VALUE_X, 210, VALUE_W, VALUE_H, C_YELLOW, 2);
  }

  if (tesla.have_0232) {
    fmt_fixed2u(val, tesla.max_discharge_cKW, "KW");
    draw_value_if_changed(SLOT_DISCH_MAX, VALUE_X, 240, VALUE_W, VALUE_H, val, C_GREEN, 2);

    fmt_fixed2u(val, tesla.max_regen_cKW, "KW");
    draw_value_if_changed(SLOT_REGEN_MAX, VALUE_X, 266, VALUE_W, VALUE_H, val, C_GREEN, 2);
  } else {
    draw_value_blank_once(SLOT_DISCH_MAX, VALUE_X, 240, VALUE_W, VALUE_H, C_YELLOW, 2);
    draw_value_blank_once(SLOT_REGEN_MAX, VALUE_X, 266, VALUE_W, VALUE_H, C_YELLOW, 2);
  }

  if (tesla.have_0382) {
    fmt_fixed1(val, tesla.nominal_remain_dkWh, "KWH");
    draw_value_if_changed(SLOT_NOM_REM, VALUE_X, 296, VALUE_W, VALUE_H, val, C_WHITE, 2);

    fmt_fixed1(val, tesla.nominal_full_dkWh, "KWH");
    draw_value_if_changed(SLOT_NOM_FULL, VALUE_X, 322, VALUE_W, VALUE_H, val, C_WHITE, 2);

    fmt_fixed1(val, tesla.energy_buffer_dkWh, "KWH");
    draw_value_if_changed(SLOT_BUFFER, VALUE_X, 348, VALUE_W, VALUE_H, val, C_WHITE, 2);
  } else {
    draw_value_blank_once(SLOT_NOM_REM, VALUE_X, 296, VALUE_W, VALUE_H, C_YELLOW, 2);
    draw_value_blank_once(SLOT_NOM_FULL, VALUE_X, 322, VALUE_W, VALUE_H, C_YELLOW, 2);
    draw_value_blank_once(SLOT_BUFFER, VALUE_X, 348, VALUE_W, VALUE_H, C_YELLOW, 2);
  }

  if (tesla.have_0106) {
    snprintf(val, sizeof(val), "%d", tesla.motor_rpm);
    draw_value_if_changed(SLOT_MOTOR_RPM, VALUE_X, 378, VALUE_W, VALUE_H, val, C_WHITE, 2);

    fmt_fixed1(val, tesla.pedal_dP, "%");
    draw_value_if_changed(SLOT_PEDAL, VALUE_X, 404, VALUE_W, VALUE_H, val, C_WHITE, 2);
  } else {
    draw_value_blank_once(SLOT_MOTOR_RPM, VALUE_X, 378, VALUE_W, VALUE_H, C_YELLOW, 2);
    draw_value_blank_once(SLOT_PEDAL, VALUE_X, 404, VALUE_W, VALUE_H, C_YELLOW, 2);
  }

  if (tesla.have_0116) {
    snprintf(val, sizeof(val), "%s", gear_name(tesla.gear));
    draw_value_if_changed(SLOT_GEAR, VALUE_X, 434, VALUE_W, VALUE_H, val, C_GREEN, 2);

    snprintf(val, sizeof(val), "%dNM", tesla.torque_est_Nm);
    draw_value_if_changed(SLOT_TORQUE, VALUE_X, 456, VALUE_W, VALUE_H, val, C_WHITE, 2);
  } else {
    draw_value_blank_once(SLOT_GEAR, VALUE_X, 434, VALUE_W, VALUE_H, C_YELLOW, 2);
    draw_value_blank_once(SLOT_TORQUE, VALUE_X, 456, VALUE_W, VALUE_H, C_YELLOW, 2);
  }
}


/* ================= ARDUINO ENTRY POINTS ================= */

void setup() {
  clock_48mhz_begin();
  lcd_begin();
  draw_static_screen();

  can_ok = can_begin_read_only(CAN_BITRATE_KBPS);

  if (can_ok) {
    lcd_draw_text(8, 64, "CAN INIT OK", C_GREEN, C_BLACK, 2);
  } else {
    lcd_draw_text(8, 64, "CAN INIT FAILED", C_RED, C_BLACK, 2);
  }

  draw_tesla_dashboard();
}

void loop() {
  while (can_read_latest()) {
    /* read all queued frames; last_frame is updated by can_read_latest() */
    tesla_decode_frame();
  }

  static uint32_t last_redraw = 0;
  uint32_t now = millis();

  /*
    Partial-update mode:
      - Redraw immediately when a decoded CAN value changes.
      - Also refresh every 500 ms so RX count/error diagnostics stay current.
  */
  if (tesla_dirty || ((now - last_redraw) >= 500)) {
    tesla_dirty = 0;
    last_redraw = now;
    draw_tesla_dashboard();
  }

  delay(2);
}
