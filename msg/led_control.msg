# LED control: control a single or multiple LED's.
# These are the externally visible LED's, not the board LED's

# colors
uint8 COLOR_OFF = 0 # this is only used in the drivers
uint8 COLOR_RED = 1
uint8 COLOR_GREEN = 2
uint8 COLOR_BLUE = 3
uint8 COLOR_YELLOW = 4
uint8 COLOR_PURPLE = 5
uint8 COLOR_AMBER = 6
uint8 COLOR_CYAN = 7
uint8 COLOR_WHITE = 8

# LED modes definitions
uint8 MODE_OFF = 0 # turn LED off
uint8 MODE_ON = 1  # turn LED on
uint8 MODE_DISABLED = 2  # disable this priority (switch to lower priority setting)
uint8 MODE_BLINK_SLOW = 3
uint8 MODE_BLINK_NORMAL = 4
uint8 MODE_BLINK_FAST = 5
uint8 MODE_BREATHE = 6 # continuously increase & decrease brightness (solid color if driver does not support it)
uint8 MODE_FLASH = 7 # two fast blinks (on/off) with timing as in MODE_BLINK_FAST and then off for a while

uint8 MAX_PRIORITY = 2 # maxium priority (minimum is 0)


uint8 led_mask # LED 제어를 위한 bitmask
uint8 color # color 설정
uint8 mode # mode 설정
uint8 num_blinks # 깜빡이는 횟수
uint8 priority # 우선순위
