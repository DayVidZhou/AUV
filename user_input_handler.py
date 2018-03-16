import curses

IDLE = 0x00
FWD_SURGE = 0x01
RWD_SURGE = 0x02
UP_HEAVE = 0x03
DWN_HEAVE = 0x04
YAW_LEFT = 0x05
YAW_RIGHT = 0x06
SHUT_DWN = 0xff
SCREEN_X = 80
SCREEN_Y = 25
cmd = IDLE

def input_handler(stdscr):
    stdscr.clear()
    stdscr.refresh()
    stdscr.addstr(SCREEN_Y,SCREEN_X,"AUTONOMOUS MODE...", curses.A_BLINK)
    first_input = stdscr.getch()
    curses.halfdelay(3)
    while True:
        k = stdscr.getch()
        if (k == curses.KEY_UP):
            cmd = FWD_SURGE
            stdscr.clear()
            stdscr.addstr(SCREEN_Y,SCREEN_X,"SURGE FWD")
        if (k == curses.KEY_DOWN):
            cmd = RWD_SURGE
            stdscr.clear()
            stdscr.addstr(SCREEN_Y,SCREEN_X,"SURGE RWD")
        if (k == curses.KEY_LEFT):
            cmd = YAW_LEFT
            stdscr.clear()
            stdscr.addstr(SCREEN_Y,SCREEN_X,"YAW LEFT")
        if (k == curses.KEY_RIGHT):
            cmd = YAW_RIGHT
            stdscr.clear()
            stdscr.addstr(SCREEN_Y,SCREEN_X,"YAW RIGHT")
        if (k == ord('w')):
            cmd = UP_HEAVE
            stdscr.clear()
            stdscr.addstr(SCREEN_Y,SCREEN_X,"HEAVE UP")
        if (k == ord('d')):
            cmd = DWN_HEAVE
            stdscr.clear()
            stdscr.addstr(SCREEN_Y,SCREEN_X,"HEAVE DWN")
        if (k == curses.ERR):
            cmd = IDLE
            stdscr.clear()
            stdscr.addstr(SCREEN_Y,SCREEN_X,"IDLE")
        if(k == ord('q')):
            cmd = SHUT_DWN
            break;

curses.wrapper(input_handler)
