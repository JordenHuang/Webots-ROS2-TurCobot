import platform

# getch() implementation
# The resources that helped me to implement:
# https://gist.github.com/jfktrey/8928865
# https://gist.github.com/SpotlightKid/816e12fc9dd7003f5c6b41ce0a633de2
# https://stackoverflow.com/questions/6179537/python-wait-x-secs-for-a-key-and-continue-execution-if-not-pressed
GETCH_DEFAULT_TIMEOUT = 0.20
def getch(timeout=0.20):
    if platform.system() == "Windows":
        import msvcrt, time
        startTime = time.time()
        c = -1
        while True:
            if msvcrt.kbhit():
                c = msvcrt.getch()
                c = str(c, encoding='ascii')
            elif time.time() - startTime > timeout:
                return c
    else:
        import sys, select, tty, termios, time
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try:
            startTime = time.time()
            tty.setcbreak(fd)
            rlist, _, _ = select.select([fd], [], [], timeout)
            c = -1  # c would be -1 or char that was read within timeout
            if fd in rlist:
                c = sys.stdin.read(1)
            # Wait until timeout, then return the value of variable c
            while True:
                if time.time() - startTime > timeout:
                    return c
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)


class Colors:
    reset = "\033[0m"

    class fg:
        black   = "\033[30m"
        red     = "\033[31m"
        green   = "\033[32m"
        yellow  = "\033[33m"
        blue    = "\033[34m"
        magenta = "\033[35m"
        cyan    = "\033[36m"
        white   = "\033[37m"

        lightblack   = "\033[90m"
        lightred     = "\033[91m"
        lightgreen   = "\033[92m"
        lightyellow  = "\033[93m"
        lightblue    = "\033[94m"
        lightmagenta = "\033[95m"
        lightcyan    = "\033[96m"
        lightwhite   = "\033[97m"

    class bg:
        black   = "\033[40m"
        red     = "\033[41m"
        green   = "\033[42m"
        yellow  = "\033[43m"
        blue    = "\033[44m"
        magenta = "\033[45m"
        cyan    = "\033[46m"
        white   = "\033[47m"

        lightblack   = "\033[100m"
        lightred     = "\033[101m"
        lightgreen   = "\033[102m"
        lightyellow  = "\033[103m"
        lightblue    = "\033[104m"
        lightmagenta = "\033[105m"
        lightcyan    = "\033[106m"
        lightwhite   = "\033[107m"



def move_to(y:int, x:int):
    '''
        Top left is (0, 0)
    '''
    print(f"\033[{y};{x}H", end='')

def put_text(y:int, x:int, text:str, color:Colors=''):
    move_to(y, x)
    if color != '':
        print(f"{color}{text}{Colors.reset}", end='')
    else:
        print(f"{text}", end='')

def clear_screen():
    print("\033[2J", end='')

def start_alt_buffer():
    print("\033[?1049h", end='')

def end_alt_buffer():
    print("\033[?1049l", end='')

def cursor_visible():
    print("\033[?25h", end='')

def cursor_invisible():
    print("\033[?25l", end='')


if __name__ == '__main__':
    import time
    import sys
    put_text(15, 15, "Hello world", Colors.bg.red)
    start_alt_buffer()
    clear_screen()
    put_text(5, 5, "Hello world", Colors.fg.red)
    sys.stdout.flush()
    time.sleep(2)
    end_alt_buffer()
