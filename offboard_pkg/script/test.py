import Tkinter
import threading
import time

def call(event):
    global ch5, ch6, ch7, ch8, ch9
    k = event.keysym
    if k == "m":
        ch6 = 1
    elif k == "h":
        ch7 = 1
    elif k == "o":
        ch8 = 1
    elif k == "p":
        ch8 = 0
    elif k == "c":
        ch9 = (ch9 + 1) % 2
    time.sleep(0.02)

def read_kbd_input():
    win = Tkinter.Tk()
    frame = Tkinter.Frame(win,width=100,height=60)
    frame.bind("<Key>",call)
    frame.focus_set()
    frame.pack()
    win.mainloop()

if __name__ == '__main__':
    inputThread = threading.Thread(target=read_kbd_input)
    inputThread.start()
    
    ch5, ch6, ch7, ch8, ch9 = 0, 0, 0, 0, 1
    while True:
        print(ch5, ch6, ch7, ch8, ch9)
        time.sleep(0.1)