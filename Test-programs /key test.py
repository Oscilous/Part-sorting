from pynput.keyboard import Key, Listener
  
def show(key):
    
    if key == Key.tab:
        print("good")
          
    if key != Key.tab:
        print("try again")
          
    # by pressing 'delete' button 
    # you can terminate the loop 
    if key == Key.delete: 
        return False
  
# Collect all event until released
with Listener(on_press = show) as listener:
    listener.join()
    
    

root = Tk()
prompt = '      Press any key      '
label1 = Label(root, text=prompt, width=len(prompt), bg='yellow')
label1.pack()

def key(event):
    if event.char == event.keysym:
        msg = 'Normal Key %r' % event.char
        print("servo left")
        pwm.ChangeDutyCycle(2.0) # rotate to 0 degrees
        time.sleep(0.5)
        pwm.ChangeDutyCycle(7.0) # rotate to 90 degrees
        time.sleep(0.5)
        pwm.ChangeDutyCycle(0)
    elif len(event.char) == 1:
        msg = 'Punctuation Key %r (%r)' % (event.keysym, event.char)
        pwm.ChangeDutyCycle(12.0) # rotate to 0 degrees
        time.sleep(0.5)
        pwm.ChangeDutyCycle(7.0) # rotate to 90 degrees
        time.sleep(0.5)
        pwm.ChangeDutyCycle(0)

root.bind_all('<Key>', key)

root.mainloop()