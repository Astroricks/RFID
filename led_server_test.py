from bbio import *
from BBIOServer import *

server = BBIOServer()

SWITCH = GPIO1_16

def setup():
  pinMode(SWITCH, INPUT)
  page = Page("Server test")
  page.add_text("This is a test of the BBIOServer library.")
  page.add_heading("LED Toggle")
  page.add_button(lambda: toggle(USR3), "Toggle USR3", newline=True)
  page.add_monitor(lambda: pinState(USR3), "Current state:")
  page.add_heading("Switch State")
  page.add_monitor(lambda: digitalRead(GPIO1_16), "Switch state:")
  server.start(page)
  stop()

def loop():
  pass
 
run(setup, loop)

