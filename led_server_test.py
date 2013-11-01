from bbio import *
from BBIOServer import *

server = BBIOServer()

def setup():
  page = Page("Server test")
  page.add_text("This is a test of the BBIOServer library.")
  page.add_button(lambda: toggle(USR3), "Toggle USR3", newline=True)
  page.add_monitor(lambda: pinState(USR3), "Current state:")
  server.start(page)
  stop()

def loop():
  pass

run(setup, loop)
