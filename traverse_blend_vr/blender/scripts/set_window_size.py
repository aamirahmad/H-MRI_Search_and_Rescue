import bge

w = bge.render.getWindowWidth()
h = bge.render.getWindowHeight()

print ("screen size ",w,h)

co = bge.logic.getCurrentController()

co.owner["screen_width"] = w
co.owner["screen_height"] = h
